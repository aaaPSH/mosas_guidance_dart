#include <mosas/guidance/command_output_mapper.hpp>
#include "body_vector_test_support.hpp"
#include "command_output_test_support.hpp"
#include "test_support.hpp"
#include <cmath>
#include <limits>

namespace {
/** @return 与当前单调时钟匹配的完整已发射映射上下文。 */
mosas::domain::CommandOutputContext context(double roll = 0) {
    using namespace mosas;
    domain::CommandOutputContext c;
    c.generation = c.active_generation = 3;
    c.source_frame_id = 7;
    c.source_time = {1'000'000'000, 1'000'000'000, 0, domain::ClockDomain::monotonic,
                     domain::TimestampSource::hardware_sync, domain::TimestampQuality::hardware_synchronized};
    c.request_valid = c.launched = c.target_found = c.output_enabled = true;
    c.invalid_reason = domain::GuidanceInvalidReason::none;
    c.emit_time_ns = 1'000'200'000;
    c.clock_domain = domain::ClockDomain::monotonic;
    c.maximum_attitude_age_ns = 50'000'000;
    c.maximum_source_age_ns = 2'000'000'000;
    domain::FlightStateSnapshot state;
    state.time = c.source_time;
    state.time.monotonic_ns = state.time.source_time_ns = 999'900'000;
    state.attitude = test::vector_test_attitude(0, roll);
    state.attitude.time = state.time;
    state.launch.phase = domain::LaunchPhase::launched;
    state.launch.transition_time = c.source_time;
    state.launch.generation = 3;
    c.latest_state = state;
    return c;
}

/** @return 正常 PNG 在两种时间轴上检查曝光总年龄，包含边界和无效零请求时返回 0。 */
int exposure_age_boundaries() {
    using namespace mosas;
    guidance::CommandOutputMapper mapper;
    for (const auto clock : {domain::ClockDomain::monotonic, domain::ClockDomain::simulation}) {
        auto c = context();
        c.clock_domain = clock;
        c.maximum_source_age_ns = 5'000'000;
        c.source_time = test::command_test_time(1'000'000'000, clock);
        c.latest_state->launch.transition_time = c.source_time;
        c.emit_time_ns = c.source_time.monotonic_ns + c.maximum_source_age_ns;
        c.latest_state->time = test::command_test_time(c.emit_time_ns - 1, clock);
        c.latest_state->attitude.time = c.latest_state->time;
        CHECK(mapper.map({0, 0.2, 0}, c).command.valid);
        ++c.emit_time_ns;
        const auto stale = mapper.map({0, 0.2, 0}, c).command;
        CHECK(!stale.valid && stale.launched && !stale.target_found);
        CHECK(stale.invalid_reason == domain::GuidanceInvalidReason::source_too_old);
        CHECK(stale.overload_g.y == 0 && stale.overload_g.z == 0);
        c.request_valid = false;
        c.invalid_reason = domain::GuidanceInvalidReason::invalid_rate;
        CHECK(!mapper.map({}, c).command.target_found); // 首帧无效速率不能保留陈旧识别位。
        c.request_valid = true;
        c.source_time = test::command_test_time(c.emit_time_ns + 1, clock);
        const auto future = mapper.map({0, 0.2, 0}, c).command;
        CHECK(!future.valid && future.invalid_reason == domain::GuidanceInvalidReason::future_source_time);
        c.source_time = {};
        const auto missing = mapper.map({0, 0.2, 0}, c).command;
        CHECK(!missing.target_found && missing.invalid_reason == domain::GuidanceInvalidReason::invalid_source_time);
    }
    // 固定模式只按原有触发契约工作，不引入视觉年龄或姿态有效性要求。
    for (const auto mode : {guidance::GuidanceDebugMode::launch_delay,
                            guidance::GuidanceDebugMode::target_detected}) {
        guidance::CommandMappingConfig config;
        config.debug.mode = mode;
        config.debug.pitch_overload_g = 0.2;
        auto c = context();
        c.maximum_source_age_ns = 1;
        c.emit_time_ns += 1'000'000'000;
        c.latest_state->attitude.valid = false;
        const auto command = guidance::CommandOutputMapper(config).map({}, c).command;
        CHECK(command.valid && command.target_found);
        CHECK_NEAR(command.overload_g.y, 0.2, 1e-12);
    }
    return 0;
}

/** @return 限幅前诊断使用同次映射的机体系选轴结果，不受两级限幅、量化或后续映射影响时返回 0。 */
int unlimited_body_diagnostics() {
    using namespace mosas;
    guidance::CommandMappingConfig config;
    config.maximum_command_overload = 0.3;
    config.maximum_command_acceleration = 0.2 * config.gravity;
    auto c = context(std::acos(-1.0) / 2);
    c.overload_step_g = 0.01;
    for (const auto channel : {guidance::ControlChannel::both,
                              guidance::ControlChannel::pitch,
                              guidance::ControlChannel::yaw}) {
        config.control_channel = channel;
        const auto mapped = guidance::CommandOutputMapper(config).map({0, 3, -4}, c);
        CHECK(mapped.command.valid && mapped.trace.unlimited_body_overload);
        const auto& raw = *mapped.trace.unlimited_body_overload;
        CHECK(raw.x == 0);
        CHECK_NEAR(raw.y, channel == guidance::ControlChannel::yaw ? 0 : -4, 1e-12);
        CHECK_NEAR(raw.z, channel == guidance::ControlChannel::pitch ? 0 : -3, 1e-12);
        CHECK(std::hypot(mapped.command.overload_g.y, mapped.command.overload_g.z) <= 0.2);
    }
    config.control_channel = guidance::ControlChannel::both;
    guidance::CommandOutputMapper mapper(config);
    c = context();
    c.overload_step_g = 0.01;
    const auto rounded = mapper.map({0, 0.0049, -0.0049}, c);
    CHECK(rounded.trace.unlimited_body_overload);
    CHECK_NEAR(rounded.trace.unlimited_body_overload->y, 0.0049, 1e-12);
    CHECK_NEAR(rounded.trace.unlimited_body_overload->z, -0.0049, 1e-12);
    CHECK(rounded.command.overload_g.y == 0 && rounded.command.overload_g.z == 0);
    const auto zero = mapper.map({}, c);
    CHECK(zero.command.valid && zero.trace.unlimited_body_overload);
    CHECK(zero.trace.unlimited_body_overload->y == 0 && zero.trace.unlimited_body_overload->z == 0);
    c.overload_step_g = -0.01; // 已完成旋转但量化失败，也不能留下有效的限幅前诊断。
    const auto rejected = mapper.map({0, 3, -4}, c);
    CHECK(!rejected.command.valid && !rejected.trace.unlimited_body_overload);
    c = context();
    c.request_valid = false;
    CHECK(!mapper.map({0, 3, -4}, c).trace.unlimited_body_overload);

    for (const auto mode : {guidance::GuidanceDebugMode::launch_delay,
                            guidance::GuidanceDebugMode::target_detected}) {
        config.debug.mode = mode;
        config.debug.trigger_delay_ms = 0;
        config.debug.pitch_overload_g = 3;
        config.debug.yaw_overload_g = -4;
        config.control_channel = guidance::ControlChannel::pitch;
        const auto mapped = guidance::CommandOutputMapper(config).map({0, 9, 8}, context());
        CHECK(mapped.command.valid && mapped.trace.unlimited_body_overload);
        CHECK(mapped.trace.unlimited_body_overload->y == 3);
        CHECK(mapped.trace.unlimited_body_overload->z == 0);
        CHECK_NEAR(mapped.command.overload_g.y, 0.2, 1e-12);
    }
    return 0;
}

/** @return 量化保持通道选择、坐标诊断及上限，连续输出不损失精度时返回 0。 */
int quantization_boundaries() {
    using namespace mosas;
    guidance::CommandMappingConfig config;
    config.maximum_command_overload = 0.3;
    auto c = context();
    guidance::CommandOutputMapper mapper(config);
    CHECK_NEAR(mapper.map({0, 0.0049, 0}, c).command.overload_g.y, 0.0049, 1e-12);
    c.overload_step_g = 0.01;
    auto mapped = mapper.map({0, 0.0049, -0.0049}, c);
    CHECK(mapped.command.overload_g.y == 0 && mapped.command.overload_g.z == 0);
    CHECK(!std::signbit(mapped.command.overload_g.y) && !std::signbit(mapped.command.overload_g.z));
    mapped = mapper.map({0, 0.005, -0.005}, c);
    CHECK_NEAR(mapped.command.overload_g.y, 0.01, 1e-12);
    CHECK_NEAR(mapped.command.overload_g.z, -0.01, 1e-12);
    mapped = mapper.map({0, 0.075, -0.075}, c);
    CHECK_NEAR(mapped.command.overload_g.y, 0.08, 1e-12);
    CHECK_NEAR(mapped.command.overload_g.z, -0.08, 1e-12);
    mapped = mapper.map({0, 0.299, 0.024}, c);
    CHECK(mapped.command.valid && mapped.trace.overload_limited);
    CHECK_NEAR(mapped.command.overload_g.y, 0.29, 1e-12);
    CHECK_NEAR(mapped.command.overload_g.z, 0.02, 1e-12);
    CHECK_NEAR(mapped.trace.command_phase.value, std::atan2(0.02, 0.29), 1e-12);

    config.maximum_command_acceleration = 0.21 * config.gravity;
    config.control_channel = guidance::ControlChannel::pitch;
    auto rolled = context(std::acos(-1.0) / 4);
    rolled.overload_step_g = 0.01;
    mapped = guidance::CommandOutputMapper(config).map({0, 1, 0}, rolled);
    CHECK(mapped.command.valid && mapped.command.overload_g.z == 0);
    CHECK_NEAR(mapped.command.overload_g.y, 0.21, 1e-12);
    CHECK_NEAR(mapped.trace.body_acceleration.y, 0.21 * config.gravity, 1e-12);
    CHECK_NEAR(mapped.trace.ballistic_acceleration.y, 0.21 * config.gravity / std::sqrt(2.0), 1e-12);
    CHECK_NEAR(mapped.trace.ballistic_acceleration.z, 0.21 * config.gravity / std::sqrt(2.0), 1e-12);
    CHECK_NEAR(mapped.trace.command_phase.value, std::acos(-1.0) / 4, 1e-12);
    for (const double step : {-0.01, std::numeric_limits<double>::infinity(),
                              std::numeric_limits<double>::quiet_NaN()}) {
        c.overload_step_g = step;
        CHECK(!mapper.map({0, 0.2, 0}, c).command.valid);
    }
    return 0;
}

/** @return 正常模式独立发布 IMU 发射状态，且只有发射后视觉请求可启用识别和过载时返回 0。 */
int normal_ascent_without_visual_request() {
    using namespace mosas;
    guidance::CommandOutputMapper mapper;
    auto c = context();
    c.launched = c.request_valid = c.target_found = false;
    c.expired = true;
    c.source_time = {};
    c.latest_state->launch.phase = domain::LaunchPhase::pre_launch;
    auto result = mapper.map({}, c);
    CHECK(!result.command.launched && !result.command.target_found && !result.command.valid);

    c.latest_state->launch.phase = domain::LaunchPhase::launched;
    result = mapper.map({}, c);
    CHECK(result.command.launched && !result.command.target_found && !result.command.valid);
    CHECK(result.command.overload_g.y == 0 && result.command.overload_g.z == 0);
    for (int invalid = 0; invalid < 4; ++invalid) {
        auto rejected = c;
        if (invalid == 0) rejected.latest_state.reset();
        if (invalid == 1) rejected.latest_state->launch.phase = domain::LaunchPhase::pre_launch;
        if (invalid == 2) ++rejected.latest_state->launch.generation;
        if (invalid == 3) ++rejected.generation;
        const auto command = mapper.map({}, rejected).command;
        CHECK(!command.launched && !command.target_found && !command.valid);
        CHECK(command.overload_g.y == 0 && command.overload_g.z == 0);
    }

    // 发射前的历史识别不能借最新 IMU 状态启用识别位或非零控制。
    c.request_valid = c.target_found = true;
    c.expired = false;
    c.source_time = c.latest_state->time;
    result = mapper.map({0, 0.25, -0.5}, c);
    CHECK(result.command.launched && !result.command.target_found && !result.command.valid);
    CHECK(result.command.overload_g.y == 0 && result.command.overload_g.z == 0);

    c.launched = true;
    c.request_valid = false;
    c.invalid_reason = domain::GuidanceInvalidReason::invalid_rate;
    result = mapper.map({}, c);
    CHECK(result.command.launched && result.command.target_found && !result.command.valid);
    CHECK(result.command.overload_g.y == 0 && result.command.overload_g.z == 0);
    c.request_valid = true;
    result = mapper.map({0, 0.25, -0.5}, c);
    CHECK(result.command.launched && result.command.target_found && result.command.valid);
    CHECK_NEAR(result.command.overload_g.y, 0.25, 1e-12);
    CHECK_NEAR(result.command.overload_g.z, -0.5, 1e-12);

    c.expired = true;
    result = mapper.map({0, 0.25, -0.5}, c);
    CHECK(result.command.launched && !result.command.target_found && !result.command.valid);
    CHECK(result.command.overload_g.y == 0 && result.command.overload_g.z == 0);
    c.latest_state->launch.phase = domain::LaunchPhase::pre_launch;
    result = mapper.map({0, 0.25, -0.5}, c);
    CHECK(!result.command.launched && !result.command.target_found && !result.command.valid);
    return 0;
}

/** @return 发射前等待不消耗延迟、视觉不提前接管且空邮箱仍能触发时返回 0。 */
int launch_delay_without_visual_request() {
    using namespace mosas;
    guidance::CommandMappingConfig config;
    config.debug.mode = guidance::GuidanceDebugMode::launch_delay;
    config.debug.trigger_delay_ms = 200;
    config.debug.duration_ms = 100;
    config.debug.yaw_overload_g = 0.5;
    guidance::CommandOutputMapper mapper(config);
    auto c = context();
    c.launched = c.request_valid = c.target_found = false;
    c.expired = true;
    c.source_time = {};
    c.latest_state->launch.phase = domain::LaunchPhase::pre_launch;
    c.emit_time_ns = 1'000'000'000;
    CHECK(!mapper.map({}, c).command.launched);
    c.emit_time_ns = 6'000'000'000; // 程序启动后等待 5 秒才发射。
    CHECK(!mapper.map({}, c).command.launched);
    c.latest_state->launch.phase = domain::LaunchPhase::launched;
    c.latest_state->launch.transition_time.monotonic_ns = c.emit_time_ns;
    c.latest_state->launch.transition_time.source_time_ns = c.emit_time_ns;
    auto result = mapper.map({}, c);
    CHECK(!result.command.valid && result.command.launched && !result.command.target_found);
    c.emit_time_ns = 6'199'999'999;
    c.launched = c.target_found = true;
    c.expired = false;
    result = mapper.map({}, c);
    CHECK(!result.command.valid && result.command.launched && !result.command.target_found);
    CHECK(result.command.overload_g.y == 0 && result.command.overload_g.z == 0);
    c.launched = c.target_found = false;
    c.expired = true;
    c.emit_time_ns = 6'200'000'000;
    result = mapper.map({}, c);
    CHECK(result.command.valid && result.command.launched && result.command.target_found);
    CHECK_NEAR(result.command.overload_g.z, 0.5, 1e-12);
    CHECK(result.command.source_time.monotonic_ns == c.latest_state->time.monotonic_ns);
    for (int invalid = 0; invalid < 8; ++invalid) {
        auto rejected = c;
        if (invalid == 0) rejected.latest_state.reset();
        if (invalid == 1) rejected.latest_state->launch.phase = domain::LaunchPhase::pre_launch;
        if (invalid == 2) ++rejected.latest_state->launch.generation;
        if (invalid == 3) ++rejected.generation;
        if (invalid == 4) rejected.latest_state->launch.transition_time = {};
        if (invalid == 5) rejected.latest_state->launch.transition_time.monotonic_ns = rejected.emit_time_ns + 1;
        if (invalid == 6) rejected.clock_domain = domain::ClockDomain::simulation;
        if (invalid == 7) rejected.output_enabled = false;
        const auto command = mapper.map({}, rejected).command;
        CHECK(!command.valid);
        CHECK(command.overload_g.y == 0 && command.overload_g.z == 0);
    }
    c.emit_time_ns = 6'300'000'000;
    result = mapper.map({}, c);
    CHECK(!result.command.valid && result.command.launched && !result.command.target_found);
    c.latest_state->launch.transition_time.monotonic_ns = 0;
    c.emit_time_ns = 200'000'000;
    CHECK(mapper.map({}, c).command.valid); // Time zero is a valid launch timestamp.
    return 0;
}

/** @return 无视觉命令时仍发送上升状态，且仅未过期的发射后识别能首次触发时返回 0。 */
int target_ascent_without_visual_request() {
    using namespace mosas;
    guidance::CommandMappingConfig config;
    config.debug.mode = guidance::GuidanceDebugMode::target_detected;
    config.debug.yaw_overload_g = 0.5;
    guidance::CommandOutputMapper mapper(config);
    auto c = context();
    c.launched = c.request_valid = c.target_found = false;
    c.expired = true;
    c.source_time = {};
    c.latest_state->launch.phase = domain::LaunchPhase::pre_launch;
    CHECK(!mapper.map({}, c).command.launched);
    c.latest_state->launch.phase = domain::LaunchPhase::launched;
    auto result = mapper.map({}, c);
    CHECK(!result.command.valid && result.command.launched && !result.command.target_found);
    CHECK(result.command.overload_g.y == 0 && result.command.overload_g.z == 0);
    for (int invalid = 0; invalid < 5; ++invalid) {
        auto rejected = c;
        rejected.launched = rejected.target_found = true;
        rejected.expired = false;
        if (invalid == 0) rejected.latest_state.reset();
        if (invalid == 1) rejected.latest_state->launch.phase = domain::LaunchPhase::pre_launch;
        if (invalid == 2) ++rejected.latest_state->launch.generation;
        if (invalid == 3) ++rejected.generation;
        if (invalid == 4) rejected.output_enabled = false;
        const auto command = mapper.map({}, rejected).command;
        CHECK(!command.valid && !command.target_found);
        CHECK(command.overload_g.y == 0 && command.overload_g.z == 0);
    }
    c.launched = c.target_found = true;
    c.source_time = c.latest_state->time;
    CHECK(!mapper.map({}, c).command.target_found); // 已过期的识别结果不能首次触发。
    c.expired = false;
    c.launched = false;
    CHECK(!mapper.map({}, c).command.target_found); // 发射前图像不能借最新 IMU 状态触发。
    c.launched = true;
    c.invalid_reason = domain::GuidanceInvalidReason::invalid_rate;
    result = mapper.map({}, c); // 首次识别无需等待 LOS 速率或 PNG 请求有效。
    CHECK(result.command.valid && result.command.launched && result.command.target_found);
    CHECK_NEAR(result.command.overload_g.z, 0.5, 1e-12);
    c.target_found = false;
    c.expired = true;
    c.emit_time_ns += 200'000'000;
    CHECK(mapper.map({}, c).command.valid); // 触发后不因目标丢失或邮箱超时撤销锁存。
    return 0;
}

/** @return Zero if explicit resets are idempotent and target-trigger time zero is not a sentinel. */
int target_trigger_state_boundaries() {
    using namespace mosas;
    guidance::CommandMappingConfig config;
    config.debug.mode = guidance::GuidanceDebugMode::target_detected;
    config.debug.duration_ms = 100;
    config.debug.yaw_overload_g = 0.5;
    guidance::CommandOutputMapper mapper(config);
    auto c = context();
    c.emit_time_ns = 0;
    CHECK(mapper.map({}, c).command.valid);
    c.target_found = false;
    c.emit_time_ns = 50'000'000;
    CHECK(mapper.map({}, c).command.valid);
    mapper.reset();
    mapper.reset();
    CHECK(!mapper.map({}, c).command.valid);
    CHECK(!mapper.map({}, c).command.target_found);
    c.target_found = true;
    CHECK(mapper.map({}, c).command.valid);
    c.target_found = false;
    c.emit_time_ns = 49'000'000;
    CHECK(!mapper.map({}, c).command.valid);
    c.emit_time_ns = 149'000'000;
    CHECK(mapper.map({}, c).command.valid);
    c.emit_time_ns = 150'000'000;
    CHECK(!mapper.map({}, c).command.valid);
    return 0;
}
}

int main() {
    using namespace mosas;
    CHECK(exposure_age_boundaries() == 0);
    CHECK(unlimited_body_diagnostics() == 0);
    CHECK(quantization_boundaries() == 0);
    CHECK(normal_ascent_without_visual_request() == 0);
    guidance::CommandMappingConfig config;
    config.maximum_command_overload = 10;
    config.maximum_command_acceleration = std::numeric_limits<double>::infinity();
    guidance::CommandOutputMapper mapper(config);
    auto c = context();
    auto mapped = mapper.map({0, 1, 0}, c);
    CHECK(mapped.command.valid); // 姿态早于曝光 100us，不应误拒绝。
    CHECK_NEAR(mapped.command.overload_g.y, 1, 1e-12);
    c.latest_state->attitude.time.monotonic_ns = c.emit_time_ns - c.maximum_attitude_age_ns;
    c.latest_state->time = c.latest_state->attitude.time;
    CHECK(mapper.map({0, 1, 0}, c).command.valid);
    --c.latest_state->attitude.time.monotonic_ns;
    c.latest_state->time = c.latest_state->attitude.time;
    const auto stale_attitude = mapper.map({0, 1, 0}, c).command;
    CHECK(!stale_attitude.valid);
    CHECK(stale_attitude.invalid_reason == domain::GuidanceInvalidReason::attitude_too_old);
    c = context();
    c.latest_state->attitude.time.monotonic_ns = c.emit_time_ns + 1;
    c.latest_state->time = c.latest_state->attitude.time;
    CHECK(mapper.map({0, 1, 0}, c).command.valid);
    const domain::GuidanceInvalidReason rejected_reasons[]{
        domain::GuidanceInvalidReason::imu_initializing,
        domain::GuidanceInvalidReason::invalid_attitude,
        domain::GuidanceInvalidReason::generation_mismatch,
        domain::GuidanceInvalidReason::generation_mismatch,
        domain::GuidanceInvalidReason::launch_not_confirmed,
        domain::GuidanceInvalidReason::output_disabled,
        domain::GuidanceInvalidReason::invalid_source_time,
        domain::GuidanceInvalidReason::invalid_attitude,
    };
    for (int bad = 0; bad < 8; ++bad) {
        c = context();
        if (bad == 0) c.latest_state.reset();
        if (bad == 1) c.latest_state->attitude.valid = false;
        if (bad == 2) ++c.generation;
        if (bad == 3) ++c.latest_state->launch.generation;
        if (bad == 4) c.launched = false;
        if (bad == 5) c.output_enabled = false;
        if (bad == 6) c.clock_domain = domain::ClockDomain::simulation;
        if (bad == 7) c.latest_state->attitude.navigation_from_dart = domain::Rotation3<domain::NavigationFrameTag, domain::DartFrameTag>({-1,0,0,0,1,0,0,0,1});
        const auto result = mapper.map({0, 4, 2}, c);
        CHECK(!result.command.valid);
        CHECK(result.command.invalid_reason == rejected_reasons[bad]);
        CHECK(result.command.overload_g.y == 0 && result.command.overload_g.z == 0);
    }
    c = context();
    c.request_valid = false;
    c.invalid_reason = domain::GuidanceInvalidReason::invalid_rate;
    mapped = mapper.map({0, 9, 8}, c);
    CHECK(!mapped.command.valid && mapped.command.target_found);
    CHECK(mapped.command.invalid_reason == domain::GuidanceInvalidReason::invalid_rate);
    CHECK(mapped.command.overload_g.y == 0 && mapped.command.overload_g.z == 0);
    c.expired = true;
    const auto expired = mapper.map({0, 9, 8}, c).command;
    CHECK(!expired.target_found);
    CHECK(expired.invalid_reason == domain::GuidanceInvalidReason::command_expired);

    const double pi = std::acos(-1.0);
    config.control_channel = guidance::ControlChannel::pitch;
    guidance::CommandOutputMapper pitch(config);
    mapped = pitch.map({0, 1, 0}, context(pi / 4));
    CHECK(mapped.command.valid);
    CHECK_NEAR(mapped.command.overload_g.y, std::sqrt(0.5), 1e-12);
    CHECK(mapped.command.overload_g.z == 0);
    CHECK_NEAR(mapped.trace.after_overload_limit.y, 0.5, 1e-12);
    CHECK_NEAR(mapped.trace.after_overload_limit.z, 0.5, 1e-12);
    config.control_channel = guidance::ControlChannel::yaw;
    mapped = guidance::CommandOutputMapper(config).map({0, 1, 0}, context(pi / 4));
    CHECK(mapped.command.overload_g.y == 0);
    CHECK_NEAR(mapped.trace.after_overload_limit.y, 0.5, 1e-12);
    CHECK_NEAR(mapped.trace.after_overload_limit.z, -0.5, 1e-12);

    config.control_channel = guidance::ControlChannel::both;
    config.maximum_command_overload = 2;
    config.maximum_command_acceleration = 9.80665;
    mapped = guidance::CommandOutputMapper(config).map({0, 3, 4}, context(-pi / 3));
    CHECK(mapped.command.valid);
    CHECK(mapped.trace.overload_limited && mapped.trace.acceleration_limited);
    CHECK_NEAR(mapped.trace.after_overload_limit.y, 1.2, 1e-12);
    CHECK_NEAR(mapped.trace.after_overload_limit.z, 1.6, 1e-12);
    CHECK_NEAR(mapped.trace.ballistic_acceleration.y, 0.6 * 9.80665, 1e-12);
    CHECK_NEAR(mapped.trace.ballistic_acceleration.z, 0.8 * 9.80665, 1e-12);
    CHECK_NEAR(mapped.trace.command_phase.value, std::atan2(0.8, 0.6), 1e-12);
    for (int cap = 0; cap < 2; ++cap) {
        auto zero = config;
        if (cap == 0) zero.maximum_command_overload = 0;
        else zero.maximum_command_acceleration = 0;
        mapped = guidance::CommandOutputMapper(zero).map({0, 1e-15, -1e-15}, context());
        CHECK(mapped.command.valid);
        CHECK(mapped.command.overload_g.y == 0 && mapped.command.overload_g.z == 0);
        CHECK(mapped.trace.command_phase.value == 0);
    }
    c = context();
    c.latest_state->attitude = test::vector_test_attitude(pi / 2, 0.7);
    c.latest_state->attitude.time = c.latest_state->time;
    mapped = mapper.map({0, 1, 2}, c);
    CHECK(mapped.command.valid);
    CHECK_NEAR(mapped.command.overload_g.y, 1, 1e-12);
    CHECK_NEAR(mapped.command.overload_g.z, 2, 1e-12);

    // =========================================================================
    // 调试模式测试：launch_delay
    // =========================================================================
    {
        guidance::CommandMappingConfig debug_cfg;
        debug_cfg.debug.mode = guidance::GuidanceDebugMode::launch_delay;
        debug_cfg.debug.trigger_delay_ms = 200;
        debug_cfg.debug.yaw_overload_g = 0.5;
        debug_cfg.debug.pitch_overload_g = -0.3;
        debug_cfg.debug.duration_ms = 100;
        guidance::CommandOutputMapper debug_mapper(debug_cfg);

        auto dc = context();
        dc.latest_state->launch.transition_time.monotonic_ns = 1'000'000'000;
        dc.target_found = false;

        // 1. 发射前：应输出零控制且 launched=false, target_found=false
        dc.launched = false;
        dc.latest_state->launch.phase = domain::LaunchPhase::pre_launch;
        dc.emit_time_ns = 1'050'000'000;
        auto res = debug_mapper.map({0, 0, 0}, dc);
        CHECK(!res.command.valid);
        CHECK(!res.command.launched);
        CHECK(!res.command.target_found);
        CHECK(res.command.overload_g.y == 0 && res.command.overload_g.z == 0);

        // 2. 发射后但未到延迟时间 (100ms < 200ms)：保持中立舵
        dc.launched = true;
        dc.latest_state->launch.phase = domain::LaunchPhase::launched;
        dc.emit_time_ns = 1'100'000'000; // +100ms
        res = debug_mapper.map({0, 0, 0}, dc);
        CHECK(!res.command.valid);
        CHECK(res.command.launched);
        CHECK(!res.command.target_found);
        CHECK(res.command.overload_g.y == 0 && res.command.overload_g.z == 0);

        // 3. 延迟到达 (200ms)：触发固定指令，且 target_found 置 1（通知下位机执行）
        dc.emit_time_ns = 1'200'000'000; // +200ms
        res = debug_mapper.map({0, 0, 0}, dc);
        CHECK(res.command.valid);
        CHECK(res.command.launched);
        CHECK(res.command.target_found); // 下位机识别标志位要求置 1
        CHECK_NEAR(res.command.overload_g.y, -0.3, 1e-12);
        CHECK_NEAR(res.command.overload_g.z, 0.5, 1e-12);

        // 4. 持续时间内 (+250ms)：保持固定指令
        dc.emit_time_ns = 1'250'000'000;
        res = debug_mapper.map({0, 0, 0}, dc);
        CHECK(res.command.valid);
        CHECK(res.command.target_found);
        CHECK_NEAR(res.command.overload_g.y, -0.3, 1e-12);
        CHECK_NEAR(res.command.overload_g.z, 0.5, 1e-12);

        // 5. 持续时间结束 (+350ms >= 200ms + 100ms)：恢复回零
        dc.emit_time_ns = 1'350'000'000;
        res = debug_mapper.map({0, 0, 0}, dc);
        CHECK(!res.command.valid);
        CHECK(res.command.launched);
        CHECK(res.command.overload_g.y == 0 && res.command.overload_g.z == 0);
    }

    // =========================================================================
    // 调试模式测试：target_detected
    // =========================================================================
    {
        guidance::CommandMappingConfig debug_cfg;
        debug_cfg.debug.mode = guidance::GuidanceDebugMode::target_detected;
        debug_cfg.debug.yaw_overload_g = 0.4;
        debug_cfg.debug.pitch_overload_g = 0.2;
        debug_cfg.debug.duration_ms = 0; // 0 表示持续到结束
        guidance::CommandOutputMapper debug_mapper(debug_cfg);

        auto dc = context();
        dc.emit_time_ns = 1'100'000'000;
        dc.target_found = false;

        // 1. 识别前：保持中立舵
        auto res = debug_mapper.map({0, 0, 0}, dc);
        CHECK(!res.command.valid);
        CHECK(!res.command.target_found);
        CHECK(res.command.overload_g.y == 0 && res.command.overload_g.z == 0);

        // 2. 识别到目标：触发固定指令
        dc.emit_time_ns = 1'150'000'000;
        dc.target_found = true;
        res = debug_mapper.map({0, 0, 0}, dc);
        CHECK(res.command.valid);
        CHECK(res.command.target_found);
        CHECK_NEAR(res.command.overload_g.y, 0.2, 1e-12);
        CHECK_NEAR(res.command.overload_g.z, 0.4, 1e-12);

        // 3. 后续帧即使目标丢失，也应锁定并持续输出
        dc.emit_time_ns = 1'200'000'000;
        dc.target_found = false;
        res = debug_mapper.map({0, 0, 0}, dc);
        CHECK(res.command.valid);
        CHECK(res.command.target_found);
        CHECK_NEAR(res.command.overload_g.y, 0.2, 1e-12);
        CHECK_NEAR(res.command.overload_g.z, 0.4, 1e-12);

        // 4. 新世代重置 (generation 改变)：状态应清零
        dc.generation = dc.active_generation = 4;
        dc.latest_state->launch.generation = 4;
        dc.target_found = false;
        res = debug_mapper.map({0, 0, 0}, dc);
        CHECK(!res.command.valid);
        CHECK(!res.command.target_found);
        CHECK(res.command.overload_g.y == 0 && res.command.overload_g.z == 0);
    }

    // =========================================================================
    // 调试模式测试：通道选择与限幅
    // =========================================================================
    {
        guidance::CommandMappingConfig debug_cfg;
        debug_cfg.debug.mode = guidance::GuidanceDebugMode::launch_delay;
        debug_cfg.debug.trigger_delay_ms = 0;
        debug_cfg.debug.yaw_overload_g = 5.0;
        debug_cfg.debug.pitch_overload_g = 5.0;
        debug_cfg.control_channel = guidance::ControlChannel::yaw;
        debug_cfg.maximum_command_overload = 1.0;
        guidance::CommandOutputMapper debug_mapper(debug_cfg);

        auto dc = context();
        dc.emit_time_ns = 1'000'000'000;
        auto res = debug_mapper.map({0, 0, 0}, dc);
        CHECK(res.command.valid);
        CHECK(res.command.overload_g.y == 0); // pitch 被通道选择清零
        CHECK_NEAR(res.command.overload_g.z, 1.0, 1e-12); // yaw 被限幅到 1.0
        CHECK(res.trace.overload_limited);
    }

    CHECK(launch_delay_without_visual_request() == 0);
    CHECK(target_ascent_without_visual_request() == 0);
    CHECK(target_trigger_state_boundaries() == 0);
    return 0;
}
