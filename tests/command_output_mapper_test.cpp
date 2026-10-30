#include <mosas/guidance/command_output_mapper.hpp>
#include "body_vector_test_support.hpp"
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

/** @return Zero if launch-delay output needs no visual request and retains its safety gates. */
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
    c.emit_time_ns = 1'100'000'000;
    auto result = mapper.map({}, c);
    CHECK(!result.command.valid && result.command.launched);
    c.emit_time_ns = 1'200'000'000;
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
    c.emit_time_ns = 1'300'000'000;
    CHECK(!mapper.map({}, c).command.valid);
    c.latest_state->launch.transition_time.monotonic_ns = 0;
    c.emit_time_ns = 200'000'000;
    CHECK(mapper.map({}, c).command.valid); // Time zero is a valid launch timestamp.
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
    CHECK(!mapper.map({0, 1, 0}, c).command.valid);
    for (int bad = 0; bad < 9; ++bad) {
        c = context();
        if (bad == 0) c.latest_state.reset();
        if (bad == 1) c.latest_state->attitude.valid = false;
        if (bad == 2) ++c.generation;
        if (bad == 3) ++c.latest_state->launch.generation;
        if (bad == 4) c.launched = false;
        if (bad == 5) c.output_enabled = false;
        if (bad == 6) c.clock_domain = domain::ClockDomain::simulation;
        if (bad == 7) { c.latest_state->attitude.time.monotonic_ns = c.emit_time_ns + 1; c.latest_state->time = c.latest_state->attitude.time; }
        if (bad == 8) c.latest_state->attitude.navigation_from_dart = domain::Rotation3<domain::NavigationFrameTag, domain::DartFrameTag>({-1,0,0,0,1,0,0,0,1});
        const auto result = mapper.map({0, 4, 2}, c);
        CHECK(!result.command.valid);
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
    CHECK(!mapper.map({0, 9, 8}, c).command.target_found);

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
    CHECK(target_trigger_state_boundaries() == 0);
    return 0;
}
