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
    state.launch.generation = 3;
    c.latest_state = state;
    return c;
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
    return 0;
}
