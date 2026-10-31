#include <mosas/runtime/command_output.hpp>
#include <mosas/guidance/command_output_mapper.hpp>
#include "body_vector_test_support.hpp"
#include "test_support.hpp"
#include <string>

namespace {
/** @return Zero if session reset and invalidation clear triggers without conflating attempt numbering. */
int debug_session_lifecycle() {
    using namespace mosas;
    guidance::CommandMappingConfig config;
    config.debug.mode = guidance::GuidanceDebugMode::target_detected;
    config.debug.yaw_overload_g = 0.5;
    config.debug.duration_ms = 100;
    auto now = std::int64_t{1'000'000'000};
    domain::FlightStateSnapshot state;
    state.launch.phase = domain::LaunchPhase::launched;
    runtime::CommandOutputBinding binding;
    binding.mapper = std::make_unique<guidance::CommandOutputMapper>(config);
    binding.latest_state = [&] { return std::optional<domain::FlightStateSnapshot>(state); };
    binding.now_ns = [&] { return now; };
    binding.output_enabled = [] { return true; };
    binding.clock_domain = domain::ClockDomain::monotonic;
    binding.maximum_attitude_age_ns = 50'000'000;
    runtime::CommandOutputProcessor processor;
    CHECK(processor.bind(std::move(binding)));
    domain::GuidanceCommand request;
    request.launched = request.target_found = true;
    auto attempt = processor.prepare(request, 0, false);
    CHECK(attempt.mapping.command.valid);
    processor.complete(attempt, domain::CommandEmissionDisposition::serial_written);
    request.target_found = false;
    now += 20'000'000;
    CHECK(processor.prepare(request, 0, false).mapping.command.valid);

    processor.reset();
    CHECK(!processor.latest_emission());
    state.launch.phase = domain::LaunchPhase::pre_launch;
    CHECK(!processor.prepare(request, 0, false).mapping.command.valid);
    state.launch.phase = domain::LaunchPhase::launched;
    attempt = processor.prepare(request, 0, false);
    CHECK(!attempt.mapping.command.valid && !attempt.mapping.command.target_found);
    CHECK(attempt.mapping.command.overload_g.z == 0);

    now += 200'000'000;
    request.target_found = true;
    attempt = processor.prepare(request, 0, false);
    CHECK(attempt.mapping.command.valid); // A new session starts a new duration, not the old expired one.
    processor.complete(attempt, domain::CommandEmissionDisposition::serial_written);
    request.target_found = false;
    const auto previous_id = attempt.attempt_id;
    processor.invalidate();
    CHECK(!processor.latest_emission());
    attempt = processor.prepare(request, 0, false);
    CHECK(attempt.attempt_id > previous_id);
    CHECK(!attempt.mapping.command.valid && !attempt.mapping.command.target_found);
    processor.reset();
    CHECK(processor.prepare(request, 0, false).attempt_id == 1);
    return 0;
}
}

int main() {
    using namespace mosas;
    runtime::CommandOutputProcessor processor;
    CHECK(!processor.bind({}));
    std::string reads;
    domain::FlightStateSnapshot state;
    state.time = {1'000,1'000,0,domain::ClockDomain::monotonic,domain::TimestampSource::hardware_sync,domain::TimestampQuality::hardware_synchronized};
    state.attitude = test::vector_test_attitude(); state.attitude.time = state.time;
    state.launch.phase = domain::LaunchPhase::launched;
    runtime::CommandOutputBinding binding;
    binding.mapper = std::make_unique<guidance::CommandOutputMapper>();
    binding.latest_state = [&]() { reads += 'S'; return std::optional<domain::FlightStateSnapshot>(state); };
    binding.now_ns = [&]() { reads += 'C'; return 2'000; };
    binding.output_enabled = [] { return true; };
    binding.clock_domain = domain::ClockDomain::monotonic;
    binding.maximum_attitude_age_ns = 50'000;
    CHECK(processor.bind(std::move(binding)));
    domain::GuidanceCommand command;
    command.source_frame_id = 5; command.source_time = state.time;
    command.valid = command.launched = command.target_found = true;
    command.overload_g = {0, 0.5, 0};
    auto first = processor.prepare(command, 0, false);
    CHECK(reads == "SC");
    CHECK(first.mapping.command.valid);
    CHECK(!processor.latest_emission()); // prepare 不冒充写出成功。
    processor.complete(first, domain::CommandEmissionDisposition::serial_written);
    CHECK(processor.latest_emission()->attempt_id == first.attempt_id);
    state.attitude = test::vector_test_attitude(0,std::acos(-1.0)/2);
    state.attitude.time = state.time;
    const auto second = processor.prepare(command, 0, false);
    CHECK(second.attempt_id > first.attempt_id);
    CHECK_NEAR(second.mapping.command.overload_g.z, -0.5, 1e-12);
    CHECK_NEAR(second.mapping.command.overload_g.y, 0, 1e-12);
    CHECK(!processor.prepare(command,0,true).mapping.command.target_found);
    CHECK(!processor.prepare(command,1,false).mapping.command.valid);
    processor.reset();
    CHECK(!processor.latest_emission());
    CHECK(debug_session_lifecycle() == 0);
    return 0;
}
