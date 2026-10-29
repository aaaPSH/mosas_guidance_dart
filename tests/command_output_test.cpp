#include <mosas/runtime/command_output.hpp>
#include <mosas/guidance/command_output_mapper.hpp>
#include "body_vector_test_support.hpp"
#include "test_support.hpp"
#include <string>
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
    return 0;
}
