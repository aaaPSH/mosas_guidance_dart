#include <mosas/adapters/simulation_pre_acquisition.hpp>
#include <mosas/adapters/simulation_audit.hpp>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <unistd.h>
#include "test_support.hpp"

#include <limits>
#include <stdexcept>
#include <vector>

using namespace mosas;
namespace {
struct Vision final : ports::IVisionDetector {
    domain::VisionDetection next;
    int calls = 0, resets = 0;
    domain::VisionDetection detect(const domain::ImageView&, std::uint64_t,
                                   const domain::SensorTime&) override {
        ++calls;
        return next;
    }
    void reset() noexcept override { ++resets; }
};
struct Sink final : runtime::CommandSink {
    std::vector<runtime::GuidanceCommand> commands;
    std::uint64_t generation = 0;
    bool active = false, accept = true;
    bool start() override { active = true; return true; }
    bool try_submit(const runtime::GuidanceCommand& command) noexcept override {
        if (!active || !accept || command.generation != generation) return false;
        commands.push_back(command);
        return true;
    }
    void set_active_generation(std::uint64_t value) noexcept override { generation = value; }
    void cancel() noexcept override { active = false; }
    void stop() noexcept override { active = false; }
    Diagnostics diagnostics() const noexcept override { return {true, commands.size(), 0, 0}; }
};
struct Fixture {
    std::optional<runtime::ImuStateSnapshot> flight = runtime::ImuStateSnapshot{};
    adapters::SimulationPreAcquisitionPolicy policy;
    Vision* vision;
    Sink* sink;
    adapters::SimulationPreAcquisitionPolicy::Ports ports;
    explicit Fixture(std::shared_ptr<adapters::SimulationAudit> audit={})
        : policy(0.3,[this]{return flight;},std::move(audit)) {
        auto detector = std::make_unique<Vision>(); vision = detector.get();
        auto output = std::make_unique<Sink>(); sink = output.get();
        ports = policy.wrap(std::move(detector), std::move(output));
        ports.command_sink->set_active_generation(7);
        ports.command_sink->start();
        ports.vision_detector->reset();
        flight->attitude.valid = true;
        flight->launch.generation = 7;
        flight->launch.reason = domain::LaunchReason::threshold_confirmed;
    }
    runtime::GuidanceCommand observe(std::int64_t ns, bool found = false,
                                    domain::ObservationReason reason = domain::ObservationReason::not_found) {
        if (flight) flight->time = domain::make_simulation_time(ns);
        vision->next.observation.valid = found;
        vision->next.observation.reason = found ? domain::ObservationReason::found : reason;
        vision->next.observation.center = {31, 42};
        ports.vision_detector->detect({}, 1000 + ns, domain::make_simulation_time(ns));
        runtime::GuidanceCommand command;
        command.generation = 7;
        command.source_time = domain::make_simulation_time(ns);
        command.invalid_reason = domain::GuidanceInvalidReason::stale_input;
        return command;
    }
    void launch() { flight->launch.phase = domain::LaunchPhase::launched; }
};
int selection_and_first_detection_latch() {
    Fixture f;
    auto command = f.observe(100);
    CHECK(f.ports.command_sink->try_submit(command));
    CHECK(!f.sink->commands.back().valid); // Prelaunch must remain neutral.
    f.launch();
    command = f.observe(110);
    CHECK(f.ports.command_sink->try_submit(command));
    const auto preset = f.sink->commands.back();
    CHECK(preset.valid && preset.invalid_reason == domain::GuidanceInvalidReason::none);
    CHECK(preset.overload_g.x == 0 && preset.overload_g.y == 0.3 && preset.overload_g.z == 0);
    CHECK(preset.generation == 7 && preset.source_frame_id == 1110);
    CHECK(preset.source_time.monotonic_ns == 110);
    CHECK(!command.valid && command.overload_g.y == 0); // Original value is not rewritten.
    CHECK(f.policy.diagnostics().preset_submitted == 1);
    CHECK(f.ports.command_sink->try_submit(command));
    CHECK(!f.sink->commands.back().valid); // An observation is consumed once.
    command = f.observe(120, true);
    CHECK(f.policy.diagnostics().first_detection_ns == 120);
    CHECK(f.ports.command_sink->try_submit(command));
    CHECK(!f.sink->commands.back().valid); // First detection, before LOS-rate initialization.
    command = f.observe(130, true);
    command.valid = true; command.invalid_reason = domain::GuidanceInvalidReason::none;
    command.source_frame_id = 1130; command.overload_g = {0, 0.1, -0.2};
    CHECK(f.ports.command_sink->try_submit(command));
    const auto& passed = f.sink->commands.back();
    CHECK(passed.overload_g.y == 0.1 && passed.overload_g.z == -0.2);
    CHECK(passed.source_frame_id == 1130 && passed.source_time.monotonic_ns == 130);
    command = f.observe(140);
    CHECK(f.ports.command_sink->try_submit(command));
    CHECK(!f.sink->commands.back().valid); // Loss must never re-enable the preset.
    CHECK(f.policy.diagnostics().preset_submitted == 1);
    CHECK(f.policy.diagnostics().first_preset_ns == 110);
    CHECK(f.policy.diagnostics().last_preset_ns == 110);
    CHECK(f.vision->calls == 5 && f.vision->resets == 1);
    return 0;
}
int unsafe_or_unrelated_inputs_stay_neutral() {
    for (int scenario = 0; scenario != 9; ++scenario) {
        Fixture f; f.launch(); auto command = f.observe(200);
        switch (scenario) {
            case 0: command.source_time = domain::make_simulation_time(201); break;
            case 1: command.source_time.source = domain::TimestampSource::hardware_sync; break;
            case 2: command.generation = 8; break;
            case 3: f.ports.command_sink->set_active_generation(8); break;
            case 4: command.invalid_reason = domain::GuidanceInvalidReason::invalid_attitude; break;
            case 5: command.overload_g.y = 1; break;
            case 6: command = f.observe(200, false, domain::ObservationReason::invalid_frame); break;
            case 7: f.flight->attitude.valid = false; command = f.observe(200); break;
            case 8: f.flight.reset(); command = f.observe(200); break;
        }
        f.ports.command_sink->try_submit(command);
        CHECK(f.policy.diagnostics().preset_submitted == 0);
        for (const auto& sent : f.sink->commands) CHECK(!sent.valid);
    }
    Fixture stopped; stopped.launch(); auto command = stopped.observe(300);
    stopped.ports.command_sink->cancel();
    CHECK(!stopped.ports.command_sink->try_submit(command));
    CHECK(stopped.sink->commands.empty());
    CHECK(stopped.ports.command_sink->diagnostics().available);
    return 0;
}
int prelaunch_detection_and_reset() {
    Fixture f;
    f.observe(10, true); // Seeing the target before launch also disables the preset.
    f.ports.command_sink->set_active_generation(8);
    f.flight->launch.generation = 8; f.launch();
    auto command = f.observe(20); command.generation = 8;
    CHECK(f.ports.command_sink->try_submit(command));
    CHECK(!f.sink->commands.back().valid);
    f.ports.vision_detector->reset(); // A new runtime segment may start cleanly.
    command = f.observe(30); command.generation = 8;
    CHECK(f.ports.command_sink->try_submit(command));
    CHECK(f.sink->commands.back().overload_g.y == 0.3);
    CHECK(f.policy.diagnostics().first_detection_ns == -1);
    const auto detection = f.ports.vision_detector->detect({}, 1040, domain::make_simulation_time(40));
    CHECK(detection.observation.center.u == 31 && detection.observation.center.v == 42);
    f.ports.command_sink->stop(); CHECK(!f.sink->active);
    return 0;
}
int audited_selection_preserves_original_results() {
    char name[]="/tmp/mosas-policy-audit-XXXXXX";auto directory=mkdtemp(name);CHECK(directory);
    struct Cleanup{std::filesystem::path path;~Cleanup(){std::filesystem::remove_all(path);}} cleanup{directory};
    adapters::SimulationAuditMetadata metadata;
    metadata.source_config="/tmp/controlled-policy-fixture.conf";metadata.width=2;metadata.height=1;
    metadata.intrinsics={1,1,0,0};metadata.camera_hz=60;metadata.imu_hz=200;
    metadata.launch_speed_mps=16;metadata.cap_g=.3;metadata.parameters={3,3,10,3.5,.0035};
    auto audit=std::make_shared<adapters::SimulationAudit>(
        adapters::SimulationAuditOptions{directory,"policy-fixture"},401,metadata);
    Fixture f(audit);f.launch();
    auto command=f.observe(110);CHECK(f.ports.command_sink->try_submit(command));
    CHECK(!command.valid&&f.sink->commands.back().overload_g.y==.3);
    command=f.observe(120,true);CHECK(f.ports.command_sink->try_submit(command));
    CHECK(!f.sink->commands.back().valid);
    command=f.observe(130,true);command.valid=true;command.invalid_reason=domain::GuidanceInvalidReason::none;
    command.source_frame_id=1130;command.overload_g={0,.1,-.2};
    CHECK(f.ports.command_sink->try_submit(command));CHECK(f.sink->commands.back().overload_g.z==-.2);
    command=f.observe(140);CHECK(f.ports.command_sink->try_submit(command));
    CHECK(!f.sink->commands.back().valid&&f.policy.diagnostics().preset_submitted==1);
    f.ports.command_sink->stop();audit->stop();CHECK(audit->diagnostics().healthy);
    CHECK(audit->diagnostics().written==8);
    std::ifstream events(std::filesystem::path(directory)/"session-401/events.jsonl");
    std::stringstream text;text<<events.rdbuf();
    CHECK(text.str().find("\"strategy\":\"preset\"")!=std::string::npos);
    CHECK(text.str().find("\"strategy\":\"png\"")!=std::string::npos);
    CHECK(text.str().find("\"strategy\":\"neutral\"")!=std::string::npos);
    return 0;
}
int limits_and_failures() {
    for (double value : {0.0, -1.0, std::numeric_limits<double>::infinity(),
                         std::numeric_limits<double>::quiet_NaN()}) {
        bool rejected = false;
        try { adapters::SimulationPreAcquisitionPolicy policy(value, [] { return std::optional<runtime::ImuStateSnapshot>{}; }); }
        catch (const std::invalid_argument&) { rejected = true; }
        CHECK(rejected);
    }
    Fixture f; f.launch(); auto command = f.observe(400); f.sink->accept = false;
    CHECK(!f.ports.command_sink->try_submit(command));
    CHECK(f.policy.diagnostics().preset_submitted == 0);
    return 0;
}
}
int main() {
    CHECK(selection_and_first_detection_latch() == 0);
    CHECK(unsafe_or_unrelated_inputs_stay_neutral() == 0);
    CHECK(prelaunch_detection_and_reset() == 0);
    CHECK(limits_and_failures() == 0);
    CHECK(audited_selection_preserves_original_results() == 0);
    return 0;
}
