#include <mosas/adapters/simulation_audit.hpp>
#include <mosas/guidance/command_output_mapper.hpp>
#include "command_output_test_support.hpp"
#include "test_support.hpp"
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <unistd.h>

namespace {
struct Fixture {
    std::filesystem::path root = std::filesystem::temp_directory_path() /
        ("mosas-audit-v2-" + std::to_string(::getpid()));
    ~Fixture() { std::filesystem::remove_all(root); }
};
std::string read_text(const std::filesystem::path& path) {
    std::ifstream file(path);
    return {std::istreambuf_iterator<char>(file), {}};
}
}
int main() {
    using namespace mosas;
    CHECK(!adapters::parse_simulation_audit_options({}));
    Fixture fixture;
    const adapters::SimulationAuditOptions options{fixture.root, "trial-1"};
    adapters::SimulationAuditMetadata meta;
    meta.source_config = "/config/actual.conf";
    meta.width=640; meta.height=480; meta.camera_hz=120; meta.imu_hz=1000;
    meta.launch_speed_mps=16; meta.cap_g=0;
    meta.parameters={3,4,4,0.0005,1,0.2};
    meta.maximum_command_acceleration=std::numeric_limits<double>::infinity();
    {
        adapters::SimulationAudit audit(options, 42, meta, 64);
        CHECK(audit.healthy()); // cap=0 合法，不得被旧训练审计限制拒绝。
        CHECK(audit.record_lifecycle(adapters::AuditLifecycle::ready, 0));
        CHECK(audit.record_vision(1,domain::make_simulation_time(100),5,true,0));
        domain::GuidanceInput input;
        input.source_frame_id=1; input.time=domain::make_simulation_time(100);
        input.vector_los={true,{1,0,0},{0,0,0}}; input.speed_mps.value=50;
        const auto trace=guidance::evaluate_ballistic_png({1,0,0},{0,0,0},50,3,4,9.80665);
        CHECK(audit.record_png(input,trace));
        domain::GuidanceCommand command;
        command.source_frame_id=1; command.source_time=input.time; command.generation=5;
        command.valid=command.launched=command.target_found=true; command.invalid_reason=domain::GuidanceInvalidReason::none;
        command.overload_g=trace.requested_overload;
        CHECK(audit.record_request(command));
        domain::CommandOutputContext context;
        context.source_frame_id=1; context.generation=context.active_generation=5;
        context.source_time=input.time; context.request_valid=context.launched=context.target_found=context.output_enabled=true;
        context.clock_domain=domain::ClockDomain::simulation; context.maximum_attitude_age_ns=1000;
        context.emit_time_ns=200; context.latest_state=test::command_test_state(190,5,0.3,domain::ClockDomain::simulation);
        guidance::CommandMappingConfig config; config.maximum_command_overload=0;
        config.maximum_command_acceleration=meta.maximum_command_acceleration;
        domain::CommandEmissionSnapshot emission;
        emission.attempt_id=1; emission.mapping=guidance::CommandOutputMapper(config).map(command.overload_g,context);
        dart_sim_transport::Overload wire;
        wire.source_frame_id=1; wire.valid=1; wire.left_g=0; wire.up_g=0;
        CHECK(audit.record_submit(emission,wire,true));
        emission.attempt_id=2;
        CHECK(audit.record_submit(emission,wire,true)); // 同一帧的新输出尝试不是重复帧错误。
        command.valid=false;command.source_frame_id=2;command.overload_g={};
        command.invalid_reason=domain::GuidanceInvalidReason::invalid_rate;
        CHECK(audit.record_request(command));
        CHECK(audit.record_lifecycle(adapters::AuditLifecycle::stopped,300));
        audit.stop();
        CHECK(audit.healthy());
        CHECK(audit.diagnostics().accepted==audit.diagnostics().written);
        CHECK(audit.diagnostics().dropped==0);
    }
    const auto metadata=read_text(fixture.root/"session-42"/"metadata.json");
    const auto events=read_text(fixture.root/"session-42"/"events.jsonl");
    CHECK(metadata.find("\"schema_version\":2")!=std::string::npos);
    CHECK(metadata.find("\"maximum_command_acceleration_mps2\":null")!=std::string::npos);
    CHECK(events.find("\"kind\":\"request\"")!=std::string::npos);
    CHECK(events.find("\"target_found\":true")!=std::string::npos);
    CHECK(events.find("\"ballistic_request_g\":[0,0,0]")!=std::string::npos);
    CHECK(events.find("\"output_attempt_id\":2")!=std::string::npos);
    CHECK(events.find("\"attitude_time_ns\":190")!=std::string::npos);
    CHECK(events.find("\"emit_time_ns\":200")!=std::string::npos);
    CHECK(events.find("\"ballistic_acceleration_mps2\"")!=std::string::npos);
    CHECK(events.find("preset")==std::string::npos);
    CHECK(events.find("\"healthy\":true")!=std::string::npos);
    bool refused=false;
    try { adapters::SimulationAudit duplicate(options,42,meta); } catch (const std::exception&) { refused=true; }
    CHECK(refused);
    for (const auto& args : std::vector<std::vector<std::string>>{{"--trial-id","x"},{"--unknown","x"},
            {"--audit-directory","relative","--trial-id","x"}}) {
        refused=false;
        try { adapters::parse_simulation_audit_options(args); } catch (const std::exception&) { refused=true; }
        CHECK(refused);
    }
    return 0;
}
