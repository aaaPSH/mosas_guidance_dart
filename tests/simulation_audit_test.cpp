#include <mosas/adapters/simulation_audit.hpp>
#include <mosas/adapters/simulation_command_sink.hpp>
#include <chrono>
#include <mosas/domain/time.hpp>
#include "test_support.hpp"
#include <filesystem>
#include <fstream>
#include <sstream>
#include <unistd.h>

struct AuditFixture {
    std::filesystem::path path;
    AuditFixture(){char name[]="/tmp/mosas-audit-XXXXXX";auto p=mkdtemp(name);if(!p)throw std::runtime_error("mkdtemp");path=p;}
    ~AuditFixture(){std::filesystem::remove_all(path);}
};
int main(){
    using namespace mosas::adapters;
    CHECK(!parse_simulation_audit_options({}));
    auto options=parse_simulation_audit_options({"--audit-directory","/tmp/audit","--trial-id","trial-1"});
    CHECK(options&&options->trial_id=="trial-1");
    for(const auto& args:std::vector<std::vector<std::string>>{{"--trial-id","a"},
        {"--audit-directory","relative","--trial-id","a"},
        {"--audit-directory","/tmp/a","--trial-id","../bad"},
        {"--unknown","a"}}){
        bool rejected=false;try{parse_simulation_audit_options(args);}catch(const std::invalid_argument&){rejected=true;}
        CHECK(rejected);
    }
    AuditFixture fixture;options->directory=fixture.path;
    SimulationAuditMetadata metadata;metadata.source_config="/tmp/original.conf";
    metadata.width=640;metadata.height=480;metadata.intrinsics={486.2,486.5,308.8,244.6};
    metadata.camera_hz=120;metadata.imu_hz=200;metadata.launch_speed_mps=16;metadata.cap_g=.3;
    metadata.parameters={3,3,10,3.5,.0035};
    SimulationAudit audit(*options,5,metadata);
    CHECK(audit.record_lifecycle(AuditLifecycle::ready,0));
    const auto time=mosas::domain::make_simulation_time(1000000000);
    CHECK(audit.record_vision(71,time,2,true,0));
    mosas::domain::GuidanceInput input;input.source_frame_id=71;input.time=time;
    PngCalculationTrace trace;trace.valid=true;trace.raw_navigation_overload={0,.819,-0.0};
    CHECK(audit.record_png(input,trace));
    mosas::runtime::GuidanceCommand command;command.generation=2;command.source_frame_id=71;
    command.source_time=time;command.valid=true;command.overload_g={0,.3,0};
    CHECK(audit.record_selected(command,AuditStrategy::preset));
    dart_sim_transport::Overload wire;wire.valid=1;wire.source_frame_id=71;wire.up_g=.3;
    CHECK(audit.record_submit(command,wire,true));
    audit.stop();auto diagnostics=audit.diagnostics();
    CHECK(diagnostics.healthy&&diagnostics.accepted==5&&diagnostics.written==5);
    CHECK(!audit.record_selected(command,AuditStrategy::png));
    std::ifstream events(fixture.path/"session-5/events.jsonl");std::stringstream text;text<<events.rdbuf();
    CHECK(text.str().find(",-0.0]")!=std::string::npos);
    CHECK(text.str().find("\"generation_source\":\"processed_watermark\"")!=std::string::npos);
    CHECK(text.str().find("\"generation\":null")!=std::string::npos);
    CHECK(text.str().find("\"kind\":\"selected\"")!=std::string::npos);
    CHECK(text.str().find("\"strategy\":\"preset\"")!=std::string::npos);
    CHECK(text.str().find("command_sequence")==std::string::npos); // submit cannot know wire sequence
    bool duplicate=false;try{SimulationAudit other(*options,5,metadata);}catch(const std::exception&){duplicate=true;}
    CHECK(duplicate);
    SimulationAudit next(*options,6,metadata);next.stop();CHECK(next.diagnostics().healthy);
    {
        SimulationAudit actual_wire(*options,8,metadata);
        auto offered=wire;offered.source_frame_id=99;
        CHECK(actual_wire.record_submit(command,offered,true));actual_wire.stop();
        std::ifstream log(fixture.path/"session-8/events.jsonl");std::stringstream rows;rows<<log.rdbuf();
        CHECK(rows.str().find("\"source_frame_id\":99")!=std::string::npos);
        CHECK(rows.str().find("\"source_time_ns\":1000000000")!=std::string::npos);
    }
    SimulationAudit repeated(*options,7,metadata);
    CHECK(repeated.record_submit(command,wire,true));
    CHECK(!repeated.record_submit(command,wire,true));repeated.stop();
    CHECK(!repeated.diagnostics().healthy&&repeated.diagnostics().dropped==1);
    {
        using namespace std::chrono_literals;
        const auto socket=(fixture.path/"socket-model").string();
        dart_sim_transport::Endpoint server(socket,{907,0,dart_sim_transport::Session{},dart_sim_transport::CameraInfo{}});
        auto session=std::make_shared<SimulationSession>(socket);CHECK(session->wait_ready(1s));
        auto recorded=std::make_shared<SimulationAudit>(*options,907,metadata);
        SimulationCommandSink sink(session,recorded);sink.set_active_generation(2);CHECK(sink.start());
        command.source_time=mosas::domain::make_simulation_time(0);command.generation=3;
        CHECK(!sink.try_submit(command));command.generation=2;CHECK(sink.try_submit(command));
        dart_sim_transport::Message received;CHECK(server.receive(dart_sim_transport::Channel::control,received,1s));
        CHECK(received.header.type==dart_sim_transport::Type::overload);
        CHECK(std::get<dart_sim_transport::Overload>(received.payload).source_frame_id==71);
        CHECK_NEAR(std::get<dart_sim_transport::Overload>(received.payload).up_g,.3,0);
        sink.stop();recorded->stop();CHECK(recorded->diagnostics().healthy);
        std::ifstream log(fixture.path/"session-907/events.jsonl");std::stringstream rows;rows<<log.rdbuf();
        CHECK(rows.str().find("\"kind\":\"rejected\"")!=std::string::npos);
        CHECK(rows.str().find("\"submit_accepted\":true")!=std::string::npos);
    }
    return 0;
}
