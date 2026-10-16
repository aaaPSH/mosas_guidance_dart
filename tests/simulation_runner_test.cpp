#include <mosas/app/simulation_runner.hpp>
#include <dart_sim_transport/endpoint.hpp>
#include "test_support.hpp"
#include <filesystem>
#include <fstream>
#include <sstream>
#include <thread>
#include <unistd.h>
using namespace std::chrono_literals;
int main() {
    using namespace mosas; using namespace dart_sim_transport;
    char name[]="/tmp/mosas-runner-XXXXXX"; const std::filesystem::path root=mkdtemp(name);
    struct Cleanup { std::filesystem::path p; ~Cleanup(){std::filesystem::remove_all(p);} } cleanup{root};
    app::SimulationConfig config;
    config.socket_directory=(root/"model").string();
    config.app.camera.width=320; config.app.camera.height=183; config.app.camera.fps=60;
    config.app.guidance.camera_intrinsics={200,200,159.5,91};
    config.app.guidance.launch_speed_mps=25;
    config.app.algorithms.vision_config.initial_roi={0,0,320,183};
    config.app.output.mode=output::OutputMode::debug;
    config.app.output.recording_path=(root/"records/flight.avi").string();
    ServerSnapshot snapshot{501,0,Session{0,Phase::ready,3},CameraInfo{}};
    Endpoint server(config.socket_directory,snapshot);
    std::vector<std::string> log;
    app::SimulationRunner runner(config,[&](const std::string& s){log.push_back(s);});
    auto advance=[&](auto predicate) { auto end=std::chrono::steady_clock::now()+2s;
        while(!predicate() && std::chrono::steady_clock::now()<end){runner.step();std::this_thread::sleep_for(2ms);} return predicate(); };
    CHECK(advance([&]{return runner.running();}));
    snapshot.session.phase=Phase::warmup; snapshot.session.target_speed_mps=30;
    server.set_snapshot(snapshot); CHECK(server.submit(snapshot.session,0));
    CHECK(advance([&]{return !runner.running();}));
    CHECK(runner.last_error().find("launch_speed")!=std::string::npos);
    for(int i=0;i<150;++i){runner.step();std::this_thread::sleep_for(2ms);}
    CHECK(!runner.running());
    snapshot.session_id=502; snapshot.session=Session{0,Phase::ready,3}; server.set_snapshot(snapshot);
    CHECK(advance([&]{return runner.running();})); CHECK(runner.session_id()==502);
    snapshot.session_id=503; snapshot.camera.cx+=2; server.set_snapshot(snapshot);
    CHECK(advance([&]{return !runner.running();}));
    CHECK(advance([&]{return runner.last_error().find("cx")!=std::string::npos;}));
    snapshot.session_id=504; snapshot.camera.cx-=2; snapshot.session.g0_mps2=10; server.set_snapshot(snapshot);
    CHECK(advance([&]{return runner.last_error().find("gravity")!=std::string::npos;}));
    runner.stop(); CHECK(!runner.running()); CHECK(!log.empty());
    auto audited_config=config;
    audited_config.socket_directory=(root/"audited-model").string();
    audited_config.app_config_path=root/"controlled-fixture.conf";
    audited_config.app.output.recording_path=(root/"audited-records/flight.avi").string();
    audited_config.app.algorithms.png_guidance.maximum_command_overload=.3;
    Endpoint audited_server(audited_config.socket_directory,{701,0,Session{0,Phase::ready,3},CameraInfo{}});
    adapters::SimulationAuditOptions options{root/"audit","runner-fixture"};
    app::SimulationRunner audited(audited_config,[](const std::string&){},options);
    const auto deadline=std::chrono::steady_clock::now()+2s;
    while(!audited.running()&&audited.last_error().empty()&&std::chrono::steady_clock::now()<deadline){
        audited.step();std::this_thread::yield();
    }
    CHECK(audited.running());audited.stop();CHECK(audited.last_error().empty());
    CHECK(std::filesystem::is_regular_file(root/"audit/session-701/metadata.json"));
    std::ifstream events(root/"audit/session-701/events.jsonl");std::stringstream audit_text;audit_text<<events.rdbuf();
    CHECK(audit_text.str().find("algorithm_ready")!=std::string::npos);
    CHECK(audit_text.str().find("stopped")!=std::string::npos);
    return 0;
}
