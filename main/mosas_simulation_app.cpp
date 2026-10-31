#include <mosas/app/simulation_runner.hpp>
#include <csignal>
#include <iostream>
#include <thread>
namespace {volatile std::sig_atomic_t cancelled=0;void signal_handler(int){cancelled=1;}}
int main(int argc,char** argv){
    if(argc<2){std::cerr<<"usage: mosas_simulation_app <simulation.conf> [--audit-directory <absolute-path> --trial-id <id>]\n";return 2;}
    std::optional<mosas::adapters::SimulationAuditOptions> audit_options;
    try {
        std::vector<std::string> arguments;
        for(int i=2;i<argc;++i)arguments.emplace_back(argv[i]);
        audit_options=mosas::adapters::parse_simulation_audit_options(arguments);
    } catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 2;}
    mosas::app::SimulationConfig config;std::string error;
    if(!mosas::app::SimulationConfigLoader::load(argv[1],&config,&error)){std::cerr<<error<<'\n';return 2;}
    std::signal(SIGINT,signal_handler);std::signal(SIGTERM,signal_handler);
    std::cout<<"waiting for "<<config.socket_directory<<"; source="<<config.app_config_path<<"; launch_speed_mps="<<config.app.algorithms.attitude.launch_speed_mps<<" (calibrated initial speed, not estimated flight speed)\n"<<std::flush;
    mosas::app::SimulationRunner runner(config,[](const std::string& message){std::cout<<message<<'\n'<<std::flush;},audit_options);
    while(!cancelled){runner.step();std::this_thread::sleep_for(std::chrono::milliseconds(5));}
    runner.stop();return 0;
}
