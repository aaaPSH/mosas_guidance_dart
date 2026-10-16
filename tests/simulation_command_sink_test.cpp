#include <mosas/adapters/simulation_command_sink.hpp>
#include <dart_sim_transport/endpoint.hpp>
#include "test_support.hpp"
#include <filesystem>
#include <unistd.h>
using namespace std::chrono_literals;
int main(){
    using namespace mosas; using namespace dart_sim_transport;
    char path[]="/tmp/mosas-command-XXXXXX";std::string root=mkdtemp(path);
    {
        Endpoint server(root+"/model",{812,0,Session{},CameraInfo{}});
        auto session=std::make_shared<adapters::SimulationSession>(root+"/model");CHECK(session->wait_ready(1s));
        adapters::SimulationCommandSink sink(session);CHECK(sink.start());sink.set_active_generation(7);
        runtime::GuidanceCommand command;command.generation=7;command.valid=true;command.source_frame_id=123;
        command.source_time=domain::make_simulation_time(200);command.overload_g={0,2,3};CHECK(sink.try_submit(command));
        Message message;CHECK(server.receive(Channel::control,message,1s));
        CHECK(message.header.session_id==812 && message.header.sim_time_ns==200);
        auto value=std::get<Overload>(message.payload);CHECK(value.left_g==-3 && value.up_g==2 && value.source_frame_id==123);
        command.generation=812;CHECK(!sink.try_submit(command)); // socket session != MOSAS generation
        command.generation=7;command.source_time.monotonic_ns=201;CHECK(!sink.try_submit(command));
        sink.set_active_generation(8);command.generation=7;CHECK(!sink.try_submit(command));
        command.generation=8;command.valid=false;command.source_time=domain::make_simulation_time(300);CHECK(sink.try_submit(command));
        CHECK(server.receive(Channel::control,message,1s));CHECK(std::get<Overload>(message.payload).valid==0);
        session->stop();CHECK(!sink.try_submit(command));
    }
    std::filesystem::remove_all(root);return 0;
}
