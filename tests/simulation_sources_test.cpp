#include <mosas/adapters/simulation_sources.hpp>
#include <dart_sim_transport/endpoint.hpp>
#include "test_support.hpp"
#include <filesystem>
#include <future>
#include <thread>
#include <unistd.h>
using namespace std::chrono_literals;
using namespace dart_sim_transport;
struct Fixture {
    std::string directory;
    Fixture(){char n[]="/tmp/mosas-source-XXXXXX";directory=mkdtemp(n);}
    ~Fixture(){std::filesystem::remove_all(directory);}
};
int main(){
    using namespace mosas;
    Fixture fixture; CameraInfo info; info.width=2; info.height=1; info.stride=6;
    info.R_S_from_C={0,-1,0,1,0,0,0,0,1};
    Endpoint server(fixture.directory+"/model",{99,0,Session{0,Phase::ready,3},info});
    auto session=std::make_shared<adapters::SimulationSession>(fixture.directory+"/model");
    CHECK(session->wait_ready(1s)); adapters::SimulationImuSource imu(session); adapters::SimulationCameraSource camera(session); adapters::SimulationClock clock(session);
    CHECK(imu.start().status==runtime::SourceStatus::ok); CHECK(camera.configure({}).status==runtime::SourceStatus::ok);
    // Leave rate measurement to Runtime; nominal handshake Hz is not actual capture throughput.
    CHECK(camera.capture_fps()==0.0);
    auto rotation=adapters::simulation_camera_rotation(info.R_S_from_C).matrix();
    CHECK(rotation[2]==1 && rotation[6]==-1 && rotation[4]==1);
    session->bind_watermark([]{runtime::ImuStateSnapshot state;state.time=domain::make_simulation_time(100'000'000);state.attitude.valid=true;return std::optional<runtime::ImuStateSnapshot>{state};});
    CHECK(server.submit(Imu{3,{1,2,3},{4,5,6}},5'000'000));
    CHECK(imu.wait_for_data()); runtime::ImuAngularVelocitySample sample;CHECK(imu.try_pop(&sample));
    CHECK(sample.acceleration.x==1 && sample.acceleration.y==3 && sample.acceleration.z==-2);
    CHECK(sample.angular_velocity.y==6 && sample.angular_velocity.z==-5);
    CHECK(sample.sensor_time->source==domain::TimestampSource::simulation);
    CHECK(server.submit(Image{2,1,6,PixelFormat::rgb8,{255,0,0,0,0,255}},4'000'000)); // cross-channel older is legal
    runtime::CameraFrame frame;CHECK(camera.capture(&frame).status==runtime::SourceStatus::ok);
    CHECK(frame.image.format==domain::PixelFormat::rgb8); CHECK(frame.image.is_valid());
    CHECK(camera.prepare(&frame).status==runtime::SourceStatus::ok);
    CHECK(frame.image.format==domain::PixelFormat::bgr8 && frame.image.data[0]==0 && frame.image.data[2]==255 && frame.image.data[3]==255);
    CHECK(clock.now_ns()==5'000'000); std::this_thread::sleep_for(120ms);CHECK(clock.now_ns()==5'000'000);
    auto wait=std::async(std::launch::async,[&]{return imu.wait_for_data();});session->cancel();
    CHECK(wait.wait_for(100ms)==std::future_status::ready && !wait.get());session->stop();
    {
        auto next=std::make_shared<adapters::SimulationSession>(fixture.directory+"/model");CHECK(next->wait_ready(1s));
        CHECK(server.submit(Imu{3,{0,0,9.80665},{}},10'000'000));
        CHECK(server.submit(Imu{3,{0,0,9.80665},{}},9'000'000));
        for(int i=0;i<100 && !next->failed();++i)std::this_thread::sleep_for(2ms);
        CHECK(next->failed()); CHECK(next->error().find("time")!=std::string::npos);next->stop();
    }
    {
        ServerSnapshot fresh{100,0,Session{},info};server.set_snapshot(fresh);
        auto next=std::make_shared<adapters::SimulationSession>(fixture.directory+"/model");CHECK(next->wait_ready(1s));
        for(int i=0;i<300&&!next->failed();++i){server.submit(Imu{3,{0,0,9.80665},{}},i*5'000'000);std::this_thread::sleep_for(1ms);}
        for(int i=0;i<100&&!next->failed();++i)std::this_thread::sleep_for(2ms);
        CHECK(next->failed());CHECK(next->error().find("overflow")!=std::string::npos);next->stop();
    }
    {
        ServerSnapshot fresh{101,0,Session{0,Phase::ready,3},info};server.set_snapshot(fresh);
        auto next=std::make_shared<adapters::SimulationSession>(fixture.directory+"/model");CHECK(next->wait_ready(1s));
        CHECK(server.submit(Clock{},250'000'001));
        for(int i=0;i<100&&!next->failed();++i)std::this_thread::sleep_for(2ms);
        CHECK(next->failed());CHECK(next->error().find("image stream")!=std::string::npos);next->stop();
    }
    return 0;
}
