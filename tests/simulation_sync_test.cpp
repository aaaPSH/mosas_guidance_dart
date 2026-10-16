#include <mosas/adapters/simulation_sources.hpp>
#include <mosas/runtime/runtime_supervisor.hpp>
#include <mosas/runtime/legacy_flight_state_adapter.hpp>
#include "fakes/fake_algorithm_ports.hpp"
#include "fakes/fake_clock.hpp"
#include "fakes/fake_command_sink.hpp"
#include "test_support.hpp"
#include <filesystem>
#include <future>
#include <unistd.h>
using namespace std::chrono_literals;
using namespace dart_sim_transport;
using namespace mosas;
struct Fixture {std::string root;Fixture(){char p[]="/tmp/mosas-sync-XXXXXX";root=mkdtemp(p);}~Fixture(){std::filesystem::remove_all(root);}};
class BlockingEstimator final:public ports::IFlightStateEstimator{
public:
    domain::FlightStateSnapshot update(const domain::ImuKinematicSample& sample)override{
        std::unique_lock<std::mutex> lock(mutex);entered=true;cv.notify_all();cv.wait(lock,[&]{return released;});
        domain::FlightStateSnapshot result;result.time=sample.time;result.attitude.valid=true;return result;
    }
    void reset()noexcept override{}
    void release(){std::lock_guard<std::mutex> lock(mutex);released=true;cv.notify_all();}
    std::mutex mutex;std::condition_variable cv;bool entered=false,released=false;
};
class Recorder final:public runtime::IVideoRecorder{
public:
    bool start()override{return true;}void reset()noexcept override{}
    bool record(const runtime::ProcessedFrame& f)override{std::lock_guard<std::mutex> lock(mutex);frame=f;cv.notify_all();return true;}
    void cancel()noexcept override{}void stop()noexcept override{}std::string last_error()const override{return {};}
    std::mutex mutex;std::condition_variable cv;std::optional<runtime::ProcessedFrame> frame;
};
int main(){
    Fixture fixture;CameraInfo c;c.width=2;c.height=1;c.stride=6;
    Endpoint server(fixture.root+"/model",{23,0,Session{},c});
    auto session=std::make_shared<adapters::SimulationSession>(fixture.root+"/model");CHECK(session->wait_ready(1s));
    adapters::SimulationCameraSource camera(session);CHECK(camera.configure({}).status==runtime::SourceStatus::ok);
    std::atomic<std::int64_t> processed{-1};
    session->bind_watermark([&]{runtime::ImuStateSnapshot state;state.time=domain::make_simulation_time(processed.load());state.attitude.valid=true;return std::optional<runtime::ImuStateSnapshot>{state};});
    CHECK(server.submit(Image{2,1,6,PixelFormat::rgb8,Bytes(6,1)},17'000'000));
    CHECK(server.submit(Imu{3,{0,0,9.80665},{}},20'000'000)); // arrival alone is not a watermark
    runtime::CameraFrame frame;CHECK(camera.capture(&frame).status==runtime::SourceStatus::timeout);
    std::this_thread::sleep_for(120ms);CHECK(camera.capture(&frame).status==runtime::SourceStatus::timeout);CHECK(camera.sync_timeouts()==0); // paused
    processed=20'000'000;CHECK(camera.capture(&frame).status==runtime::SourceStatus::ok);CHECK(frame.timestamp_ns==17'000'000);
    CHECK(server.submit(Image{2,1,6,PixelFormat::rgb8,Bytes(6,2)},22'000'000));
    CHECK(server.submit(Clock{Phase::ready,0,-1},33'000'001));
    CHECK(camera.capture(&frame).status==runtime::SourceStatus::timeout);CHECK(camera.sync_timeouts()==1);
    auto waiting=std::async(std::launch::async,[&]{return camera.capture(&frame).status;});session->cancel();
    CHECK(waiting.wait_for(100ms)==std::future_status::ready);CHECK(waiting.get()==runtime::SourceStatus::cancelled);session->stop();session->clear_watermark();CHECK(!session->processed_state());
    {
        auto next=std::make_shared<adapters::SimulationSession>(fixture.root+"/model");CHECK(next->wait_ready(1s));
        auto estimator=std::make_unique<BlockingEstimator>();auto* blocker=estimator.get();
        auto video=std::make_unique<Recorder>();auto* recording=video.get();runtime::RuntimeOutputPorts outputs;outputs.video=std::move(video);
        auto commands=std::make_unique<test::FakeCommandSink>();auto* command_control=commands.get();
        runtime::RuntimeSupervisorDependencies deps;deps.flight_state_estimator=std::move(estimator);
        deps.vision_detector=std::make_unique<test::FakeVisionDetector>();deps.los_estimator=std::make_unique<test::FakeLosEstimator>();
        deps.los_rate_filter=std::make_unique<test::FakeLosRateFilter>();deps.guidance_law=std::make_unique<test::FakeGuidanceLaw>();deps.monotonic_clock=std::make_unique<adapters::SimulationClock>(next);
        runtime::RuntimeSupervisor supervisor(std::make_unique<adapters::SimulationImuSource>(next),std::make_unique<adapters::SimulationCameraSource>(next),std::move(commands),runtime::RuntimeSupervisorConfig{},std::move(deps),std::move(outputs));
        next->bind_watermark([&]{return supervisor.latest_state();});CHECK(supervisor.start());
        CHECK(server.submit(Image{2,1,6,PixelFormat::rgb8,Bytes(6,3)},17'000'000));CHECK(server.submit(Imu{3,{0,0,9.80665},{}},20'000'000));
        {std::unique_lock<std::mutex> lock(blocker->mutex);CHECK(blocker->cv.wait_for(lock,1s,[&]{return blocker->entered;}));}
        CHECK(!supervisor.latest_state());
        {std::lock_guard<std::mutex> lock(recording->mutex);CHECK(!recording->frame);}
        blocker->release();
        {std::unique_lock<std::mutex> lock(recording->mutex);CHECK(recording->cv.wait_for(lock,1s,[&]{return recording->frame.has_value();}));}
        supervisor.stop();next->clear_watermark();next->stop();
        CHECK(!recording->frame->guidance.valid); // watermark cannot invent the missing lower IMU bracket
        CHECK(recording->frame->guidance.invalid_reason==domain::GuidanceInvalidReason::stale_input);
        for(auto& command:command_control->commands())CHECK(!command.valid);
    }
    return 0;
}
