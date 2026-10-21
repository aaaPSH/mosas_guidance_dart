#include <mosas/domain/time.hpp>
#include <mosas/runtime/runtime_types.hpp>
#include <mosas/runtime/runtime_supervisor.hpp>
#include <mosas/runtime/imu_angular_velocity.hpp>
#include "runtime_value_helpers.hpp"
#include "imu_pipeline.hpp"
#include "launch_generation_coordinator.hpp"
#include "fakes/fake_algorithm_ports.hpp"
#include "fakes/fake_imu_source.hpp"
#include "fakes/fake_camera_source.hpp"
#include "fakes/fake_command_sink.hpp"
#include "fakes/fake_clock.hpp"
#include <mosas/runtime/legacy_flight_state_adapter.hpp>
#include "test_support.hpp"
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>

namespace {
using namespace mosas;
using namespace std::chrono_literals;
class FrameRecorder final : public runtime::IVideoRecorder {
public:
    bool start() override { return true; }
    void reset() noexcept override {}
    bool record(const runtime::ProcessedFrame& frame) override {
        std::lock_guard<std::mutex> lock(mutex); frames.push_back(frame); cv.notify_all(); return true;
    }
    void cancel() noexcept override {}
    void stop() noexcept override {}
    std::string last_error() const override { return {}; }
    std::mutex mutex; std::condition_variable cv; std::vector<runtime::ProcessedFrame> frames;
};
int verify_frame_metadata() {
    auto make_frame=[](std::int64_t ns) {
        auto pixels=std::make_shared<std::vector<std::uint8_t>>(12,0);
        runtime::CameraFrame frame; frame.timestamp_ns=ns; frame.received_timestamp_ns=20'000'000;
        frame.timestamp_source=runtime::CameraTimestampSource::simulation;
        frame.sensor_time=domain::make_simulation_time(ns);
        frame.image={pixels,pixels->data(),pixels->size(),2,2,6,domain::PixelFormat::bgr8}; return frame;
    };
    std::vector<runtime::ImuAngularVelocitySample> samples{{0,{0,9.80665,0},{0,0,0},0,0},{20'000'000,{0,9.80665,0},{0,0,0},0,0}};
    for(auto& sample:samples) sample.sensor_time=domain::make_simulation_time(sample.timestamp_ns);
    auto imu=std::make_unique<test::FakeImuSource>(samples); auto* imu_control=imu.get();
    auto camera=std::make_unique<test::FakeCameraSource>(std::vector<runtime::CameraFrame>{make_frame(5'000'000),make_frame(15'000'000)}); auto* camera_control=camera.get();
    auto sink=std::make_unique<test::FakeCommandSink>(); auto* sink_control=sink.get();
    auto recorder=std::make_unique<FrameRecorder>(); auto* recording=recorder.get();
    runtime::RuntimeOutputPorts ports; ports.video=std::move(recorder);
    auto vision=std::make_unique<test::FakeVisionDetector>(); auto* vision_control=vision.get();
    auto estimator=std::make_unique<test::FakeFlightStateEstimator>(); auto* estimator_control=estimator.get();
    auto los=std::make_unique<test::FakeLosEstimator>(); domain::LineOfSight line; line.valid=true; los->set_result(line);
    auto rate=std::make_unique<test::FakeLosRateFilter>(); domain::LosRateEstimate r; r.valid=true; r.angular_velocity.valid=true; rate->set_result(r);
    auto guidance=std::make_unique<test::FakeGuidanceLaw>(); domain::GuidanceOutput g; g.valid=true; g.command.overload_g={0,2,3}; guidance->set_result(g);
    runtime::RuntimeSupervisorDependencies dependencies;
    dependencies.vision_detector=std::move(vision); dependencies.flight_state_estimator=std::move(estimator);
    dependencies.los_estimator=std::move(los); dependencies.los_rate_filter=std::move(rate); dependencies.guidance_law=std::move(guidance);
    dependencies.monotonic_clock=std::make_unique<test::FakeClock>(20'000'000);
    runtime::RuntimeSupervisorConfig config; config.capture_queue_capacity=2; config.output_queue_capacity=4;
    runtime::RuntimeSupervisor supervisor(std::move(imu),std::move(camera),std::move(sink),config,std::move(dependencies),std::move(ports));
    CHECK(supervisor.start());
    domain::FlightStateSnapshot state; state.attitude.valid=true; state.launch.phase=domain::LaunchPhase::launched; state.launch.transition_time=domain::make_simulation_time(0); estimator_control->set_result(state);
    domain::VisionDetection detection; detection.observation.valid=true; vision_control->push(detection); vision_control->push(detection);
    imu_control->release();
    auto deadline=std::chrono::steady_clock::now()+1s;
    while(std::chrono::steady_clock::now()<deadline) {auto s=supervisor.latest_state();if(s && s->time.monotonic_ns==20'000'000) break;std::this_thread::yield();}
    CHECK(supervisor.latest_state().has_value() && supervisor.latest_state()->time.monotonic_ns==20'000'000);
    camera_control->release();
    {std::unique_lock<std::mutex> lock(recording->mutex);CHECK(recording->cv.wait_for(lock,1s,[&]{return recording->frames.size()==2;}));}
    supervisor.stop(); CHECK(!supervisor.faulted());
    for(const auto& frame:recording->frames) {
        CHECK(frame.vision_detection.observation.time.source==domain::TimestampSource::simulation);
        CHECK(frame.flight_state.time.source==domain::TimestampSource::simulation);
        CHECK(frame.flight_state.attitude.time.source==domain::TimestampSource::simulation);
        CHECK(frame.flight_state.launch.transition_time.source==domain::TimestampSource::simulation);
        CHECK(frame.guidance.command.source_time.source==domain::TimestampSource::simulation);
    }
    bool valid_seen=false,neutral_seen=false;
    for(const auto& command:sink_control->commands()) {
        if(command.source_time.monotonic_ns==5'000'000) {neutral_seen=true;CHECK(!command.valid);CHECK(command.source_time.source==domain::TimestampSource::simulation);}
        if(command.valid) {valid_seen=true;CHECK(command.source_time.source==domain::TimestampSource::simulation);CHECK(command.source_time.monotonic_ns==15'000'000);}
    }
    CHECK(valid_seen && neutral_seen);
    return 0;
}
} // namespace

int main() {
    using namespace mosas; using namespace std::chrono_literals;
    auto time=domain::make_simulation_time(123);
    CHECK(time.source_domain==domain::ClockDomain::simulation);
    CHECK(time.source==domain::TimestampSource::simulation);
    CHECK(time.quality==domain::TimestampQuality::simulation);
    auto resolved=runtime::detail::resolve_sensor_time(123,time);
    CHECK(resolved.monotonic_ns==123 && resolved.source_time_ns==123);
    const auto neutral = runtime::detail::make_neutral_command(resolved, 7, true, true);
    CHECK(neutral.source_time.source == domain::TimestampSource::simulation);
    CHECK(neutral.launched && neutral.target_found && !neutral.valid);
    CHECK(runtime::detail::resolve_sensor_time(1,std::nullopt).source_domain==domain::ClockDomain::monotonic);
    auto rejects=[&](domain::SensorTime t){try{runtime::detail::resolve_sensor_time(123,t);}catch(const std::invalid_argument&){return true;}return false;};
    auto bad=time; bad.monotonic_ns=124; CHECK(rejects(bad));
    bad=time; bad.quality=domain::TimestampQuality::hardware_synchronized; CHECK(rejects(bad));
    bad=time; bad.source=domain::TimestampSource::imu_uart_receive_estimate; CHECK(rejects(bad));
    bad=time; bad.source_domain=domain::ClockDomain::monotonic; CHECK(rejects(bad));
    bad=time; bad.source_time_ns=122; CHECK(rejects(bad));
    std::atomic<bool> stopped{false}; std::mutex mutex; std::condition_variable cv;
    bool recorded=false; domain::SensorTime recorded_time;
    runtime::ImuAngularVelocitySample sample{123,{0,9.80665,0},{0,0,0},0,0}; sample.sensor_time=time;
    auto source=std::make_unique<test::FakeImuSource>(std::vector<runtime::ImuAngularVelocitySample>{sample}); auto* control=source.get();
    class Sink final: public runtime::CommandSink {
    public: bool start() override{return true;} bool try_submit(const runtime::GuidanceCommand&) noexcept override{return true;}
    } sink;
    runtime::LaunchGenerationCoordinator generation; generation.reset(sink);
    runtime::ImuPipelineCallbacks callbacks;
    callbacks.stop_requested=[&]{return stopped.load();};
    callbacks.try_record_sample=[&](const domain::ImuKinematicSample& s){std::lock_guard<std::mutex> lock(mutex); recorded_time=s.time;recorded=true;cv.notify_all();return true;};
    runtime::ImuPipeline pipeline(std::move(source),std::make_unique<test::FakeFlightStateEstimator>(),8,generation,callbacks);
    CHECK(pipeline.start_source().status==runtime::SourceStatus::ok); pipeline.reset(); pipeline.start_worker(); control->release();
    {std::unique_lock<std::mutex> lock(mutex);CHECK(cv.wait_for(lock,1s,[&]{return recorded;}));}
    auto state=pipeline.latest_state();CHECK(state.has_value());
    CHECK(state->time.source==domain::TimestampSource::simulation);
    CHECK(state->imu_sample.time.source==domain::TimestampSource::simulation);
    CHECK(state->attitude.time.source==domain::TimestampSource::simulation);
    CHECK(recorded_time.source==domain::TimestampSource::simulation);
    stopped=true;pipeline.cancel_source();pipeline.join_worker();pipeline.stop_source();
    runtime::LegacyFlightStateEstimatorAdapter legacy;
    domain::FlightStateSnapshot flight;
    for(int i=0;i<50;++i) {
        domain::ImuKinematicSample input; input.time=domain::make_simulation_time(i*5'000'000); input.acceleration={i<20?0.0:250.0,9.80665,0};
        flight=legacy.update(input);
    }
    CHECK(flight.launch.phase==domain::LaunchPhase::launched);
    CHECK(flight.launch.transition_time.source==domain::TimestampSource::simulation);
    CHECK(flight.launch.transition_time.source_time_ns==flight.launch.transition_time.monotonic_ns);
    CHECK(verify_frame_metadata()==0);
    return 0;
}
