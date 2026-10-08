#include <mosas/runtime/runtime_supervisor.hpp>

#include "fakes/fake_algorithm_ports.hpp"
#include "fakes/fake_clock.hpp"
#include "fakes/fake_imu_source.hpp"
#include "test_support.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <utility>
#include <vector>

namespace {

using namespace std::chrono_literals;

class StderrCapture {
public:
    StderrCapture() : previous_(std::cerr.rdbuf(output_.rdbuf())) {}
    ~StderrCapture() { std::cerr.rdbuf(previous_); }
    bool empty() const { return output_.str().empty(); }

private:
    std::ostringstream output_;
    std::streambuf* previous_;
};

class SteppedCameraSource final : public mosas::runtime::CameraSource {
public:
    mosas::runtime::SourceResult configure(
        const mosas::runtime::CameraCaptureConfig&) override {
        std::lock_guard<std::mutex> lock(mutex_);
        cancelled_ = false;
        return {mosas::runtime::SourceStatus::ok, {}};
    }

    mosas::runtime::SourceResult capture(
        mosas::runtime::CameraFrame* frame) override {
        if (frame == nullptr) {
            return {mosas::runtime::SourceStatus::fatal,
                    "camera frame output is null"};
        }
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait(lock,
                        [this] { return cancelled_ || !frames_.empty(); });
        if (cancelled_) {
            return {mosas::runtime::SourceStatus::cancelled, {}};
        }
        *frame = std::move(frames_.front());
        frames_.erase(frames_.begin());
        return {mosas::runtime::SourceStatus::ok, {}};
    }

    void cancel() noexcept override {
        std::lock_guard<std::mutex> lock(mutex_);
        cancelled_ = true;
        condition_.notify_all();
    }

    void push(mosas::runtime::CameraFrame frame) {
        std::lock_guard<std::mutex> lock(mutex_);
        frames_.push_back(std::move(frame));
        condition_.notify_one();
    }

    bool wait_until_cancelled() {
        std::unique_lock<std::mutex> lock(mutex_);
        return condition_.wait_for(lock, 1s, [this] { return cancelled_; });
    }

private:
    mutable std::mutex mutex_;
    std::condition_variable condition_;
    std::vector<mosas::runtime::CameraFrame> frames_;
    bool cancelled_ = false;
};

class ProbeCommandSink final : public mosas::runtime::CommandSink {
public:
    bool start() override {
        std::lock_guard<std::mutex> lock(mutex_);
        commands_.clear();
        active_generation_ = 0;
        return true;
    }

    bool try_submit(
        const mosas::runtime::GuidanceCommand& command) noexcept override {
        std::lock_guard<std::mutex> lock(mutex_);
        if (command.generation != active_generation_) {
            return false;
        }
        commands_.push_back(command);
        condition_.notify_all();
        return true;
    }

    void set_active_generation(std::uint64_t generation) noexcept override {
        std::lock_guard<std::mutex> lock(mutex_);
        active_generation_ = generation;
    }

    void cancel() noexcept override {}
    void stop() noexcept override {}

    bool wait_for_count(std::size_t count) {
        std::unique_lock<std::mutex> lock(mutex_);
        return condition_.wait_for(lock, 1s,
                                   [this, count] {
                                       return commands_.size() >= count;
                                   });
    }

    std::size_t count() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return commands_.size();
    }

private:
    mutable std::mutex mutex_;
    std::condition_variable condition_;
    std::vector<mosas::runtime::GuidanceCommand> commands_;
    std::uint64_t active_generation_ = 0;
};

class BlockingVideoRecorder final : public mosas::runtime::IVideoRecorder {
public:
    bool start() override { return true; }
    void reset() noexcept override {}

    bool record(const mosas::runtime::ProcessedFrame&) override {
        std::unique_lock<std::mutex> lock(mutex_);
        entered_ = true;
        condition_.notify_all();
        condition_.wait(lock, [this] { return released_ || cancelled_; });
        return true;
    }

    void cancel() noexcept override {
        std::lock_guard<std::mutex> lock(mutex_);
        cancelled_ = true;
        condition_.notify_all();
    }

    void stop() noexcept override {}
    std::string last_error() const override { return {}; }

    bool wait_until_entered() {
        std::unique_lock<std::mutex> lock(mutex_);
        return condition_.wait_for(lock, 1s, [this] { return entered_; });
    }

    bool still_blocked() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return entered_ && !released_ && !cancelled_;
    }

    void release() {
        std::lock_guard<std::mutex> lock(mutex_);
        released_ = true;
        condition_.notify_all();
    }

private:
    mutable std::mutex mutex_;
    std::condition_variable condition_;
    bool entered_ = false;
    bool released_ = false;
    bool cancelled_ = false;
};

class FailingAsyncLogger final : public mosas::runtime::IAsyncLogger {
public:
    bool start() override { return true; }

    bool try_submit(
        const mosas::runtime::RuntimeLogEvent&) noexcept override {
        submitted_count_.fetch_add(1, std::memory_order_relaxed);
        return false;
    }

    mosas::runtime::SinkDiagnostics diagnostics() const noexcept override {
        mosas::runtime::SinkDiagnostics result;
        result.enabled = true;
        result.healthy = false;
        result.failure_count = 1;
        result.last_error = "injected logger failure";
        return result;
    }

    void stop_and_flush() noexcept override {}
    std::size_t submitted_count() const noexcept {
        return submitted_count_.load(std::memory_order_relaxed);
    }

private:
    std::atomic<std::size_t> submitted_count_{0};
};

class RejectingImuRecorder final : public mosas::runtime::IImuRecorder {
public:
    bool start() override { return true; }

    bool try_record(
        const mosas::domain::ImuKinematicSample&) noexcept override {
        dropped_count_.fetch_add(1, std::memory_order_relaxed);
        return false;
    }

    mosas::runtime::SinkDiagnostics diagnostics() const noexcept override {
        mosas::runtime::SinkDiagnostics result;
        result.enabled = true;
        result.healthy = false;
        result.dropped_count = dropped_count_.load(std::memory_order_relaxed);
        result.failure_count = 1;
        result.last_error = "injected IMU recording failure";
        return result;
    }

    void cancel() noexcept override { ++cancel_count_; }
    void stop() noexcept override { ++stop_count_; }
    std::size_t cancel_count() const noexcept { return cancel_count_.load(); }
    std::size_t stop_count() const noexcept { return stop_count_.load(); }

private:
    std::atomic<std::size_t> dropped_count_{0};
    std::atomic<std::size_t> cancel_count_{0};
    std::atomic<std::size_t> stop_count_{0};
};

class CountingTelemetryRecorder final
    : public mosas::runtime::ITelemetryRecorder {
public:
    bool start() override { return true; }
    void reset() noexcept override {}

    bool record(const mosas::runtime::ProcessedFrame&) override {
        std::lock_guard<std::mutex> lock(mutex_);
        ++recorded_count_;
        condition_.notify_all();
        return true;
    }

    void cancel() noexcept override {}
    void stop() noexcept override {}
    std::string last_error() const override { return {}; }

    bool wait_for_count(std::size_t count) {
        std::unique_lock<std::mutex> lock(mutex_);
        return condition_.wait_for(lock, 1s,
                                   [this, count] {
                                       return recorded_count_ >= count;
                                   });
    }

private:
    std::mutex mutex_;
    std::condition_variable condition_;
    std::size_t recorded_count_ = 0;
};

int test_imu_recording_failure_faults_runtime() {
    using namespace mosas;

    auto imu_source = std::make_unique<test::FakeImuSource>(
        std::vector<runtime::ImuAngularVelocitySample>{
            {1'010'000'000, {0.0, 0.0, 9.80665}, {0.0, 0.0, 0.0}, 0.0,
             0.0}});
    auto* imu_source_control = imu_source.get();
    auto camera_source = std::make_unique<SteppedCameraSource>();
    auto* camera_source_control = camera_source.get();
    auto command_sink = std::make_unique<ProbeCommandSink>();
    auto* command_sink_control = command_sink.get();
    auto recorder = std::make_unique<RejectingImuRecorder>();
    auto* recorder_control = recorder.get();
    runtime::RuntimeOutputPorts output_ports;
    output_ports.imu_recorder = std::move(recorder);

    auto vision = std::make_unique<test::FakeVisionDetector>();
    auto flight_state = std::make_unique<test::FakeFlightStateEstimator>();
    auto* flight_state_control = flight_state.get();
    auto los = std::make_unique<test::FakeLosEstimator>();
    auto rate_filter = std::make_unique<test::FakeLosRateFilter>();
    auto guidance = std::make_unique<test::FakeGuidanceLaw>();
    runtime::RuntimeSupervisorDependencies dependencies;
    dependencies.vision_detector = std::move(vision);
    dependencies.flight_state_estimator = std::move(flight_state);
    dependencies.los_estimator = std::move(los);
    dependencies.los_rate_filter = std::move(rate_filter);
    dependencies.guidance_law = std::move(guidance);
    dependencies.monotonic_clock = std::make_unique<test::FakeClock>(
        1'010'000'000);

    runtime::RuntimeSupervisorConfig config;
    StderrCapture stderr_capture;
    runtime::RuntimeSupervisor runtime(
        std::move(imu_source), std::move(camera_source),
        std::move(command_sink), config, std::move(dependencies),
        std::move(output_ports));
    CHECK(runtime.start());

    domain::FlightStateSnapshot flight_result{};
    flight_result.attitude.valid = true;
    flight_result.launch.phase = domain::LaunchPhase::launched;
    flight_result.launch.transition_time.monotonic_ns = 900'000'000;
    flight_state_control->set_result(flight_result);
    imu_source_control->release();

    const auto fault_deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(1);
    while (!runtime.faulted() &&
           std::chrono::steady_clock::now() < fault_deadline) {
        std::this_thread::yield();
    }
    CHECK(runtime.faulted());
    CHECK(runtime.last_error() == "failed to record IMU sample");
    CHECK(camera_source_control->wait_until_cancelled());
    CHECK(command_sink_control->count() == 1);
    runtime.stop();
    const auto diagnostics = runtime.output_diagnostics();
    CHECK(diagnostics.imu_recording.enabled);
    CHECK(!diagnostics.imu_recording.healthy);
    CHECK(diagnostics.imu_recording.dropped_count == 1);
    CHECK(diagnostics.imu_recording.last_error ==
          "injected IMU recording failure");
    CHECK(recorder_control->cancel_count() == 1);
    CHECK(recorder_control->stop_count() == 1);
    CHECK(!diagnostics.logging.enabled);
    CHECK(stderr_capture.empty());
    return 0;
}

mosas::runtime::CameraFrame make_frame(std::int64_t timestamp_ns) {
    auto pixels = std::make_shared<std::vector<std::uint8_t>>(12, 0x44);
    mosas::runtime::CameraFrame frame{};
    frame.timestamp_ns = timestamp_ns;
    frame.image.owner = pixels;
    frame.image.data = pixels->data();
    frame.image.data_size_bytes = pixels->size();
    frame.image.width = 2;
    frame.image.height = 2;
    frame.image.stride_bytes = 6;
    frame.image.format = mosas::domain::PixelFormat::bgr8;
    return frame;
}

}  // namespace

int main() {
    using namespace mosas;

    std::vector<runtime::ImuAngularVelocitySample> imu_samples;
    for (std::int64_t index = 0; index <= 10; ++index) {
        imu_samples.push_back(
            {1'000'000'000 + index * 1'000'000,
             {0.0, 0.0, 9.80665}, {0.01, 0.02, 0.03}, 0.0, 0.0});
    }
    auto imu_source = std::make_unique<test::FakeImuSource>(
        std::move(imu_samples));
    auto* imu_source_control = imu_source.get();
    auto camera_source = std::make_unique<SteppedCameraSource>();
    auto* camera_source_control = camera_source.get();
    auto command_sink = std::make_unique<ProbeCommandSink>();
    auto* command_sink_control = command_sink.get();

    auto video_recorder = std::make_unique<BlockingVideoRecorder>();
    auto* video_control = video_recorder.get();
    auto telemetry_recorder = std::make_unique<CountingTelemetryRecorder>();
    auto* telemetry_control = telemetry_recorder.get();
    runtime::RuntimeOutputPorts output_ports;
    output_ports.video = std::move(video_recorder);
    output_ports.telemetry = std::move(telemetry_recorder);
    auto logger = std::make_unique<FailingAsyncLogger>();
    auto* logger_control = logger.get();
    output_ports.logger = std::move(logger);

    auto flight_state = std::make_unique<test::FakeFlightStateEstimator>();
    auto* flight_state_control = flight_state.get();
    auto vision = std::make_unique<test::FakeVisionDetector>();
    auto* vision_control = vision.get();
    auto los = std::make_unique<test::FakeLosEstimator>();
    auto los_control = los.get();
    auto rate_filter = std::make_unique<test::FakeLosRateFilter>();
    auto* rate_filter_control = rate_filter.get();
    auto guidance = std::make_unique<test::FakeGuidanceLaw>();
    auto* guidance_control = guidance.get();

    runtime::RuntimeSupervisorDependencies dependencies;
    dependencies.vision_detector = std::move(vision);
    dependencies.flight_state_estimator = std::move(flight_state);
    dependencies.los_estimator = std::move(los);
    dependencies.los_rate_filter = std::move(rate_filter);
    dependencies.guidance_law = std::move(guidance);
    dependencies.monotonic_clock = std::make_unique<test::FakeClock>(
        1'000'000'000);

    runtime::RuntimeSupervisorConfig config;
    config.history_capacity = 32;
    config.capture_queue_capacity = 8;
    config.output_queue_capacity = 8;
    config.camera_intrinsics = {100.0, 100.0, 1.0, 1.0};

    runtime::RuntimeSupervisor runtime(
        std::move(imu_source), std::move(camera_source),
        std::move(command_sink), config, std::move(dependencies),
        std::move(output_ports));
    CHECK(runtime.start());

    domain::FlightStateSnapshot flight_result{};
    flight_result.attitude.valid = true;
    flight_result.attitude.navigation_from_dart = {};
    flight_result.attitude.corrected_angular_velocity = {0.0, 0.0, 0.0};
    flight_result.launch.phase = domain::LaunchPhase::launched;
    flight_result.launch.transition_time.monotonic_ns = 900'000'000;
    flight_result.launch.transition_time.source_domain =
        domain::ClockDomain::monotonic;
    flight_result.launch.reason =
        domain::LaunchReason::threshold_confirmed;
    flight_state_control->set_result(flight_result);

    domain::VisionDetection detection{};
    detection.observation.valid = true;
    detection.observation.reason = domain::ObservationReason::found;
    detection.observation.center = {10.0, 20.0};
    detection.diagnostics.bounding_box = {8, 18, 4, 4};
    detection.diagnostics.area_pixels = 12;
    for (std::size_t index = 0; index < 5; ++index) {
        vision_control->push(detection);
    }
    los_control->set_result({true, {0.1}, {-0.2}});
    rate_filter_control->set_result({true, {0.3}, {-0.4}});
    domain::GuidanceOutput guidance_result{};
    guidance_result.valid = true;
    guidance_result.invalid_reason = domain::GuidanceInvalidReason::none;
    guidance_result.command.valid = true;
    guidance_result.command.invalid_reason =
        domain::GuidanceInvalidReason::none;
    guidance_control->set_result(guidance_result);

    imu_source_control->release();
    const auto imu_deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(1);
    bool imu_ready = false;
    while (std::chrono::steady_clock::now() < imu_deadline) {
        const auto state = runtime.latest_state();
        if (state.has_value() &&
            state->time.monotonic_ns == 1'010'000'000) {
            imu_ready = true;
            break;
        }
        std::this_thread::yield();
    }
    CHECK(imu_ready);

    camera_source_control->push(make_frame(1'002'000'000));
    CHECK(command_sink_control->wait_for_count(1));
    CHECK(video_control->wait_until_entered());
    CHECK(video_control->still_blocked());

    for (std::size_t index = 1; index < 5; ++index) {
        const std::int64_t frame_timestamp =
            1'002'000'000 +
            static_cast<std::int64_t>(index == 1 ? 0 : index - 1) *
                1'000'000;
        camera_source_control->push(make_frame(frame_timestamp));
        CHECK(command_sink_control->wait_for_count(index + 1));
    }
    CHECK(telemetry_control->wait_for_count(5));
    CHECK(video_control->still_blocked());
    CHECK(!runtime.faulted());
    CHECK(command_sink_control->count() >= 5);

    video_control->release();
    runtime.stop();
    const auto diagnostics = runtime.output_diagnostics();
    CHECK(diagnostics.video.enabled);
    CHECK(diagnostics.video.healthy);
    CHECK(diagnostics.telemetry.enabled);
    CHECK(diagnostics.telemetry.healthy);
    CHECK(!diagnostics.visualization.enabled);
    CHECK(diagnostics.visualization.healthy);
    CHECK(!diagnostics.imu_recording.enabled);
    CHECK(diagnostics.imu_recording.healthy);
    CHECK(diagnostics.logging.enabled);
    CHECK(!diagnostics.logging.healthy);
    CHECK(diagnostics.logging.failure_count == 1);
    CHECK(diagnostics.logging.last_error == "injected logger failure");
    CHECK(logger_control->submitted_count() > 0);
    CHECK(!runtime.faulted());
    CHECK(test_imu_recording_failure_faults_runtime() == 0);
    return 0;
}
