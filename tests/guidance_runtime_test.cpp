#include <mosas/runtime/flight_phase_detector.hpp>
#include <mosas/runtime/guidance_runtime.hpp>
#include <mosas/runtime/imu_state_history.hpp>
#include <mosas/runtime/runtime_interfaces.hpp>
#include <mosas/runtime/runtime_types.hpp>

#include <cassert>
#include <cmath>
#include <condition_variable>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

namespace {

class EmptyImuSource final : public mosas::runtime::ImuSource {
public:
    bool read(mosas::runtime::ImuSample* sample) override {
        (void)sample;
        return false;
    }

    void cancel() noexcept override {}
};

class EmptyCameraSource final : public mosas::runtime::CameraSource {
public:
    bool configure(const mosas::runtime::CameraCaptureConfig&) override {
        return true;
    }

    bool capture(mosas::runtime::CameraFrame* frame) override {
        (void)frame;
        return false;
    }

    void cancel() noexcept override {}
};

class EmptyCommandSink final : public mosas::runtime::CommandSink {
public:
    bool send(const mosas::runtime::GuidanceCommand&) override {
        return true;
    }
};

class ScriptedImuSource final : public mosas::runtime::ImuSource {
public:
    explicit ScriptedImuSource(std::vector<mosas::runtime::ImuSample> samples)
        : samples_(std::move(samples)) {}

    bool read(mosas::runtime::ImuSample* sample) override {
        std::lock_guard<std::mutex> lock(mutex_);
        if (sample == nullptr || next_sample_ >= samples_.size()) {
            return false;
        }
        *sample = samples_[next_sample_++];
        return true;
    }

    void cancel() noexcept override {
        std::lock_guard<std::mutex> lock(mutex_);
        cancelled_ = true;
        condition_.notify_all();
    }

    bool cancelled() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return cancelled_;
    }

private:
    std::vector<mosas::runtime::ImuSample> samples_;
    std::size_t next_sample_ = 0;
    mutable std::mutex mutex_;
    std::condition_variable condition_;
    bool cancelled_ = false;
};

class BlockingImuSource final : public mosas::runtime::ImuSource {
public:
    bool read(mosas::runtime::ImuSample*) override {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait(lock, [this] { return cancelled_; });
        return false;
    }

    void cancel() noexcept override {
        std::lock_guard<std::mutex> lock(mutex_);
        cancelled_ = true;
        condition_.notify_all();
    }

    bool cancelled() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return cancelled_;
    }

private:
    mutable std::mutex mutex_;
    std::condition_variable condition_;
    bool cancelled_ = false;
};

class BlockingCameraSource final : public mosas::runtime::CameraSource {
public:
    bool configure(const mosas::runtime::CameraCaptureConfig&) override {
        return true;
    }

    bool capture(mosas::runtime::CameraFrame*) override {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait(lock, [this] { return cancelled_; });
        return false;
    }

    void cancel() noexcept override {
        std::lock_guard<std::mutex> lock(mutex_);
        cancelled_ = true;
        condition_.notify_all();
    }

    bool cancelled() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return cancelled_;
    }

private:
    mutable std::mutex mutex_;
    std::condition_variable condition_;
    bool cancelled_ = false;
};

class ScriptedCameraSource final : public mosas::runtime::CameraSource {
public:
    explicit ScriptedCameraSource(std::vector<mosas::runtime::CameraFrame> frames)
        : frames_(std::move(frames)) {}

    bool configure(const mosas::runtime::CameraCaptureConfig& config) override {
        std::lock_guard<std::mutex> lock(mutex_);
        config_ = config;
        configured_ = true;
        return true;
    }

    bool capture(mosas::runtime::CameraFrame* frame) override {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait(lock, [this] {
            return released_ || cancelled_;
        });
        if (cancelled_ || next_frame_ >= frames_.size()) {
            return false;
        }
        *frame = frames_[next_frame_++];
        return true;
    }

    void release() {
        std::lock_guard<std::mutex> lock(mutex_);
        released_ = true;
        condition_.notify_all();
    }

    void cancel() noexcept override {
        std::lock_guard<std::mutex> lock(mutex_);
        cancelled_ = true;
        condition_.notify_all();
    }

    bool configured() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return configured_;
    }

    mosas::runtime::CameraCaptureMode mode() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return config_.mode;
    }

private:
    std::vector<mosas::runtime::CameraFrame> frames_;
    std::size_t next_frame_ = 0;
    mutable std::mutex mutex_;
    std::condition_variable condition_;
    mosas::runtime::CameraCaptureConfig config_{};
    bool configured_ = false;
    bool released_ = false;
    bool cancelled_ = false;
};

class RecordingCommandSink final : public mosas::runtime::CommandSink {
public:
    bool send(const mosas::runtime::GuidanceCommand& command) override {
        std::lock_guard<std::mutex> lock(mutex_);
        commands_.push_back(command);
        return true;
    }

    std::size_t size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return commands_.size();
    }

    mosas::runtime::GuidanceCommand command_at(std::size_t index) const {
        std::lock_guard<std::mutex> lock(mutex_);
        return commands_.at(index);
    }

private:
    mutable std::mutex mutex_;
    std::vector<mosas::runtime::GuidanceCommand> commands_;
};

class FailingCommandSink final : public mosas::runtime::CommandSink {
public:
    bool send(const mosas::runtime::GuidanceCommand&) override {
        std::lock_guard<std::mutex> lock(mutex_);
        ++send_count_;
        return false;
    }

    std::size_t send_count() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return send_count_;
    }

private:
    mutable std::mutex mutex_;
    std::size_t send_count_ = 0;
};

class ThrowingImuSource final : public mosas::runtime::ImuSource {
public:
    bool read(mosas::runtime::ImuSample*) override {
        throw std::runtime_error("imu read failed");
    }

    void cancel() noexcept override {}
};

bool wait_until(const std::function<bool()>& predicate,
                std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (predicate()) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return predicate();
}

cv::Mat make_guidance_frame() {
    cv::Mat frame(48, 64, CV_8UC3, cv::Scalar(0, 0, 0));
    frame(cv::Rect(28, 20, 8, 8)).setTo(cv::Scalar(0, 255, 0));
    return frame;
}

mosas::runtime::FlightPhaseDetectorConfig free_flight_detector_config() {
    mosas::runtime::FlightPhaseDetectorConfig config;
    config.min_static_samples = 2;
    config.min_static_duration_ns = 1000000;
    config.max_static_axis_variation = 0.2;
    config.ejection_start_threshold = 20.0;
    config.ejection_confirm_duration_ns = 2000000;
    config.free_flight_release_threshold = 3.0;
    config.free_flight_confirm_duration_ns = 2000000;
    return config;
}

std::vector<mosas::runtime::ImuSample> free_flight_samples() {
    return {
        {0, {0.0, 9.8, 0.0}, {0.0, 0.0, 0.0}},
        {1000000, {0.0, 9.8, 0.0}, {0.0, 0.0, 0.0}},
        {2000000, {21.0, 9.8, 0.0}, {0.0, 0.0, 0.0}},
        {4000000, {21.0, 9.8, 0.0}, {0.0, 0.0, 0.0}},
        {5000000, {1.0, 9.8, 0.0}, {0.0, 0.0, 0.0}},
        {7000000, {1.0, 9.8, 0.0}, {0.0, 0.0, 0.0}},
    };
}

void test_runtime_types_are_constructible() {
    const mosas::runtime::ImuSample imu{
        100, {0.0, 9.80665, 0.0}, {0.0, 0.0, 0.0}};
    const mosas::runtime::CameraFrame frame{200, cv::Mat()};
    assert(imu.timestamp_ns < frame.timestamp_ns);
    assert(mosas::runtime::FlightPhase::pre_launch !=
           mosas::runtime::FlightPhase::free_flight);
}

void test_non_horizontal_static_window_initializes_from_three_axis_baseline() {
    mosas::runtime::FlightPhaseDetectorConfig config;
    config.min_static_samples = 3;
    config.min_static_duration_ns = 2;
    config.max_static_axis_variation = 0.2;
    mosas::runtime::FlightPhaseDetector detector(config);

    assert(detector.update({0, {3.0, 8.0, -4.0}, {0.0, 0.0, 0.0}}) ==
           mosas::runtime::FlightPhase::pre_launch);
    assert(detector.update({1, {3.0, 8.0, -4.0}, {0.0, 0.0, 0.0}}) ==
           mosas::runtime::FlightPhase::pre_launch);
    assert(detector.update({2, {3.0, 8.0, -4.0}, {0.0, 0.0, 0.0}}) ==
           mosas::runtime::FlightPhase::pre_launch);
    const Vector3 baseline = detector.baseline_acceleration();
    assert(std::abs(baseline.x - 3.0) < 1e-12);
    assert(std::abs(baseline.y - 8.0) < 1e-12);
    assert(std::abs(baseline.z + 4.0) < 1e-12);
    assert(detector.initialized());
}

void test_short_elastic_impulse_transitions_to_ejection_then_free_flight() {
    mosas::runtime::FlightPhaseDetectorConfig config;
    config.min_static_samples = 2;
    config.min_static_duration_ns = 1;
    config.max_static_axis_variation = 0.2;
    config.launch_direction = 1;
    config.ejection_start_threshold = 20.0;
    config.ejection_confirm_duration_ns = 2;
    config.free_flight_release_threshold = 3.0;
    config.free_flight_confirm_duration_ns = 2;
    mosas::runtime::FlightPhaseDetector detector(config);

    detector.update({0, {0.0, 9.8, 0.0}, {0.0, 0.0, 0.0}});
    detector.update({1, {0.0, 9.8, 0.0}, {0.0, 0.0, 0.0}});
    assert(detector.update({2, {21.0, 9.8, 0.0}, {0.0, 0.0, 0.0}}) ==
           mosas::runtime::FlightPhase::pre_launch);
    assert(detector.update({4, {21.0, 9.8, 0.0}, {0.0, 0.0, 0.0}}) ==
           mosas::runtime::FlightPhase::ejection);
    assert(detector.update({5, {1.0, 9.8, 0.0}, {0.0, 0.0, 0.0}}) ==
           mosas::runtime::FlightPhase::ejection);
    assert(detector.update({7, {1.0, 9.8, 0.0}, {0.0, 0.0, 0.0}}) ==
           mosas::runtime::FlightPhase::free_flight);
    assert(detector.update({8, {0.0, 9.8, 0.0}, {0.0, 0.0, 0.0}}) ==
           mosas::runtime::FlightPhase::free_flight);
}

void test_light_acceleration_does_not_trigger_launch() {
    mosas::runtime::FlightPhaseDetectorConfig config;
    config.min_static_samples = 2;
    config.min_static_duration_ns = 1;
    config.max_static_axis_variation = 0.2;
    config.ejection_start_threshold = 20.0;
    config.ejection_confirm_duration_ns = 3;
    mosas::runtime::FlightPhaseDetector detector(config);

    detector.update({0, {0.0, 9.8, 0.0}, {0.0, 0.0, 0.0}});
    detector.update({1, {0.0, 9.8, 0.0}, {0.0, 0.0, 0.0}});
    detector.update({2, {10.0, 9.8, 0.0}, {0.0, 0.0, 0.0}});
    detector.update({3, {0.0, 9.8, 0.0}, {0.0, 0.0, 0.0}});
    assert(detector.phase() == mosas::runtime::FlightPhase::pre_launch);
}

void test_history_returns_latest_snapshot_not_after_frame_timestamp() {
    mosas::runtime::ImuStateHistory history(2);
    history.push({10, mosas::runtime::FlightPhase::pre_launch,
                  {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0},
                  {0.0, 9.8, 0.0}, {0.0, 0.0, 0.0}});
    history.push({20, mosas::runtime::FlightPhase::ejection,
                  {0.1, 0.0, 0.0}, {2.0, 0.0, 0.0},
                  {20.0, 9.8, 0.0}, {0.0, 0.0, 0.0}});

    const auto result = history.find_at_or_before(19);
    assert(result.has_value());
    assert(result->timestamp_ns == 10);
    assert(!history.find_at_or_before(9).has_value());
}

void test_history_evicts_oldest_snapshot_at_capacity() {
    mosas::runtime::ImuStateHistory history(2);
    for (mosas::runtime::TimestampNs timestamp = 1; timestamp <= 3;
         ++timestamp) {
        history.push({timestamp, mosas::runtime::FlightPhase::pre_launch,
                      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
                      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}});
    }
    assert(history.size() == 2);
    assert(!history.find_at_or_before(1).has_value());
    assert(history.find_at_or_before(3)->timestamp_ns == 3);
}

void test_runtime_imu_worker_initializes_and_publishes_free_flight_state() {
    mosas::runtime::FlightPhaseDetectorConfig detector_config;
    detector_config.min_static_samples = 2;
    detector_config.min_static_duration_ns = 1;
    detector_config.max_static_axis_variation = 0.2;
    detector_config.ejection_start_threshold = 20.0;
    detector_config.ejection_confirm_duration_ns = 2;
    detector_config.free_flight_release_threshold = 3.0;
    detector_config.free_flight_confirm_duration_ns = 2;

    mosas::runtime::GuidanceRuntimeConfig config;
    config.phase_detector = detector_config;
    config.history_capacity = 16;
    config.max_imu_age_ns = 100;

    auto imu_source = std::make_unique<ScriptedImuSource>(
        std::vector<mosas::runtime::ImuSample>{
            {0, {3.0, 8.0, -4.0}, {0.0, 0.0, 0.0}},
            {1, {3.0, 8.0, -4.0}, {0.0, 0.0, 0.0}},
            {2, {23.0, 8.0, -4.0}, {0.0, 0.0, 0.0}},
            {4, {23.0, 8.0, -4.0}, {0.0, 0.0, 0.0}},
            {5, {4.0, 8.0, -4.0}, {0.0, 0.0, 0.0}},
            {7, {4.0, 8.0, -4.0}, {0.0, 0.0, 0.0}},
        });
    auto camera_source = std::make_unique<EmptyCameraSource>();
    auto command_sink = std::make_unique<EmptyCommandSink>();
    mosas::runtime::GuidanceRuntime runtime(
        std::move(imu_source), std::move(camera_source),
        std::move(command_sink), config);

    assert(runtime.start());
    assert(wait_until(
        [&runtime] {
            const auto state = runtime.latest_state();
            return state.has_value() &&
                   state->phase == mosas::runtime::FlightPhase::free_flight;
        },
        std::chrono::milliseconds(500)));
    const auto state = runtime.latest_state();
    assert(state.has_value());
    assert(std::abs(state->attitude.pitch) > 1e-6);
    runtime.stop();
    assert(!runtime.running());
}

void test_runtime_rejects_invalid_config_without_starting_workers() {
    mosas::runtime::GuidanceRuntimeConfig config;
    config.history_capacity = 0;
    auto imu_source = std::make_unique<EmptyImuSource>();
    auto camera_source = std::make_unique<EmptyCameraSource>();
    auto command_sink = std::make_unique<EmptyCommandSink>();
    mosas::runtime::GuidanceRuntime runtime(
        std::move(imu_source), std::move(camera_source),
        std::move(command_sink), config);

    assert(!runtime.start());
    assert(!runtime.running());
    assert(!runtime.last_error().empty());
}

void test_runtime_stop_cancels_blocking_sources() {
    auto imu_source = std::make_unique<BlockingImuSource>();
    BlockingImuSource* imu_source_view = imu_source.get();
    auto camera_source = std::make_unique<BlockingCameraSource>();
    BlockingCameraSource* camera_source_view = camera_source.get();
    auto command_sink = std::make_unique<EmptyCommandSink>();
    mosas::runtime::GuidanceRuntimeConfig config;
    config.history_capacity = 4;

    mosas::runtime::GuidanceRuntime runtime(
        std::move(imu_source), std::move(camera_source),
        std::move(command_sink), config);
    assert(runtime.start());
    runtime.stop();
    assert(imu_source_view->cancelled());
    assert(camera_source_view->cancelled());
}

void test_vision_worker_sends_command_only_for_fresh_free_flight_state() {
    auto camera_source = std::make_unique<ScriptedCameraSource>(
        std::vector<mosas::runtime::CameraFrame>{
            {1000000, make_guidance_frame()},
            {8000000, make_guidance_frame()},
            {9000000, make_guidance_frame()},
        });
    ScriptedCameraSource* camera_source_view = camera_source.get();
    auto command_sink = std::make_unique<RecordingCommandSink>();
    RecordingCommandSink* command_sink_view = command_sink.get();
    auto imu_source = std::make_unique<ScriptedImuSource>(
        free_flight_samples());

    mosas::runtime::GuidanceRuntimeConfig config;
    config.phase_detector = free_flight_detector_config();
    config.launch_speed_mps = 100.0;
    config.history_capacity = 32;
    config.max_imu_age_ns = 3000000;
    config.camera_intrinsics = {100.0, 100.0, 32.0, 24.0};
    config.vision_config.initial_roi = {0, 0, 64, 48};

    mosas::runtime::GuidanceRuntime runtime(
        std::move(imu_source), std::move(camera_source),
        std::move(command_sink), config);
    assert(runtime.start());
    assert(wait_until(
        [&runtime] {
            const auto state = runtime.latest_state();
            return state.has_value() &&
                   state->phase == mosas::runtime::FlightPhase::free_flight;
        },
        std::chrono::milliseconds(500)));
    camera_source_view->release();
    assert(wait_until(
        [command_sink_view] { return command_sink_view->size() >= 1; },
        std::chrono::milliseconds(500)));
    runtime.stop();

    assert(command_sink_view->size() == 1);
    const auto command = command_sink_view->command_at(0);
    assert(command.timestamp_ns == 9000000);
    assert(command.output.valid);
}

void test_stale_imu_state_does_not_send_command() {
    auto camera_source = std::make_unique<ScriptedCameraSource>(
        std::vector<mosas::runtime::CameraFrame>{{100000000,
                                                  make_guidance_frame()}});
    ScriptedCameraSource* camera_source_view = camera_source.get();
    auto command_sink = std::make_unique<RecordingCommandSink>();
    RecordingCommandSink* command_sink_view = command_sink.get();
    auto imu_source = std::make_unique<ScriptedImuSource>(
        free_flight_samples());
    mosas::runtime::GuidanceRuntimeConfig config;
    config.phase_detector = free_flight_detector_config();
    config.launch_speed_mps = 100.0;
    config.history_capacity = 32;
    config.max_imu_age_ns = 1000000;
    config.camera_intrinsics = {100.0, 100.0, 32.0, 24.0};
    config.vision_config.initial_roi = {0, 0, 64, 48};

    mosas::runtime::GuidanceRuntime runtime(
        std::move(imu_source), std::move(camera_source),
        std::move(command_sink), config);
    assert(runtime.start());
    assert(wait_until(
        [&runtime] { return runtime.latest_state().has_value(); },
        std::chrono::milliseconds(500)));
    camera_source_view->release();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    runtime.stop();
    assert(command_sink_view->size() == 0);
}

void test_non_monotonic_frame_timestamp_resets_los_rate_and_skips_frame() {
    auto camera_source = std::make_unique<ScriptedCameraSource>(
        std::vector<mosas::runtime::CameraFrame>{
            {8000000, make_guidance_frame()},
            {7000000, make_guidance_frame()},
        });
    ScriptedCameraSource* camera_source_view = camera_source.get();
    auto command_sink = std::make_unique<RecordingCommandSink>();
    RecordingCommandSink* command_sink_view = command_sink.get();
    auto imu_source = std::make_unique<ScriptedImuSource>(
        free_flight_samples());
    mosas::runtime::GuidanceRuntimeConfig config;
    config.phase_detector = free_flight_detector_config();
    config.launch_speed_mps = 100.0;
    config.history_capacity = 32;
    config.max_imu_age_ns = 3000000;
    config.camera_intrinsics = {100.0, 100.0, 32.0, 24.0};
    config.vision_config.initial_roi = {0, 0, 64, 48};

    mosas::runtime::GuidanceRuntime runtime(
        std::move(imu_source), std::move(camera_source),
        std::move(command_sink), config);
    assert(runtime.start());
    assert(wait_until(
        [&runtime] {
            const auto state = runtime.latest_state();
            return state.has_value() &&
                   state->phase == mosas::runtime::FlightPhase::free_flight;
        },
        std::chrono::milliseconds(500)));
    camera_source_view->release();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    runtime.stop();
    assert(command_sink_view->size() == 0);
}

void test_hardware_trigger_mode_is_forwarded_to_camera_source() {
    auto camera_source = std::make_unique<ScriptedCameraSource>(
        std::vector<mosas::runtime::CameraFrame>{});
    ScriptedCameraSource* camera_source_view = camera_source.get();
    auto imu_source = std::make_unique<EmptyImuSource>();
    auto command_sink = std::make_unique<EmptyCommandSink>();
    mosas::runtime::GuidanceRuntimeConfig config;
    config.camera_capture.mode =
        mosas::runtime::CameraCaptureMode::hardware_trigger;

    mosas::runtime::GuidanceRuntime runtime(
        std::move(imu_source), std::move(camera_source),
        std::move(command_sink), config);
    assert(runtime.start());
    assert(camera_source_view->configured());
    assert(camera_source_view->mode() ==
           mosas::runtime::CameraCaptureMode::hardware_trigger);
    runtime.stop();
}

void test_command_sink_failure_enters_fault_and_stops_future_sends() {
    auto camera_source = std::make_unique<ScriptedCameraSource>(
        std::vector<mosas::runtime::CameraFrame>{
            {8000000, make_guidance_frame()},
            {9000000, make_guidance_frame()},
            {10000000, make_guidance_frame()},
        });
    ScriptedCameraSource* camera_source_view = camera_source.get();
    auto command_sink = std::make_unique<FailingCommandSink>();
    FailingCommandSink* command_sink_view = command_sink.get();
    auto imu_source = std::make_unique<ScriptedImuSource>(
        free_flight_samples());
    mosas::runtime::GuidanceRuntimeConfig config;
    config.phase_detector = free_flight_detector_config();
    config.launch_speed_mps = 100.0;
    config.history_capacity = 32;
    config.max_imu_age_ns = 3000000;
    config.camera_intrinsics = {100.0, 100.0, 32.0, 24.0};
    config.vision_config.initial_roi = {0, 0, 64, 48};

    mosas::runtime::GuidanceRuntime runtime(
        std::move(imu_source), std::move(camera_source),
        std::move(command_sink), config);
    assert(runtime.start());
    assert(wait_until(
        [&runtime] {
            const auto state = runtime.latest_state();
            return state.has_value() &&
                   state->phase == mosas::runtime::FlightPhase::free_flight;
        },
        std::chrono::milliseconds(500)));
    camera_source_view->release();
    assert(wait_until(
        [&runtime] { return runtime.faulted(); },
        std::chrono::milliseconds(500)));
    const std::size_t sends_at_fault = command_sink_view->send_count();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    runtime.stop();
    assert(sends_at_fault == 1);
    assert(command_sink_view->send_count() == sends_at_fault);
    assert(!runtime.last_error().empty());
}

void test_worker_exception_enters_fault_and_cancels_other_source() {
    auto imu_source = std::make_unique<ThrowingImuSource>();
    auto camera_source = std::make_unique<BlockingCameraSource>();
    BlockingCameraSource* camera_source_view = camera_source.get();
    auto command_sink = std::make_unique<EmptyCommandSink>();
    mosas::runtime::GuidanceRuntimeConfig config;
    config.history_capacity = 4;

    mosas::runtime::GuidanceRuntime runtime(
        std::move(imu_source), std::move(camera_source),
        std::move(command_sink), config);
    assert(runtime.start());
    assert(wait_until(
        [&runtime] { return runtime.faulted(); },
        std::chrono::milliseconds(500)));
    assert(camera_source_view->cancelled());
    runtime.stop();
}

void test_source_timeout_is_not_a_runtime_fault() {
    auto imu_source = std::make_unique<EmptyImuSource>();
    auto camera_source = std::make_unique<BlockingCameraSource>();
    auto command_sink = std::make_unique<EmptyCommandSink>();
    mosas::runtime::GuidanceRuntimeConfig config;
    config.history_capacity = 4;

    mosas::runtime::GuidanceRuntime runtime(
        std::move(imu_source), std::move(camera_source),
        std::move(command_sink), config);
    assert(runtime.start());
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    assert(runtime.running());
    assert(!runtime.faulted());
    runtime.stop();
}

}  // namespace

int main() {
    test_runtime_types_are_constructible();
    test_non_horizontal_static_window_initializes_from_three_axis_baseline();
    test_short_elastic_impulse_transitions_to_ejection_then_free_flight();
    test_light_acceleration_does_not_trigger_launch();
    test_history_returns_latest_snapshot_not_after_frame_timestamp();
    test_history_evicts_oldest_snapshot_at_capacity();
    test_runtime_imu_worker_initializes_and_publishes_free_flight_state();
    test_runtime_rejects_invalid_config_without_starting_workers();
    test_runtime_stop_cancels_blocking_sources();
    test_vision_worker_sends_command_only_for_fresh_free_flight_state();
    test_stale_imu_state_does_not_send_command();
    test_non_monotonic_frame_timestamp_resets_los_rate_and_skips_frame();
    test_hardware_trigger_mode_is_forwarded_to_camera_source();
    test_command_sink_failure_enters_fault_and_stops_future_sends();
    test_worker_exception_enters_fault_and_cancels_other_source();
    test_source_timeout_is_not_a_runtime_fault();
    EmptyImuSource imu_source;
    EmptyCameraSource camera_source;
    EmptyCommandSink command_sink;
    (void)imu_source;
    (void)camera_source;
    (void)command_sink;
    return 0;
}
