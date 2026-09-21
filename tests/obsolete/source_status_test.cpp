#include <mosas/app/host_adapters.hpp>
#include <mosas/runtime/guidance_runtime.hpp>

#include <opencv2/core.hpp>

#include <atomic>
#include <algorithm>
#include <cassert>
#include <chrono>
#include <condition_variable>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace {

using mosas::runtime::CameraCaptureConfig;
using mosas::runtime::CameraFrame;
using mosas::runtime::CameraSource;
using mosas::runtime::FrameSink;
using mosas::runtime::GuidanceRuntime;
using mosas::runtime::GuidanceRuntimeConfig;
using mosas::runtime::ImuSample;
using mosas::runtime::ImuSource;
using mosas::runtime::SourceResult;
using mosas::runtime::SourceStatus;

class ScriptedImuSource final : public ImuSource {
public:
    ScriptedImuSource(SourceResult result, bool invalid_sample = false)
        : result_(std::move(result)), invalid_sample_(invalid_sample) {}

    SourceResult read(ImuSample* sample) override {
        if (sample == nullptr) {
            return {SourceStatus::fatal, "imu sample output is null"};
        }
        if (cancelled_.load()) {
            return {SourceStatus::cancelled, "test imu source cancelled"};
        }
        if (result_.status == SourceStatus::ok) {
            sample->timestamp_ns = invalid_sample_ ? -1 : steady_timestamp_ns();
            sample->acceleration = {0.0, 0.0, 0.0};
            sample->angular_velocity = {0.0, 0.0, 0.0};
        }
        return result_;
    }

    void cancel() noexcept override { cancelled_.store(true); }

private:
    static mosas::runtime::TimestampNs steady_timestamp_ns() {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
                   std::chrono::steady_clock::now().time_since_epoch())
            .count();
    }

    SourceResult result_;
    bool invalid_sample_ = false;
    std::atomic<bool> cancelled_{false};
};

class BlockingCameraSource final : public CameraSource {
public:
    explicit BlockingCameraSource(SourceResult configure_result = {
        SourceStatus::ok, {}})
        : configure_result_(std::move(configure_result)) {}

    SourceResult configure(const CameraCaptureConfig&) override {
        return configure_result_;
    }

    SourceResult capture(CameraFrame*) override {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait(lock, [this] { return cancelled_; });
        return {SourceStatus::cancelled, "test camera source cancelled"};
    }

    SourceResult prepare(CameraFrame*) override {
        return {SourceStatus::ok, {}};
    }

    void cancel() noexcept override {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            cancelled_ = true;
        }
        condition_.notify_all();
    }

private:
    SourceResult configure_result_;
    std::mutex mutex_;
    std::condition_variable condition_;
    bool cancelled_ = false;
};

class SingleFrameCameraSource final : public CameraSource {
public:
    explicit SingleFrameCameraSource(int initial_delay_ms = 0)
        : initial_delay_ms_(initial_delay_ms) {}

    SourceResult configure(const CameraCaptureConfig&) override {
        return {SourceStatus::ok, {}};
    }

    SourceResult capture(CameraFrame* frame) override {
        if (frame == nullptr) {
            return {SourceStatus::fatal, "test camera frame is null"};
        }
        std::unique_lock<std::mutex> lock(mutex_);
        if (!sent_) {
            if (initial_delay_ms_ > 0) {
                lock.unlock();
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(initial_delay_ms_));
                lock.lock();
                if (cancelled_) {
                    return {SourceStatus::cancelled,
                            "test camera source cancelled"};
                }
            }
            sent_ = true;
            frame->timestamp_ns = steady_timestamp_ns();
            frame->image = cv::Mat::zeros(8, 8, CV_8UC3);
            return {SourceStatus::ok, {}};
        }
        condition_.wait(lock, [this] { return cancelled_; });
        return {SourceStatus::cancelled, "test camera source cancelled"};
    }

    void cancel() noexcept override {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            cancelled_ = true;
        }
        condition_.notify_all();
    }

private:
    static mosas::runtime::TimestampNs steady_timestamp_ns() {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
                   std::chrono::steady_clock::now().time_since_epoch())
            .count();
    }

    std::mutex mutex_;
    std::condition_variable condition_;
    int initial_delay_ms_ = 0;
    bool sent_ = false;
    bool cancelled_ = false;
};

class OneValidImuSource final : public ImuSource {
public:
    SourceResult read(ImuSample* sample) override {
        if (sample == nullptr) {
            return {SourceStatus::fatal, "test imu sample is null"};
        }
        if (cancelled_.load()) {
            return {SourceStatus::cancelled, "test imu source cancelled"};
        }
        if (!sent_) {
            sent_ = true;
            sample->timestamp_ns = steady_timestamp_ns();
            sample->acceleration = {0.0, 9.80665, 0.0};
            sample->angular_velocity = {0.0, 0.0, 0.0};
            return {SourceStatus::ok, {}};
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        return cancelled_.load()
                   ? SourceResult{SourceStatus::cancelled,
                                  "test imu source cancelled"}
                   : SourceResult{SourceStatus::timeout, "test IMU timeout"};
    }

    void cancel() noexcept override { cancelled_.store(true); }

private:
    static mosas::runtime::TimestampNs steady_timestamp_ns() {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
                   std::chrono::steady_clock::now().time_since_epoch())
            .count();
    }

    bool sent_ = false;
    std::atomic<bool> cancelled_{false};
};

class RecordingFrameSink final : public FrameSink {
public:
    bool publish(const CameraFrame&, const VisionResult&,
                 const mosas::runtime::ImuStateSnapshot&,
                 const PngGuidanceOutput&) override {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            published_ = true;
        }
        condition_.notify_all();
        return true;
    }

    void cancel() noexcept override {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            cancelled_ = true;
        }
        condition_.notify_all();
    }

    bool wait_for_publish() {
        std::unique_lock<std::mutex> lock(mutex_);
        return condition_.wait_for(lock, std::chrono::milliseconds(500),
                                   [this] { return published_ || cancelled_; }) &&
               published_;
    }

private:
    std::mutex mutex_;
    std::condition_variable condition_;
    bool published_ = false;
    bool cancelled_ = false;
};

class OrderedCameraSource final : public CameraSource {
public:
    SourceResult configure(const CameraCaptureConfig&) override {
        return {SourceStatus::ok, {}};
    }

    SourceResult capture(CameraFrame*) override {
        std::unique_lock<std::mutex> lock(mutex_);
        events_.emplace_back("capture_started");
        condition_.notify_all();
        condition_.wait(lock, [this] { return cancelled_; });
        events_.emplace_back("capture_exited");
        return {SourceStatus::cancelled, "test camera source cancelled"};
    }

    void cancel() noexcept override {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            events_.emplace_back("cancel");
            cancelled_ = true;
        }
        condition_.notify_all();
    }

    void stop() noexcept override {
        std::lock_guard<std::mutex> lock(mutex_);
        events_.emplace_back("stop");
        stopped_after_capture_ =
            std::find(events_.begin(), events_.end(), "capture_exited") !=
            events_.end();
    }

    void wait_for_capture_started() {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait(lock, [this] {
            return std::find(events_.begin(), events_.end(),
                             "capture_started") != events_.end();
        });
    }

    bool stopped_after_capture() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return stopped_after_capture_;
    }

    std::vector<std::string> events() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return events_;
    }

private:
    mutable std::mutex mutex_;
    std::condition_variable condition_;
    std::vector<std::string> events_;
    bool cancelled_ = false;
    bool stopped_after_capture_ = false;
};

GuidanceRuntimeConfig test_config() {
    GuidanceRuntimeConfig config;
    config.skip_imu_self_check = true;
    return config;
}

bool wait_for_fault(const GuidanceRuntime& runtime) {
    for (int attempt = 0; attempt < 500; ++attempt) {
        if (runtime.faulted()) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return runtime.faulted();
}

void test_runtime_timeout_does_not_fault() {
    auto imu = std::make_unique<ScriptedImuSource>(
        SourceResult{SourceStatus::timeout, {}});
    auto camera = std::make_unique<BlockingCameraSource>();
    auto command = std::make_unique<mosas::app::LoggingCommandSink>(
        std::cerr);
    GuidanceRuntime runtime(std::move(imu), std::move(camera),
                            std::move(command), test_config());

    assert(runtime.start());
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    assert(!runtime.faulted());
    runtime.stop();
    assert(!runtime.faulted());
}

void test_runtime_logs_imu_timeout_and_frame_match_failure() {
    auto imu = std::make_unique<ScriptedImuSource>(
        SourceResult{SourceStatus::timeout, "IMU 读取超时"});
    auto camera = std::make_unique<SingleFrameCameraSource>();
    auto command = std::make_unique<mosas::app::LoggingCommandSink>(
        std::cerr);
    auto frame_sink = std::make_unique<RecordingFrameSink>();
    auto* frame_sink_ptr = frame_sink.get();
    std::ostringstream log;
    GuidanceRuntime runtime(std::move(imu), std::move(camera),
                            std::move(command), test_config(),
                            std::move(frame_sink), log);

    assert(runtime.start());
    assert(frame_sink_ptr->wait_for_publish());
    runtime.stop();

    const std::string output = log.str();
    assert(output.find("[运行时]") != std::string::npos);
    assert(output.find("IMU 读取超时") != std::string::npos);
    assert(output.find("相机帧无法匹配可用 IMU 状态") != std::string::npos);
    assert(output.find("IMU 状态历史为空") != std::string::npos);
    assert(output.find("camera_timestamp_ns=") != std::string::npos);
    assert(output.find("imu_source_diagnostics=") != std::string::npos);
}

void test_runtime_logs_vision_n_a_reason_after_imu_is_ready() {
    auto imu = std::make_unique<OneValidImuSource>();
    auto camera = std::make_unique<SingleFrameCameraSource>(10);
    auto command = std::make_unique<mosas::app::LoggingCommandSink>(
        std::cerr);
    auto frame_sink = std::make_unique<RecordingFrameSink>();
    auto* frame_sink_ptr = frame_sink.get();
    std::ostringstream log;
    GuidanceRuntime runtime(std::move(imu), std::move(camera),
                            std::move(command), test_config(),
                            std::move(frame_sink), log);

    assert(runtime.start());
    assert(frame_sink_ptr->wait_for_publish());
    runtime.stop();

    const std::string output = log.str();
    assert(output.find("视觉未找到目标") != std::string::npos);
    assert(output.find("camera_timestamp_ns=") != std::string::npos);
}

void test_runtime_fatal_propagates_source_message() {
    auto imu = std::make_unique<ScriptedImuSource>(SourceResult{
        SourceStatus::fatal, "sensor offline"});
    auto camera = std::make_unique<BlockingCameraSource>();
    auto command = std::make_unique<mosas::app::LoggingCommandSink>(
        std::cerr);
    GuidanceRuntime runtime(std::move(imu), std::move(camera),
                            std::move(command), test_config());

    assert(runtime.start());
    assert(wait_for_fault(runtime));
    assert(runtime.last_error().find("sensor offline") != std::string::npos);
    runtime.stop();
}

void test_runtime_unexpected_cancel_is_fault() {
    auto imu = std::make_unique<ScriptedImuSource>(SourceResult{
        SourceStatus::cancelled, "sensor stopped unexpectedly"});
    auto camera = std::make_unique<BlockingCameraSource>();
    auto command = std::make_unique<mosas::app::LoggingCommandSink>(
        std::cerr);
    GuidanceRuntime runtime(std::move(imu), std::move(camera),
                            std::move(command), test_config());

    assert(runtime.start());
    assert(wait_for_fault(runtime));
    assert(runtime.last_error().find("sensor stopped unexpectedly") !=
           std::string::npos);
    runtime.stop();
}

void test_runtime_configure_failure_propagates_source_message() {
    auto imu = std::make_unique<ScriptedImuSource>(
        SourceResult{SourceStatus::timeout, {}});
    auto camera = std::make_unique<BlockingCameraSource>(SourceResult{
        SourceStatus::fatal, "camera open failed"});
    auto command = std::make_unique<mosas::app::LoggingCommandSink>(
        std::cerr);
    GuidanceRuntime runtime(std::move(imu), std::move(camera),
                            std::move(command), test_config());

    assert(!runtime.start());
    assert(runtime.faulted());
    assert(runtime.last_error().find("camera open failed") !=
           std::string::npos);
    runtime.stop();
}

void test_runtime_ok_with_invalid_imu_sample_is_fault() {
    auto imu = std::make_unique<ScriptedImuSource>(
        SourceResult{SourceStatus::ok, {}}, true);
    auto camera = std::make_unique<BlockingCameraSource>();
    auto command = std::make_unique<mosas::app::LoggingCommandSink>(
        std::cerr);
    GuidanceRuntime runtime(std::move(imu), std::move(camera),
                            std::move(command), test_config());

    assert(runtime.start());
    assert(wait_for_fault(runtime));
    assert(runtime.last_error().find("invalid sample") != std::string::npos);
    runtime.stop();
}

void test_runtime_stops_camera_after_capture_worker_exits() {
    auto imu = std::make_unique<ScriptedImuSource>(
        SourceResult{SourceStatus::timeout, {}});
    auto camera = std::make_unique<OrderedCameraSource>();
    auto* camera_ptr = camera.get();
    auto command = std::make_unique<mosas::app::LoggingCommandSink>(
        std::cerr);
    GuidanceRuntime runtime(std::move(imu), std::move(camera),
                            std::move(command), test_config());

    assert(runtime.start());
    camera_ptr->wait_for_capture_started();
    runtime.stop();

    assert(camera_ptr->stopped_after_capture());
    const auto events = camera_ptr->events();
    const auto cancel = std::find(events.begin(), events.end(), "cancel");
    const auto capture_exited =
        std::find(events.begin(), events.end(), "capture_exited");
    const auto stop = std::find(events.begin(), events.end(), "stop");
    assert(cancel != events.end());
    assert(capture_exited != events.end());
    assert(stop != events.end());
    assert(cancel < capture_exited);
    assert(capture_exited < stop);
}

void test_simulated_imu_source_reports_fatal_and_cancelled() {
    mosas::app::SimulatedImuSource source(1000.0);
    const SourceResult null_result = source.read(nullptr);
    assert(null_result.status == SourceStatus::fatal);

    source.cancel();
    ImuSample sample{};
    const SourceResult cancelled_result = source.read(&sample);
    assert(cancelled_result.status == SourceStatus::cancelled);
}

}  // 匿名命名空间结束

int main() {
    test_runtime_timeout_does_not_fault();
    test_runtime_logs_imu_timeout_and_frame_match_failure();
    test_runtime_logs_vision_n_a_reason_after_imu_is_ready();
    test_runtime_fatal_propagates_source_message();
    test_runtime_unexpected_cancel_is_fault();
    test_runtime_configure_failure_propagates_source_message();
    test_runtime_ok_with_invalid_imu_sample_is_fault();
    test_runtime_stops_camera_after_capture_worker_exits();
    test_simulated_imu_source_reports_fatal_and_cancelled();
    std::cout << "source_status_test passed\n";
    return 0;
}
