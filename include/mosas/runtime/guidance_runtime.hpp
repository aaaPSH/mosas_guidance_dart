#ifndef MOSAS_GUIDANCE_RUNTIME_HPP
#define MOSAS_GUIDANCE_RUNTIME_HPP

#include <mosas/domain/algorithm_results.hpp>
#include <mosas/ports/clock.hpp>
#include <mosas/ports/flight_state_estimator.hpp>
#include <mosas/ports/guidance_law.hpp>
#include <mosas/ports/los_estimator.hpp>
#include <mosas/ports/los_rate_filter.hpp>
#include <mosas/ports/vision_detector.hpp>
#include <mosas/runtime/async_info_logger.hpp>
#include <mosas/runtime/attitude_history.hpp>
#include <mosas/runtime/frame_rate_meter.hpp>
#include <mosas/runtime/imu_angular_velocity_history.hpp>
#include <mosas/runtime/latest_frame_queue.hpp>
#include <mosas/runtime/runtime_interfaces.hpp>
#include <mosas/runtime/timing_distribution.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <iosfwd>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

namespace mosas::runtime {

struct GuidanceRuntimeStatistics {
    double capture_fps = 0.0;
    double processing_fps = 0.0;
    double output_fps = 0.0;
};

struct GuidanceRuntimeTimingStage {
    std::uint64_t samples = 0;
    double minimum_ms = 0.0;
    double average_ms = 0.0;
    double p50_ms = 0.0;
    double p95_ms = 0.0;
    double p99_ms = 0.0;
    double maximum_ms = 0.0;
};

struct GuidanceRuntimeTiming {
    GuidanceRuntimeTimingStage capture;
    GuidanceRuntimeTimingStage capture_queue_wait;
    GuidanceRuntimeTimingStage capture_queue_delay;
    GuidanceRuntimeTimingStage prepare;
    GuidanceRuntimeTimingStage vision;
    GuidanceRuntimeTimingStage line_of_sight;
    GuidanceRuntimeTimingStage guidance;
    GuidanceRuntimeTimingStage command;
    GuidanceRuntimeTimingStage processing_total;
    GuidanceRuntimeTimingStage output_queue_wait;
    GuidanceRuntimeTimingStage output_queue_delay;
    GuidanceRuntimeTimingStage output;
    std::uint64_t capture_queue_dropped = 0;
    std::uint64_t output_queue_dropped = 0;
};

struct GuidanceRuntimeConfig {
    double launch_speed_mps = 100.0;
    std::size_t history_capacity = 256;
    TimestampNs max_imu_age_ns = 50000000;
    TimestampNs max_camera_future_ns = 50000000;
    TimestampNs max_camera_age_ns = 2000000000;
    domain::CameraIntrinsics camera_intrinsics{1.0, 1.0, 0.0, 0.0};
    domain::Rotation3<domain::DartFrameTag, domain::CameraFrameTag>
        dart_from_camera{};
    CameraCaptureConfig camera_capture{};
    std::size_t capture_queue_capacity = 1;
    std::size_t output_queue_capacity = 1;
    std::size_t info_log_queue_capacity = 256;
};

struct GuidanceRuntimeDependencies {
    std::unique_ptr<ports::IVisionDetector> vision_detector;
    std::unique_ptr<ports::IFlightStateEstimator> flight_state_estimator;
    std::unique_ptr<ports::ILosEstimator> los_estimator;
    std::unique_ptr<ports::ILosRateFilter> los_rate_filter;
    std::unique_ptr<ports::IGuidanceLaw> guidance_law;
    std::unique_ptr<ports::IMonotonicClock> monotonic_clock;
};

class GuidanceRuntime {
public:
    GuidanceRuntime(std::unique_ptr<ImuSource> imu_source,
                    std::unique_ptr<CameraSource> camera_source,
                    std::unique_ptr<CommandSink> command_sink,
                    const GuidanceRuntimeConfig& config,
                    GuidanceRuntimeDependencies dependencies,
                    std::unique_ptr<FrameSink> frame_sink = nullptr);
    GuidanceRuntime(std::unique_ptr<ImuSource> imu_source,
                    std::unique_ptr<CameraSource> camera_source,
                    std::unique_ptr<CommandSink> command_sink,
                    const GuidanceRuntimeConfig& config,
                    GuidanceRuntimeDependencies dependencies,
                    std::unique_ptr<FrameSink> frame_sink,
                    std::ostream& output);
    ~GuidanceRuntime();

    GuidanceRuntime(const GuidanceRuntime&) = delete;
    GuidanceRuntime& operator=(const GuidanceRuntime&) = delete;

    bool start();
    void stop() noexcept;

    bool running() const noexcept;
    bool faulted() const noexcept;
    std::string last_error() const;
    std::optional<ImuStateSnapshot> latest_state() const;
    GuidanceRuntimeStatistics statistics() const;
    GuidanceRuntimeTiming timing() const;

private:
    struct ProcessedFrame {
        CameraFrame frame;
        domain::VisionDetection vision_detection{};
        domain::FlightStateSnapshot flight_state{};
        domain::GuidanceOutput guidance{};
        domain::GuidanceOverlay overlay{};
        std::chrono::steady_clock::time_point enqueued_at{};
    };

    struct QueuedCameraFrame {
        CameraFrame frame;
        std::chrono::steady_clock::time_point enqueued_at{};
    };

    bool validate_config() const;
    void imu_worker();
    void capture_worker();
    void processing_worker();
    void output_worker();
    enum class TimingStage {
        capture,
        capture_queue_wait,
        capture_queue_delay,
        prepare,
        vision,
        line_of_sight,
        guidance,
        command,
        processing_total,
        output_queue_wait,
        output_queue_delay,
        output,
    };

    struct TimingAccumulator {
        TimingDistribution distribution;
    };

    void record_timing(TimingStage stage,
                       std::chrono::steady_clock::duration duration) noexcept;
    void log(const std::string& message);
    void set_error(const std::string& message);
    void set_fault(const std::string& message);
    void cancel_sources() noexcept;

    std::unique_ptr<ImuSource> imu_source_;
    std::unique_ptr<CameraSource> camera_source_;
    std::unique_ptr<CommandSink> command_sink_;
    std::unique_ptr<FrameSink> frame_sink_;
    GuidanceRuntimeConfig config_;
    GuidanceRuntimeDependencies dependencies_;
    std::ostream* output_ = nullptr;
    AsyncInfoLogger info_logger_;
    LatestFrameQueue<QueuedCameraFrame> capture_queue_;
    LatestFrameQueue<ProcessedFrame> output_queue_;
    ImuAngularVelocityHistory angular_velocity_history_;
    AttitudeHistory attitude_history_;

    std::atomic<bool> stop_requested_{false};
    std::atomic<bool> running_{false};
    std::atomic<bool> faulted_{false};
    std::atomic<bool> sources_cancelled_{false};
    std::atomic<std::uint64_t> active_launch_generation_{0};
    // 序列化周期切换、IMU 样本入历史和命令提交，明确新旧周期边界。
    mutable std::mutex launch_generation_mutex_;
    std::thread imu_thread_;
    std::thread capture_thread_;
    std::thread processing_thread_;
    std::thread output_thread_;

    mutable std::mutex state_mutex_;
    std::optional<ImuStateSnapshot> latest_state_;
    FrameRateMeter capture_fps_meter_;
    FrameRateMeter processing_fps_meter_;
    FrameRateMeter output_fps_meter_;
    mutable std::mutex timing_mutex_;
    TimingAccumulator capture_timing_;
    TimingAccumulator capture_queue_wait_timing_;
    TimingAccumulator capture_queue_delay_timing_;
    TimingAccumulator prepare_timing_;
    TimingAccumulator vision_timing_;
    TimingAccumulator line_of_sight_timing_;
    TimingAccumulator guidance_timing_;
    TimingAccumulator command_timing_;
    TimingAccumulator processing_total_timing_;
    TimingAccumulator output_queue_wait_timing_;
    TimingAccumulator output_queue_delay_timing_;
    TimingAccumulator output_timing_;
    mutable std::mutex error_mutex_;
    std::string last_error_;
    mutable std::mutex log_mutex_;
};

}  // namespace mosas::runtime

#endif  // MOSAS_GUIDANCE_RUNTIME_HPP
