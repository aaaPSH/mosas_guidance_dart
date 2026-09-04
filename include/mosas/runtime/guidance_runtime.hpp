#ifndef MOSAS_GUIDANCE_RUNTIME_HPP
#define MOSAS_GUIDANCE_RUNTIME_HPP

#include <mosas/guidance/guidance_estimator.hpp>
#include <mosas/runtime/flight_phase_detector.hpp>
#include <mosas/runtime/frame_rate_meter.hpp>
#include <mosas/runtime/imu_state_history.hpp>
#include <mosas/runtime/latest_frame_queue.hpp>
#include <mosas/runtime/runtime_interfaces.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
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
    double average_ms = 0.0;
    double maximum_ms = 0.0;
};

struct GuidanceRuntimeTiming {
    GuidanceRuntimeTimingStage capture;
    GuidanceRuntimeTimingStage prepare;
    GuidanceRuntimeTimingStage vision;
    GuidanceRuntimeTimingStage line_of_sight;
    GuidanceRuntimeTimingStage guidance;
    GuidanceRuntimeTimingStage command;
    GuidanceRuntimeTimingStage processing_total;
    GuidanceRuntimeTimingStage output;
};

struct GuidanceRuntimeConfig {
    // 测试模式：使用假基准跳过 IMU 静止自检。
    bool skip_imu_self_check = false;
    FlightPhaseDetectorConfig phase_detector{};
    double launch_speed_mps = 100.0;
    std::size_t history_capacity = 256;
    TimestampNs max_imu_age_ns = 50000000;
    VisionConfig vision_config{
        {0, 0, 320, 240}, {35, 85, 80, 255, 100, 255}, 40, 40, 8, 0.5,
        0.35};
    CameraIntrinsics camera_intrinsics{1.0, 1.0, 0.0, 0.0};
    RotationMatrix3 camera_to_body{
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0};
    LineOfSightRateFilterConfig los_rate_filter{};
    PngGuidanceConfig png_guidance{};
    CameraCaptureConfig camera_capture{};
    std::size_t capture_queue_capacity = 1;
    std::size_t output_queue_capacity = 1;
};

class GuidanceRuntime {
public:
    GuidanceRuntime(std::unique_ptr<ImuSource> imu_source,
                    std::unique_ptr<CameraSource> camera_source,
                    std::unique_ptr<CommandSink> command_sink,
                    const GuidanceRuntimeConfig& config,
                    std::unique_ptr<FrameSink> frame_sink = nullptr);
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
        VisionResult vision_result{};
        ImuStateSnapshot imu_state{};
        PngGuidanceOutput guidance{};
        VisionOverlayData overlay{};
    };

    bool validate_config() const;
    void imu_worker();
    void capture_worker();
    void processing_worker();
    void output_worker();
    enum class TimingStage {
        capture,
        prepare,
        vision,
        line_of_sight,
        guidance,
        command,
        processing_total,
        output,
    };

    struct TimingAccumulator {
        std::uint64_t samples = 0;
        std::uint64_t total_ns = 0;
        std::uint64_t maximum_ns = 0;
    };

    void record_timing(TimingStage stage,
                       std::chrono::steady_clock::duration duration) noexcept;
    void set_error(const std::string& message);
    void set_fault(const std::string& message);
    void cancel_sources() noexcept;

    std::unique_ptr<ImuSource> imu_source_;
    std::unique_ptr<CameraSource> camera_source_;
    std::unique_ptr<CommandSink> command_sink_;
    std::unique_ptr<FrameSink> frame_sink_;
    GuidanceRuntimeConfig config_;
    LatestFrameQueue<CameraFrame> capture_queue_;
    LatestFrameQueue<ProcessedFrame> output_queue_;
    FlightPhaseDetector phase_detector_;
    ImuStateHistory state_history_;
    DartCondition dart_condition_;

    std::atomic<bool> stop_requested_{false};
    std::atomic<bool> running_{false};
    std::atomic<bool> faulted_{false};
    std::atomic<bool> sources_cancelled_{false};
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
    TimingAccumulator prepare_timing_;
    TimingAccumulator vision_timing_;
    TimingAccumulator line_of_sight_timing_;
    TimingAccumulator guidance_timing_;
    TimingAccumulator command_timing_;
    TimingAccumulator processing_total_timing_;
    TimingAccumulator output_timing_;
    mutable std::mutex error_mutex_;
    std::string last_error_;
};

}  // namespace mosas::runtime

#endif  // MOSAS_GUIDANCE_RUNTIME_HPP
