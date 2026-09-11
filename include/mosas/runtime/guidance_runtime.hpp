#ifndef MOSAS_GUIDANCE_RUNTIME_HPP
#define MOSAS_GUIDANCE_RUNTIME_HPP

#include <mosas/guidance/guidance_estimator.hpp>
#include <mosas/runtime/control_mailbox.hpp>
#include <mosas/runtime/flight_phase_detector.hpp>
#include <mosas/runtime/frame_rate_meter.hpp>
#include <mosas/runtime/imu_state_history.hpp>
#include <mosas/runtime/latest_frame_queue.hpp>
#include <mosas/runtime/runtime_interfaces.hpp>
#include <mosas/runtime/runtime_timing_statistics.hpp>

#include <atomic>
#include <chrono>
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
    std::size_t processing_queue_depth = 0;
    std::size_t processing_queue_high_water_mark = 0;
    std::uint64_t frame_drop_count = 0;
    std::uint64_t control_command_generated_count = 0;
    std::uint64_t control_command_sent_count = 0;
    std::uint64_t control_command_superseded_count = 0;
    std::uint64_t stale_command_count = 0;
    std::uint64_t send_failure_count = 0;
};

struct GuidanceRuntimeTimingStage {
    std::uint64_t samples = 0;
    double average_ms = 0.0;
    double p95_ms = 0.0;
    double p99_ms = 0.0;
    double maximum_ms = 0.0;
    std::uint64_t deadline_miss_count = 0;
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
    // Zero disables the optional stale-command guard until hardware
    // validation establishes the correct age threshold.
    TimestampNs max_command_age_ns = 0;
    // Zero disables no-new-command watchdog thresholding.
    TimestampNs control_watchdog_timeout_ns = 0;
    // Zero disables processing deadline accounting.
    TimestampNs processing_deadline_ns = 0;
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

enum class ControlWatchdogState {
    no_command,
    healthy,
    stale_command,
    send_failure,
    device_disconnected,
    shutdown,
};

inline const char* control_watchdog_state_name(
    ControlWatchdogState state) noexcept {
    switch (state) {
        case ControlWatchdogState::no_command:
            return "no_command";
        case ControlWatchdogState::healthy:
            return "healthy";
        case ControlWatchdogState::stale_command:
            return "stale_command";
        case ControlWatchdogState::send_failure:
            return "send_failure";
        case ControlWatchdogState::device_disconnected:
            return "device_disconnected";
        case ControlWatchdogState::shutdown:
            return "shutdown";
    }
    return "unknown";
}

struct GuidanceRuntimeControlStatus {
    ControlWatchdogState state = ControlWatchdogState::no_command;
    std::uint64_t last_generated_sequence = 0;
    std::uint64_t last_sent_sequence = 0;
    TimestampNs last_generated_time_ns = -1;
    TimestampNs last_sent_time_ns = -1;
    std::uint64_t consecutive_send_failures = 0;
    std::uint64_t consecutive_stale_commands = 0;
    std::uint64_t last_stale_sequence = 0;
    TimestampNs last_stale_age_ns = 0;
};

class GuidanceRuntime {
public:
    GuidanceRuntime(std::unique_ptr<ImuSource> imu_source,
                    std::unique_ptr<CameraSource> camera_source,
                    std::unique_ptr<CommandSink> command_sink,
                    const GuidanceRuntimeConfig& config,
                    std::unique_ptr<FrameSink> frame_sink = nullptr);
    GuidanceRuntime(std::unique_ptr<ImuSource> imu_source,
                    std::unique_ptr<CameraSource> camera_source,
                    std::unique_ptr<CommandSink> command_sink,
                    const GuidanceRuntimeConfig& config,
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
    GuidanceRuntimeControlStatus control_status() const;

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

    void record_timing(TimingStage stage,
                       std::chrono::steady_clock::duration duration) noexcept;
    void log(const std::string& message);
    void set_error(const std::string& message);
    void set_fault(const std::string& message);
    void cancel_sources() noexcept;
    void control_output_worker();

    std::unique_ptr<ImuSource> imu_source_;
    std::unique_ptr<CameraSource> camera_source_;
    std::unique_ptr<CommandSink> command_sink_;
    std::unique_ptr<FrameSink> frame_sink_;
    GuidanceRuntimeConfig config_;
    std::ostream* output_ = nullptr;
    LatestFrameQueue<CameraFrame> capture_queue_;
    LatestFrameQueue<ProcessedFrame> output_queue_;
    FlightPhaseDetector phase_detector_;
    ImuStateHistory state_history_;
    DartCondition dart_condition_;
    ControlMailbox control_mailbox_;

    std::atomic<bool> stop_requested_{false};
    std::atomic<bool> running_{false};
    std::atomic<bool> faulted_{false};
    std::atomic<bool> sources_cancelled_{false};
    std::thread imu_thread_;
    std::thread capture_thread_;
    std::thread processing_thread_;
    std::thread output_thread_;
    std::thread control_thread_;

    mutable std::mutex state_mutex_;
    std::optional<ImuStateSnapshot> latest_state_;
    FrameRateMeter capture_fps_meter_;
    FrameRateMeter processing_fps_meter_;
    FrameRateMeter output_fps_meter_;
    RuntimeTimingAccumulator capture_timing_;
    RuntimeTimingAccumulator prepare_timing_;
    RuntimeTimingAccumulator vision_timing_;
    RuntimeTimingAccumulator line_of_sight_timing_;
    RuntimeTimingAccumulator guidance_timing_;
    RuntimeTimingAccumulator command_timing_;
    RuntimeTimingAccumulator processing_total_timing_;
    RuntimeTimingAccumulator output_timing_;
    std::atomic<std::uint64_t> control_command_generated_count_{0};
    std::atomic<std::uint64_t> control_command_sent_count_{0};
    std::atomic<std::uint64_t> stale_command_count_{0};
    std::atomic<std::uint64_t> send_failure_count_{0};
    std::atomic<ControlWatchdogState> control_watchdog_state_{
        ControlWatchdogState::no_command};
    std::atomic<std::uint64_t> last_generated_sequence_{0};
    std::atomic<std::uint64_t> last_sent_sequence_{0};
    std::atomic<TimestampNs> last_generated_time_ns_{-1};
    std::atomic<TimestampNs> last_sent_time_ns_{-1};
    std::atomic<std::uint64_t> consecutive_send_failures_{0};
    std::atomic<std::uint64_t> consecutive_stale_commands_{0};
    std::atomic<std::uint64_t> last_stale_sequence_{0};
    std::atomic<TimestampNs> last_stale_age_ns_{0};
    mutable std::mutex error_mutex_;
    std::string last_error_;
    mutable std::mutex log_mutex_;
};

}  // namespace mosas::runtime

#endif  // MOSAS_GUIDANCE_RUNTIME_HPP
