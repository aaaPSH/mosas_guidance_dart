#ifndef MOSAS_RUNTIME_FRAME_PIPELINE_HPP
#define MOSAS_RUNTIME_FRAME_PIPELINE_HPP

#include <mosas/runtime/frame_rate_meter.hpp>
#include <mosas/runtime/latest_frame_queue.hpp>
#include <mosas/runtime/runtime_interfaces.hpp>
#include <mosas/runtime/runtime_supervisor.hpp>
#include <mosas/runtime/timing_distribution.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace mosas::runtime {

class ImuPipeline;
class LaunchGenerationCoordinator;
class OutputDispatcher;

struct FramePipelineCallbacks {
    std::function<bool()> stop_requested;
    std::function<bool()> faulted;
    std::function<void(std::string)> report_fault;
    std::function<void(const std::string&)> log;
};

class FramePipeline final {
  public:
    FramePipeline(std::unique_ptr<CameraSource> camera_source,
                  std::unique_ptr<ports::IVisionDetector> vision_detector,
                  std::unique_ptr<ports::ILosEstimator> los_estimator,
                  std::unique_ptr<ports::ILosRateFilter> los_rate_filter,
                  std::unique_ptr<ports::IGuidanceLaw> guidance_law,
                  RuntimeSupervisorConfig config, ImuPipeline& imu_pipeline,
                  CommandSink& command_sink,
                  LaunchGenerationCoordinator& generation_coordinator,
                  OutputDispatcher& output_dispatcher,
                  FramePipelineCallbacks callbacks);
    ~FramePipeline();

    FramePipeline(const FramePipeline&) = delete;
    FramePipeline& operator=(const FramePipeline&) = delete;

    bool valid() const noexcept;
    SourceResult configure_source();
    void reset() noexcept;
    void start_workers();
    void cancel_source() noexcept;
    void join_capture_worker() noexcept;
    void stop_source() noexcept;
    void close_queue() noexcept;
    void join_processing_worker() noexcept;

    double processing_fps() const;
    RuntimeStatistics statistics() const;
    RuntimeTiming timing() const;

  private:
    struct QueuedCameraFrame {
        CameraFrame frame;
        std::chrono::steady_clock::time_point enqueued_at{};
    };

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
    };

    struct TimingAccumulator {
        TimingDistribution distribution;
    };

    void capture_worker() noexcept;
    void processing_worker() noexcept;
    void record_timing(TimingStage stage,
                       std::chrono::steady_clock::duration duration) noexcept;
    void report_fault(std::string message) noexcept;
    void log(const std::string& message) const;

    std::unique_ptr<CameraSource> camera_source_;
    std::unique_ptr<ports::IVisionDetector> vision_detector_;
    std::unique_ptr<ports::ILosEstimator> los_estimator_;
    std::unique_ptr<ports::ILosRateFilter> los_rate_filter_;
    std::unique_ptr<ports::IGuidanceLaw> guidance_law_;
    RuntimeSupervisorConfig config_;
    ImuPipeline& imu_pipeline_;
    CommandSink& command_sink_;
    LaunchGenerationCoordinator& generation_coordinator_;
    OutputDispatcher& output_dispatcher_;
    FramePipelineCallbacks callbacks_;
    LatestFrameQueue<QueuedCameraFrame> capture_queue_;
    std::thread capture_thread_;
    std::thread processing_thread_;
    FrameRateMeter capture_fps_meter_;
    FrameRateMeter processing_fps_meter_;
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
};

} // namespace mosas::runtime

#endif // MOSAS_RUNTIME_FRAME_PIPELINE_HPP
