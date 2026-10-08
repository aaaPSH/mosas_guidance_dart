#ifndef MOSAS_RUNTIME_OUTPUT_DISPATCHER_HPP
#define MOSAS_RUNTIME_OUTPUT_DISPATCHER_HPP

#include <mosas/runtime/latest_frame_queue.hpp>
#include <mosas/runtime/output_ports.hpp>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace mosas::runtime {

enum class OutputDispatchTimingStage {
    queue_wait,
    queue_delay,
    publish,
};

struct OutputDispatcherCallbacks {
    std::function<bool()> stop_requested;
    std::function<std::uint64_t()> active_generation;
    std::function<double()> processing_fps;
    std::function<void(OutputDispatchTimingStage,
                       std::chrono::steady_clock::duration)> record_timing;
    std::function<void()> record_published_frame;
    std::function<void()> request_rearm;
    std::function<void(std::string)> report_fault;
};

class OutputFrameWorker;

class OutputDispatcher final {
public:
    OutputDispatcher(std::unique_ptr<IVideoRecorder> video_recorder,
                     std::unique_ptr<ITelemetryRecorder> telemetry_recorder,
                     std::unique_ptr<IVisualizationSink> visualization_sink,
                     std::unique_ptr<IImuRecorder> imu_recorder,
                     std::size_t queue_capacity,
                     OutputDispatcherCallbacks callbacks);
    ~OutputDispatcher();

    OutputDispatcher(const OutputDispatcher&) = delete;
    OutputDispatcher& operator=(const OutputDispatcher&) = delete;

    void reset() noexcept;
    void start();
    bool enqueue(ProcessedFrame snapshot);
    bool try_record_imu_sample(
        const domain::ImuKinematicSample& sample) noexcept;
    void cancel() noexcept;
    void cancel_imu_recorder() noexcept;
    void close_queue() noexcept;
    void join() noexcept;
    std::string stop_sink();
    std::uint64_t dropped_count() const;
    RuntimeOutputDiagnostics diagnostics() const;

private:
    void report_fault(std::string message) noexcept;
    std::unique_ptr<IImuRecorder> imu_recorder_;
    std::vector<std::shared_ptr<void>> frame_output_owners_;
    std::vector<std::unique_ptr<OutputFrameWorker>> frame_workers_;
    OutputDispatcherCallbacks callbacks_;
    bool sink_stopped_ = false;
};

}  // namespace mosas::runtime

#endif  // MOSAS_RUNTIME_OUTPUT_DISPATCHER_HPP
