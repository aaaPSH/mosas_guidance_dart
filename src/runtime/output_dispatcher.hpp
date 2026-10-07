#ifndef MOSAS_RUNTIME_OUTPUT_DISPATCHER_HPP
#define MOSAS_RUNTIME_OUTPUT_DISPATCHER_HPP

#include <mosas/runtime/latest_frame_queue.hpp>
#include <mosas/runtime/runtime_interfaces.hpp>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <thread>

namespace mosas::runtime {

struct ProcessedFrame {
    CameraFrame frame;
    domain::VisionDetection vision_detection{};
    domain::FlightStateSnapshot flight_state{};
    domain::GuidanceOutput guidance{};
    domain::GuidanceOverlay overlay{};
    std::chrono::steady_clock::time_point enqueued_at{};
};

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

class OutputDispatcher final {
public:
    OutputDispatcher(std::unique_ptr<FrameSink> frame_sink,
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
    void close_queue() noexcept;
    void join() noexcept;
    std::string stop_sink();
    std::uint64_t dropped_count() const;

private:
    void worker_loop() noexcept;
    void report_fault(std::string message) noexcept;

    std::unique_ptr<FrameSink> frame_sink_;
    LatestFrameQueue<ProcessedFrame> queue_;
    OutputDispatcherCallbacks callbacks_;
    std::thread worker_;
    bool sink_stopped_ = false;
};

}  // namespace mosas::runtime

#endif  // MOSAS_RUNTIME_OUTPUT_DISPATCHER_HPP
