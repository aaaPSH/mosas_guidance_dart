#include "output_dispatcher.hpp"

#include <atomic>
#include <exception>
#include <mutex>
#include <stdexcept>
#include <utility>

namespace mosas::runtime {
namespace {

struct QueuedProcessedFrame {
    ProcessedFrame frame;
    std::chrono::steady_clock::time_point enqueued_at{};
};

enum class FrameOutputKind {
    video,
    telemetry,
    visualization,
};

struct FrameOutputHandlers {
    std::function<bool()> start;
    std::function<void()> reset;
    std::function<bool(const ProcessedFrame&)> process;
    std::function<void()> cancel;
    std::function<void()> stop;
    std::function<std::string()> last_error;
    std::function<bool()> take_rearm_request;
};

}  // namespace

class OutputFrameWorker final {
public:
    OutputFrameWorker(FrameOutputKind kind, std::size_t queue_capacity,
                      bool primary, FrameOutputHandlers handlers,
                      OutputDispatcherCallbacks callbacks)
        : kind_(kind), queue_(queue_capacity), primary_(primary),
          handlers_(std::move(handlers)), callbacks_(std::move(callbacks)) {}

    OutputFrameWorker(const OutputFrameWorker&) = delete;
    OutputFrameWorker& operator=(const OutputFrameWorker&) = delete;

    void reset() noexcept {
        queue_.reset();
        accepted_count_.store(0, std::memory_order_relaxed);
        failure_count_.store(0, std::memory_order_relaxed);
        healthy_.store(true, std::memory_order_release);
        {
            std::lock_guard<std::mutex> lock(error_mutex_);
            last_error_.clear();
        }
        sink_stopped_ = false;
        try {
            if (handlers_.reset) {
                handlers_.reset();
            }
        } catch (...) {
            set_failure("failed to reset frame output sink", false);
        }
    }

    bool start() {
        if (worker_.joinable()) {
            throw std::logic_error("frame output worker is already running");
        }
        if (handlers_.start && !handlers_.start()) {
            set_failure(read_sink_error(), false);
            return false;
        }
        worker_ = std::thread(&OutputFrameWorker::worker_loop, this);
        return true;
    }

    bool enqueue(ProcessedFrame frame) {
        QueuedProcessedFrame queued;
        queued.frame = std::move(frame);
        queued.enqueued_at = std::chrono::steady_clock::now();
        if (!queue_.push(std::move(queued))) {
            return false;
        }
        accepted_count_.fetch_add(1, std::memory_order_relaxed);
        return true;
    }

    void cancel() noexcept {
        try {
            if (handlers_.cancel) {
                handlers_.cancel();
            }
        } catch (...) {
        }
    }

    void close_queue() noexcept { queue_.close(); }

    void join() noexcept {
        if (worker_.joinable()) {
            worker_.join();
        }
    }

    std::string stop_sink() noexcept {
        if (sink_stopped_) {
            return diagnostics().last_error;
        }
        sink_stopped_ = true;
        try {
            if (handlers_.stop) {
                handlers_.stop();
            }
        } catch (const std::exception& exception) {
            set_failure(exception.what(), false);
        } catch (...) {
            set_failure("unknown exception while stopping frame output", false);
        }
        const std::string sink_error = read_sink_error();
        if (!sink_error.empty()) {
            set_failure(sink_error, false);
        }
        return diagnostics().last_error;
    }

    SinkDiagnostics diagnostics() const {
        SinkDiagnostics result;
        result.enabled = true;
        result.healthy = healthy_.load(std::memory_order_acquire);
        result.accepted_count =
            accepted_count_.load(std::memory_order_relaxed);
        result.dropped_count = queue_.dropped_count();
        result.failure_count =
            failure_count_.load(std::memory_order_relaxed);
        result.queue_depth = queue_.size();
        result.queue_capacity = queue_.capacity();
        {
            std::lock_guard<std::mutex> lock(error_mutex_);
            result.last_error = last_error_;
        }
        return result;
    }

    FrameOutputKind kind() const noexcept { return kind_; }

private:
    void set_failure(const std::string& error, bool report) noexcept {
        if (healthy_.exchange(false, std::memory_order_acq_rel)) {
            failure_count_.fetch_add(1, std::memory_order_relaxed);
        }
        try {
            std::lock_guard<std::mutex> lock(error_mutex_);
            if (last_error_.empty()) {
                last_error_ = error.empty() ? "frame output failed" : error;
            }
        } catch (...) {
        }
        if (report && callbacks_.report_fault) {
            try {
                callbacks_.report_fault("frame output failed");
            } catch (...) {
            }
        }
    }

    std::string read_sink_error() const {
        if (!handlers_.last_error) {
            return {};
        }
        try {
            return handlers_.last_error();
        } catch (...) {
            return "failed to read frame output error";
        }
    }

    void record_timing(OutputDispatchTimingStage stage,
                       std::chrono::steady_clock::duration duration) const {
        if (callbacks_.record_timing) {
            callbacks_.record_timing(stage, duration);
        }
    }

    /**
     * @brief 消费本 sink 的帧队列并执行阻塞式发布/记录操作。
     * @note 队列和 worker 按 sink 独立；代际不匹配的帧被忽略，处理失败会上报运行时故障。
     */
    void worker_loop() noexcept {
        try {
            QueuedProcessedFrame queued{};
            while (true) {
                const auto queue_wait_started =
                    std::chrono::steady_clock::now();
                if (!queue_.pop(&queued)) {
                    break;
                }
                const auto dequeued_at = std::chrono::steady_clock::now();
                record_timing(OutputDispatchTimingStage::queue_wait,
                              dequeued_at - queue_wait_started);
                record_timing(OutputDispatchTimingStage::queue_delay,
                              dequeued_at - queued.enqueued_at);
                if (callbacks_.stop_requested && callbacks_.stop_requested()) {
                    break;
                }
                if (callbacks_.active_generation &&
                    queued.frame.flight_state.launch.generation !=
                        callbacks_.active_generation()) {
                    queued = QueuedProcessedFrame{};
                    continue;
                }

                if (callbacks_.processing_fps) {
                    queued.frame.overlay.processing_fps =
                        callbacks_.processing_fps();
                }
                const auto output_started = std::chrono::steady_clock::now();
                if (handlers_.process && !handlers_.process(queued.frame)) {
                    set_failure(read_sink_error(), true);
                    return;
                }
                if (handlers_.take_rearm_request &&
                    handlers_.take_rearm_request() && callbacks_.request_rearm) {
                    callbacks_.request_rearm();
                }
                record_timing(OutputDispatchTimingStage::publish,
                              std::chrono::steady_clock::now() - output_started);
                if (primary_ && callbacks_.record_published_frame) {
                    callbacks_.record_published_frame();
                }
                queued = QueuedProcessedFrame{};
            }
        } catch (const std::exception& exception) {
            set_failure(exception.what(), true);
        } catch (...) {
            set_failure("unknown exception in frame output worker", true);
        }
    }

    const FrameOutputKind kind_;
    LatestFrameQueue<QueuedProcessedFrame> queue_;
    const bool primary_;
    FrameOutputHandlers handlers_;
    OutputDispatcherCallbacks callbacks_;
    std::thread worker_;
    std::atomic<std::uint64_t> accepted_count_{0};
    std::atomic<std::uint64_t> failure_count_{0};
    std::atomic<bool> healthy_{true};
    mutable std::mutex error_mutex_;
    std::string last_error_;
    bool sink_stopped_ = false;
};

namespace {

template <typename Sink>
std::shared_ptr<Sink> take_shared(std::unique_ptr<Sink> sink) {
    return std::shared_ptr<Sink>(std::move(sink));
}

}  // namespace

OutputDispatcher::OutputDispatcher(
    std::unique_ptr<IVideoRecorder> video_recorder,
    std::unique_ptr<ITelemetryRecorder> telemetry_recorder,
    std::unique_ptr<IVisualizationSink> visualization_sink,
    std::unique_ptr<IImuRecorder> imu_recorder, std::size_t queue_capacity,
    OutputDispatcherCallbacks callbacks)
    : imu_recorder_(std::move(imu_recorder)), callbacks_(std::move(callbacks)) {
    bool primary_assigned = false;
    if (video_recorder != nullptr) {
        auto owned = take_shared(std::move(video_recorder));
        IVideoRecorder* const sink = owned.get();
        frame_output_owners_.push_back(owned);
        FrameOutputHandlers handlers;
        handlers.start = [sink] { return sink->start(); };
        handlers.reset = [sink] { sink->reset(); };
        handlers.process = [sink](const ProcessedFrame& frame) {
            return sink->record(frame);
        };
        handlers.take_rearm_request = [sink] {
            return sink->take_rearm_request();
        };
        handlers.cancel = [sink] { sink->cancel(); };
        handlers.stop = [sink] { sink->stop(); };
        handlers.last_error = [sink] { return sink->last_error(); };
        frame_workers_.push_back(std::make_unique<OutputFrameWorker>(
            FrameOutputKind::video, queue_capacity, !primary_assigned,
            std::move(handlers), callbacks_));
        primary_assigned = true;
    }
    if (telemetry_recorder != nullptr) {
        auto owned = take_shared(std::move(telemetry_recorder));
        ITelemetryRecorder* const sink = owned.get();
        frame_output_owners_.push_back(owned);
        FrameOutputHandlers handlers;
        handlers.start = [sink] { return sink->start(); };
        handlers.reset = [sink] { sink->reset(); };
        handlers.process = [sink](const ProcessedFrame& frame) {
            return sink->record(frame);
        };
        handlers.take_rearm_request = [sink] {
            return sink->take_rearm_request();
        };
        handlers.cancel = [sink] { sink->cancel(); };
        handlers.stop = [sink] { sink->stop(); };
        handlers.last_error = [sink] { return sink->last_error(); };
        frame_workers_.push_back(std::make_unique<OutputFrameWorker>(
            FrameOutputKind::telemetry, queue_capacity, !primary_assigned,
            std::move(handlers), callbacks_));
        primary_assigned = true;
    }
    if (visualization_sink != nullptr) {
        auto owned = take_shared(std::move(visualization_sink));
        IVisualizationSink* const sink = owned.get();
        frame_output_owners_.push_back(owned);
        FrameOutputHandlers handlers;
        handlers.start = [sink] { return sink->start(); };
        handlers.reset = [sink] { sink->reset(); };
        handlers.process = [sink](const ProcessedFrame& frame) {
            return sink->publish(frame);
        };
        handlers.cancel = [sink] { sink->cancel(); };
        handlers.stop = [sink] { sink->stop(); };
        handlers.last_error = [sink] { return sink->last_error(); };
        frame_workers_.push_back(std::make_unique<OutputFrameWorker>(
            FrameOutputKind::visualization, queue_capacity, !primary_assigned,
            std::move(handlers), callbacks_));
    }
}

OutputDispatcher::~OutputDispatcher() {
    if (!sink_stopped_) {
        cancel();
    }
    close_queue();
    join();
    try {
        stop_sink();
    } catch (...) {
    }
}

void OutputDispatcher::reset() noexcept {
    sink_stopped_ = false;
    for (const auto& worker : frame_workers_) {
        worker->reset();
    }
}

void OutputDispatcher::start() {
    if (imu_recorder_ != nullptr) {
        // Preserve the existing first-sample fault behavior on recorder startup
        // failure; ImuPipeline observes the failed non-blocking submission.
        imu_recorder_->start();
    }
    for (const auto& worker : frame_workers_) {
        if (!worker->start()) {
            report_fault("frame output failed");
            throw std::runtime_error("frame output failed");
        }
    }
}

bool OutputDispatcher::enqueue(ProcessedFrame snapshot) {
    bool accepted = true;
    for (std::size_t index = 0; index < frame_workers_.size(); ++index) {
        ProcessedFrame frame = index + 1 == frame_workers_.size()
                                   ? std::move(snapshot)
                                   : snapshot;
        accepted = frame_workers_[index]->enqueue(std::move(frame)) && accepted;
    }
    return accepted;
}

bool OutputDispatcher::try_record_imu_sample(
    const domain::ImuKinematicSample& sample) noexcept {
    return imu_recorder_ == nullptr || imu_recorder_->try_record(sample);
}

void OutputDispatcher::cancel() noexcept {
    for (const auto& worker : frame_workers_) {
        worker->cancel();
    }
}

void OutputDispatcher::cancel_imu_recorder() noexcept {
    if (imu_recorder_ != nullptr) {
        imu_recorder_->cancel();
    }
}

void OutputDispatcher::close_queue() noexcept {
    for (const auto& worker : frame_workers_) {
        worker->close_queue();
    }
}

void OutputDispatcher::join() noexcept {
    for (const auto& worker : frame_workers_) {
        worker->join();
    }
}

std::string OutputDispatcher::stop_sink() {
    if (sink_stopped_) {
        return {};
    }
    sink_stopped_ = true;

    std::string first_error;
    for (const auto& worker : frame_workers_) {
        const std::string error = worker->stop_sink();
        if (first_error.empty() && !error.empty()) {
            first_error = error;
        }
    }
    if (imu_recorder_ != nullptr) {
        imu_recorder_->stop();
        const std::string error = imu_recorder_->diagnostics().last_error;
        if (first_error.empty() && !error.empty()) {
            first_error = error;
        }
    }
    return first_error;
}

std::uint64_t OutputDispatcher::dropped_count() const {
    std::uint64_t total = 0;
    for (const auto& worker : frame_workers_) {
        total += worker->diagnostics().dropped_count;
    }
    return total;
}

RuntimeOutputDiagnostics OutputDispatcher::diagnostics() const {
    RuntimeOutputDiagnostics result;
    for (const auto& worker : frame_workers_) {
        const SinkDiagnostics sink_diagnostics = worker->diagnostics();
        switch (worker->kind()) {
            case FrameOutputKind::video:
                result.video = sink_diagnostics;
                break;
            case FrameOutputKind::telemetry:
                result.telemetry = sink_diagnostics;
                break;
            case FrameOutputKind::visualization:
                result.visualization = sink_diagnostics;
                break;
        }
    }
    if (imu_recorder_ != nullptr) {
        result.imu_recording = imu_recorder_->diagnostics();
    }
    return result;
}

void OutputDispatcher::report_fault(std::string message) noexcept {
    if (!callbacks_.report_fault) {
        return;
    }
    try {
        callbacks_.report_fault(std::move(message));
    } catch (...) {
    }
}

}  // namespace mosas::runtime
