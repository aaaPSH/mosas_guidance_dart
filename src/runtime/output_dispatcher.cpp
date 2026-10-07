#include "output_dispatcher.hpp"

#include <exception>
#include <stdexcept>
#include <utility>

namespace mosas::runtime {

OutputDispatcher::OutputDispatcher(
    std::unique_ptr<FrameSink> frame_sink, std::size_t queue_capacity,
    OutputDispatcherCallbacks callbacks)
    : frame_sink_(std::move(frame_sink)),
      queue_(queue_capacity),
      callbacks_(std::move(callbacks)) {}

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
    if (frame_sink_ != nullptr) {
        frame_sink_->reset();
    }
    queue_.reset();
}

void OutputDispatcher::start() {
    if (worker_.joinable()) {
        throw std::logic_error("output dispatcher is already running");
    }
    worker_ = std::thread(&OutputDispatcher::worker_loop, this);
}

bool OutputDispatcher::enqueue(ProcessedFrame snapshot) {
    if (frame_sink_ == nullptr) {
        return true;
    }
    snapshot.enqueued_at = std::chrono::steady_clock::now();
    return queue_.push(std::move(snapshot));
}

bool OutputDispatcher::try_record_imu_sample(
    const domain::ImuKinematicSample& sample) noexcept {
    return frame_sink_ == nullptr ||
           frame_sink_->try_record_imu_sample(sample);
}

void OutputDispatcher::cancel() noexcept {
    if (frame_sink_ != nullptr) {
        frame_sink_->cancel();
    }
}

void OutputDispatcher::close_queue() noexcept { queue_.close(); }

void OutputDispatcher::join() noexcept {
    if (worker_.joinable()) {
        worker_.join();
    }
}

std::string OutputDispatcher::stop_sink() {
    if (frame_sink_ == nullptr) {
        return {};
    }
    if (!sink_stopped_) {
        frame_sink_->stop();
        sink_stopped_ = true;
    }
    return frame_sink_->last_error();
}

std::uint64_t OutputDispatcher::dropped_count() const {
    return queue_.dropped_count();
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

void OutputDispatcher::worker_loop() noexcept {
    try {
        ProcessedFrame result{};
        while (true) {
            const auto queue_wait_started = std::chrono::steady_clock::now();
            if (!queue_.pop(&result)) {
                break;
            }
            const auto dequeued_at = std::chrono::steady_clock::now();
            if (callbacks_.record_timing) {
                callbacks_.record_timing(OutputDispatchTimingStage::queue_wait,
                                         dequeued_at - queue_wait_started);
                callbacks_.record_timing(
                    OutputDispatchTimingStage::queue_delay,
                    dequeued_at - result.enqueued_at);
            }
            if (callbacks_.stop_requested && callbacks_.stop_requested()) {
                break;
            }
            if (callbacks_.active_generation &&
                result.flight_state.launch.generation !=
                    callbacks_.active_generation()) {
                result = ProcessedFrame{};
                continue;
            }
            if (frame_sink_ != nullptr) {
                domain::GuidanceOverlay overlay = result.overlay;
                if (callbacks_.processing_fps) {
                    overlay.processing_fps = callbacks_.processing_fps();
                }
                const auto output_started = std::chrono::steady_clock::now();
                if (!frame_sink_->publish(
                        result.frame, result.vision_detection,
                        result.flight_state, result.guidance, overlay)) {
                    report_fault("frame output failed");
                    return;
                }
                if (frame_sink_->take_rearm_request() &&
                    callbacks_.request_rearm) {
                    callbacks_.request_rearm();
                }
                if (callbacks_.record_timing) {
                    callbacks_.record_timing(
                        OutputDispatchTimingStage::publish,
                        std::chrono::steady_clock::now() - output_started);
                }
                if (callbacks_.record_published_frame) {
                    callbacks_.record_published_frame();
                }
            }
            result = ProcessedFrame{};
        }
    } catch (const std::exception& exception) {
        report_fault(exception.what());
    } catch (...) {
        report_fault("unknown exception in output worker");
    }
}

}  // namespace mosas::runtime
