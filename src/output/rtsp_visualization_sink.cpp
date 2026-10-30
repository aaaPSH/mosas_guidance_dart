#include <mosas/output/rtsp_visualization_sink.hpp>

#include <mosas/output/output_mapping.hpp>
#include <mosas/vision/opencv_image_frame.hpp>
#include <mosas/vision/vision_visualizer.hpp>

#include <exception>
#include <utility>

#include <opencv2/imgproc.hpp>

namespace mosas::output {

RtspVisualizationSink::RtspVisualizationSink(
    OutputConfig output_config, wireless::RtspStreamConfig stream_config)
    : output_config_(std::move(output_config)),
      stream_config_(std::move(stream_config)) {}

RtspVisualizationSink::~RtspVisualizationSink() { stop(); }

bool RtspVisualizationSink::start() {
    cancelled_.store(false, std::memory_order_release);
    return true;
}

void RtspVisualizationSink::reset() noexcept {
    cancelled_.store(false, std::memory_order_release);
    {
        std::lock_guard<std::mutex> lock(error_mutex_);
        last_error_.clear();
    }
    std::lock_guard<std::mutex> lock(stream_mutex_);
    if (streamer_ != nullptr) {
        streamer_->reset();
    }
}

bool RtspVisualizationSink::publish(const runtime::ProcessedFrame& frame) {
    if (cancelled_.load(std::memory_order_acquire)) {
        return true;
    }
    try {
        cv::Mat source = vision::make_bgr8_mat_view(frame.frame.image.view());
        if (source.empty()) {
            return fail("camera frame must be a valid BGR8 image");
        }

        const cv::Mat* effective_frame = &source;
        if (output_config_.draw_visualization) {
            source.copyTo(rendered_frame_);
            ::VisionVisualizer::draw_result(
                rendered_frame_,
                detail::make_legacy_vision_result(frame.vision_detection));
            ::VisionVisualizer::draw_guidance_overlay(
                rendered_frame_, detail::make_legacy_overlay(
                                     frame.overlay, frame.flight_state));
            effective_frame = &rendered_frame_;
        }

        wireless::RtspStreamer* streamer = get_streamer();
        if (streamer == nullptr ||
            cancelled_.load(std::memory_order_acquire)) {
            return true;
        }
        std::string error;
        if (!streamer->running() && !streamer->start(&error)) {
            return fail(error.empty() ? "failed to start RTSP stream"
                                      : std::move(error));
        }
        if (effective_frame->cols != stream_config_.width ||
            effective_frame->rows != stream_config_.height) {
            cv::resize(*effective_frame, rtsp_frame_,
                       cv::Size(stream_config_.width, stream_config_.height),
                       0.0, 0.0, cv::INTER_LINEAR);
        } else {
            effective_frame->copyTo(rtsp_frame_);
        }
        if (!streamer->send_bgr(rtsp_frame_, frame.frame.timestamp_ns, &error)) {
            return fail(error.empty() ? "failed to send RTSP frame"
                                      : std::move(error));
        }
        return true;
    } catch (const std::exception& exception) {
        return fail(exception.what());
    } catch (...) {
        return fail("unknown RTSP visualization failure");
    }
}

void RtspVisualizationSink::cancel() noexcept {
    cancelled_.store(true, std::memory_order_release);
    wireless::RtspStreamer* streamer =
        streamer_ptr_.load(std::memory_order_acquire);
    if (streamer != nullptr) {
        streamer->cancel();
    }
}

void RtspVisualizationSink::stop() noexcept {
    cancelled_.store(true, std::memory_order_release);
    std::unique_ptr<wireless::RtspStreamer> streamer;
    {
        std::lock_guard<std::mutex> lock(stream_mutex_);
        streamer_ptr_.store(nullptr, std::memory_order_release);
        streamer = std::move(streamer_);
    }
    if (streamer != nullptr) {
        streamer->stop();
    }
}

std::string RtspVisualizationSink::last_error() const {
    std::lock_guard<std::mutex> lock(error_mutex_);
    return last_error_;
}

wireless::RtspStreamer* RtspVisualizationSink::get_streamer() {
    std::lock_guard<std::mutex> lock(stream_mutex_);
    if (streamer_ == nullptr) {
        streamer_ = std::make_unique<wireless::RtspStreamer>(stream_config_);
        streamer_ptr_.store(streamer_.get(), std::memory_order_release);
    }
    return streamer_.get();
}

bool RtspVisualizationSink::fail(std::string message) noexcept {
    try {
        std::lock_guard<std::mutex> lock(error_mutex_);
        if (last_error_.empty()) {
            last_error_ = std::move(message);
        }
    } catch (...) {
    }
    return false;
}

}  // namespace mosas::output
