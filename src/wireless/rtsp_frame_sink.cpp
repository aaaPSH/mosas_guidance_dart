#include <mosas/wireless/rtsp_frame_sink.hpp>

#include <utility>

#include <mosas/vision/vision_visualizer.hpp>

#include <opencv2/imgproc.hpp>

namespace mosas::wireless {

RtspFrameSink::RtspFrameSink(
    RtspFrameSinkConfig config, cv::Point2d line_of_sight_reference_point)
    : config_(std::move(config)),
      line_of_sight_reference_point_(line_of_sight_reference_point) {}

RtspFrameSink::~RtspFrameSink() { stop(); }

bool RtspFrameSink::publish(
    const mosas::runtime::CameraFrame& frame,
    const VisionResult& vision_result,
    const mosas::runtime::ImuStateSnapshot& imu_state,
    const PngGuidanceOutput& guidance) {
    (void)imu_state;
    (void)guidance;
    return publish_impl(frame, vision_result, nullptr);
}

bool RtspFrameSink::publish(
    const mosas::runtime::CameraFrame& frame,
    const VisionResult& vision_result,
    const mosas::runtime::ImuStateSnapshot& imu_state,
    const PngGuidanceOutput& guidance,
    const VisionOverlayData& overlay) {
    (void)imu_state;
    (void)guidance;
    return publish_impl(frame, vision_result, &overlay);
}

bool RtspFrameSink::publish_impl(
    const mosas::runtime::CameraFrame& frame,
    const VisionResult& vision_result,
    const VisionOverlayData* overlay) {
    if (!config_.enable_wireless_stream) {
        return true;
    }
    if (frame.image.empty() || frame.image.type() != CV_8UC3) {
        last_error_ = "camera frame must be a non-empty CV_8UC3 image";
        return false;
    }

    cv::Mat output;
    try {
        output = frame.image.clone();
        if (config_.draw_visualization) {
            if (overlay != nullptr) {
                VisionVisualizer::draw_result(
                    output, vision_result, line_of_sight_reference_point_);
                VisionVisualizer::draw_guidance_overlay(output, *overlay);
            } else {
                VisionVisualizer::draw_result(
                    output, vision_result, line_of_sight_reference_point_);
            }
        }

        if (output.cols != config_.stream.width ||
            output.rows != config_.stream.height) {
            cv::resize(output, output,
                       cv::Size(config_.stream.width, config_.stream.height),
                       0.0, 0.0, cv::INTER_LINEAR);
        }

        cv::Mat bgr_output;
        cv::cvtColor(output, bgr_output, cv::COLOR_RGB2BGR);
        if (streamer_ == nullptr) {
            streamer_ = std::make_unique<RtspStreamer>(config_.stream);
        }
        if (!streamer_->running() && !streamer_->start(&last_error_)) {
            return false;
        }
        return streamer_->send_bgr(bgr_output, frame.timestamp_ns,
                                   &last_error_);
    } catch (const cv::Exception& exception) {
        last_error_ = exception.what();
        return false;
    }
}

void RtspFrameSink::stop() noexcept {
    if (streamer_ != nullptr) {
        streamer_->stop();
    }
}

bool RtspFrameSink::running() const noexcept {
    return config_.enable_wireless_stream && streamer_ != nullptr &&
           streamer_->running();
}

std::string RtspFrameSink::last_error() const { return last_error_; }

}  // namespace mosas::wireless
