#include <mosas/wireless/rtsp_frame_sink.hpp>

#include <utility>

#include <mosas/vision/vision_visualizer.hpp>

#include <cmath>
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
    if (cancel_requested_.load()) {
        return true;
    }
    if (!config_.enable_wireless_stream && !config_.enable_recording) {
        return true;
    }
    if (config_.enable_recording && config_.recording_path.empty()) {
        last_error_ = "recording path must not be empty";
        return false;
    }
    if (config_.enable_recording &&
        (!std::isfinite(config_.recording_fps) ||
         config_.recording_fps <= 0.0)) {
        last_error_ = "recording FPS must be positive";
        return false;
    }
    if (frame.image.empty() || frame.image.type() != CV_8UC3) {
        last_error_ = "camera frame must be a non-empty CV_8UC3 image";
        return false;
    }

    try {
        if (config_.draw_visualization) {
            frame.image.copyTo(annotated_rgb_frame_);
            if (overlay != nullptr) {
                VisionVisualizer::draw_result(
                    annotated_rgb_frame_, vision_result,
                    line_of_sight_reference_point_);
                VisionVisualizer::draw_guidance_overlay(annotated_rgb_frame_,
                                                        *overlay);
            } else {
                VisionVisualizer::draw_result(
                    annotated_rgb_frame_, vision_result,
                    line_of_sight_reference_point_);
            }
            if (annotated_rgb_frame_.cols != config_.stream.width ||
                annotated_rgb_frame_.rows != config_.stream.height) {
                cv::resize(
                    annotated_rgb_frame_, resized_rgb_frame_,
                    cv::Size(config_.stream.width, config_.stream.height),
                    0.0, 0.0, cv::INTER_LINEAR);
                cv::cvtColor(resized_rgb_frame_, bgr_frame_,
                             cv::COLOR_RGB2BGR);
            } else {
                cv::cvtColor(annotated_rgb_frame_, bgr_frame_,
                             cv::COLOR_RGB2BGR);
            }
        } else if (frame.image.cols != config_.stream.width ||
                   frame.image.rows != config_.stream.height) {
            cv::resize(frame.image, resized_rgb_frame_,
                       cv::Size(config_.stream.width, config_.stream.height),
                       0.0, 0.0, cv::INTER_LINEAR);
            cv::cvtColor(resized_rgb_frame_, bgr_frame_, cv::COLOR_RGB2BGR);
        } else {
            cv::cvtColor(frame.image, bgr_frame_, cv::COLOR_RGB2BGR);
        }

        if (config_.enable_wireless_stream) {
            RtspStreamer* streamer = nullptr;
            {
                std::lock_guard<std::mutex> lock(streamer_mutex_);
                if (streamer_ == nullptr) {
                    streamer_ = std::make_unique<RtspStreamer>(config_.stream);
                }
                streamer = streamer_.get();
            }
            if (cancel_requested_.load()) {
                return true;
            }
            if (!streamer->running() && !streamer->start(&last_error_)) {
                return false;
            }
            if (cancel_requested_.load()) {
                return true;
            }
            if (!streamer->send_bgr(bgr_frame_, frame.timestamp_ns,
                                    &last_error_)) {
                if (cancel_requested_.load()) {
                    return true;
                }
                return false;
            }
        }
        if (config_.enable_recording) {
            if (!recording_writer_.isOpened() &&
                !recording_writer_.open(
                    config_.recording_path,
                    cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                    config_.recording_fps, bgr_frame_.size(), true)) {
                last_error_ = "open recording writer failed: " +
                              config_.recording_path;
                return false;
            }
            recording_writer_.write(bgr_frame_);
            // VideoWriter::write() 没有返回值，只能检查 writer 是否仍保持打开；
            // 后端抛出的 OpenCV 异常会由外层捕获并报告为发布失败。
            if (!recording_writer_.isOpened()) {
                last_error_ = "write recording frame failed";
                return false;
            }
        }
        return true;
    } catch (const cv::Exception& exception) {
        last_error_ = exception.what();
        return false;
    }
}

void RtspFrameSink::stop() noexcept {
    cancel_requested_.store(true);
    if (recording_writer_.isOpened()) {
        recording_writer_.release();
    }
    std::unique_ptr<RtspStreamer> streamer;
    {
        std::lock_guard<std::mutex> lock(streamer_mutex_);
        if (streamer_ != nullptr) {
            streamer_->cancel();
        }
        streamer = std::move(streamer_);
    }
    if (streamer != nullptr) {
        streamer->stop();
    }
}

void RtspFrameSink::cancel() noexcept {
    cancel_requested_.store(true);
    std::lock_guard<std::mutex> lock(streamer_mutex_);
    if (streamer_ != nullptr) {
        streamer_->cancel();
    }
}

void RtspFrameSink::reset() noexcept {
    cancel_requested_.store(false);
    std::lock_guard<std::mutex> lock(streamer_mutex_);
    if (streamer_ != nullptr) {
        streamer_->reset();
    }
    last_error_.clear();
}

bool RtspFrameSink::running() const noexcept {
    std::lock_guard<std::mutex> lock(streamer_mutex_);
    const bool streaming = config_.enable_wireless_stream &&
                           streamer_ != nullptr && streamer_->running();
    const bool recording = config_.enable_recording &&
                           recording_writer_.isOpened();
    return streaming || recording;
}

std::string RtspFrameSink::last_error() const { return last_error_; }

}  // namespace mosas::wireless
