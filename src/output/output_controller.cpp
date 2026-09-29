#include <mosas/output/output_controller.hpp>

#include "recording_paths.hpp"

#include <mosas/vision/vision_visualizer.hpp>

#include <filesystem>
#include <iostream>
#include <utility>

#include <opencv2/imgproc.hpp>

namespace mosas::output {
namespace {

std::filesystem::path recording_path_for_run(
    const std::string& configured_recording_path) {
    std::filesystem::path configured_path(configured_recording_path);
    if (configured_path.empty()) {
        configured_path = "recordings/mosas_guidance.avi";
    } else if (configured_path.filename().empty()) {
        configured_path /= "mosas_guidance.avi";
    }

    std::string stem = configured_path.stem().string();
    if (stem.empty()) {
        stem = "recording";
    }
    const std::filesystem::path run_directory =
        configured_path.parent_path() /
        (stem + "_" + detail::make_session_id());
    return run_directory / configured_path.filename();
}

}  // namespace

OutputController::OutputController(OutputControllerConfig config)
    : config_(std::move(config)) {
    if (!validate_config()) {
        return;
    }
    config_.output.recording_path =
        recording_path_for_run(config_.output.recording_path).string();
    imu_csv_recorder_ = std::make_unique<ImuCsvRecorder>(
        config_.output.recording_path);
    if (config_.output.mode == OutputMode::rtsp) {
        return;
    }

    LocalRecorderConfig recorder_config;
    recorder_config.recording_path = config_.output.recording_path;
    recorder_config.fps = config_.camera_fps;
    recorder_config.width = config_.camera_width;
    recorder_config.height = config_.camera_height;
    recorder_config.mode = config_.output.mode == OutputMode::debug
                               ? LocalRecordingMode::continuous
                               : LocalRecordingMode::launch_window;
    recorder_config.window_duration_ns =
        static_cast<std::int64_t>(config_.output.match_window_duration_ms) *
        1'000'000;
    recorder_config.max_buffer_bytes = kMatchRecordingMemoryLimitBytes;
    local_recorder_ = std::make_unique<LocalRecorder>(
        std::move(recorder_config));
}

OutputController::~OutputController() { stop(); }

bool OutputController::publish(
    const runtime::CameraFrame& frame, const VisionResult& vision_result,
    const runtime::ImuStateSnapshot& imu_state,
    const PngGuidanceOutput& guidance) {
    return publish_impl(frame, vision_result, imu_state, guidance, nullptr);
}

bool OutputController::publish(
    const runtime::CameraFrame& frame, const VisionResult& vision_result,
    const runtime::ImuStateSnapshot& imu_state,
    const PngGuidanceOutput& guidance, const VisionOverlayData& overlay) {
    return publish_impl(frame, vision_result, imu_state, guidance, &overlay);
}

bool OutputController::try_record_imu_sample(
    const runtime::ImuAngularVelocitySample& sample) noexcept {
    if (imu_csv_recorder_ == nullptr || !imu_csv_recorder_->try_record(sample)) {
        if (imu_csv_recorder_ != nullptr) {
            try {
                const std::string error = imu_csv_recorder_->last_error();
                if (!error.empty()) {
                    std::clog << "IMU CSV 记录失败: " << error << '\n';
                }
            } catch (...) {
            }
        }
        return false;
    }
    return true;
}

bool OutputController::publish_impl(
    const runtime::CameraFrame& frame, const VisionResult& vision_result,
    const runtime::ImuStateSnapshot& imu_state,
    const PngGuidanceOutput& guidance, const VisionOverlayData* overlay) {
    if (cancelled_.load()) {
        return true;
    }
    if (!last_error_.empty()) {
        return false;
    }
    if (frame.image.empty() || frame.image.type() != CV_8UC3) {
        last_error_ = "camera frame must be a non-empty CV_8UC3 image";
        return false;
    }

    const cv::Mat* effective_image = &frame.image;
    if (config_.output.draw_visualization) {
        frame.image.copyTo(rendered_frame_);
        VisionVisualizer::draw_result(
            rendered_frame_, vision_result,
            config_.line_of_sight_reference_point);
        if (overlay != nullptr) {
            VisionVisualizer::draw_guidance_overlay(rendered_frame_, *overlay);
        }
        effective_image = &rendered_frame_;
    }

    if (config_.output.mode == OutputMode::rtsp) {
        return publish_rtsp(frame, *effective_image);
    }
    if (local_recorder_ == nullptr) {
        last_error_ = "local recording output is not initialized";
        return false;
    }

    runtime::CameraFrame rendered_frame = frame;
    rendered_frame.image = *effective_image;
    const VisionOverlayData empty_overlay{};
    return local_recorder_->publish(
        rendered_frame, vision_result, imu_state, guidance,
        overlay == nullptr ? empty_overlay : *overlay);
}

bool OutputController::publish_rtsp(const runtime::CameraFrame& frame,
                                    const cv::Mat& rendered_frame) {
#if defined(MOSAS_OUTPUT_HAS_WIRELESS)
    wireless::RtspStreamer* streamer = nullptr;
    {
        std::lock_guard<std::mutex> lock(rtsp_mutex_);
        if (rtsp_streamer_ == nullptr) {
            rtsp_streamer_ =
                std::make_unique<wireless::RtspStreamer>(config_.wireless);
        }
        streamer = rtsp_streamer_.get();
    }
    if (cancelled_.load()) {
        return true;
    }
    if (!streamer->running() && !streamer->start(&last_error_)) {
        return false;
    }
    if (rendered_frame.cols != config_.wireless.width ||
        rendered_frame.rows != config_.wireless.height) {
        cv::resize(rendered_frame, rtsp_frame_,
                   cv::Size(config_.wireless.width, config_.wireless.height),
                   0.0, 0.0, cv::INTER_LINEAR);
    } else {
        rendered_frame.copyTo(rtsp_frame_);
    }
    if (!streamer->send_bgr(rtsp_frame_, frame.timestamp_ns, &last_error_)) {
        return false;
    }
    return true;
#else
    (void)frame;
    (void)rendered_frame;
    last_error_ = "RTSP mode is unavailable because wireless support is disabled";
    return false;
#endif
}

bool OutputController::validate_config() {
    if (config_.camera_width <= 0 || config_.camera_height <= 0 ||
        config_.camera_fps <= 0) {
        last_error_ = "camera output size and FPS must be positive";
        return false;
    }

    if (config_.output.mode == OutputMode::rtsp) {
#if !defined(MOSAS_OUTPUT_HAS_WIRELESS)
        last_error_ = "RTSP mode is unavailable because wireless support is disabled";
        return false;
#else
        return true;
#endif
    }

    if (config_.output.recording_path.empty()) {
        last_error_ = "recording path must not be empty";
        return false;
    }
    if (config_.output.mode == OutputMode::match &&
        !match_window_fits_memory(
            config_.camera_width, config_.camera_height, config_.camera_fps,
            config_.output.match_window_duration_ms)) {
        last_error_ = "match window exceeds the configured image-memory limit";
        return false;
    }
    return true;
}

void OutputController::cancel() noexcept {
    cancelled_.store(true);
#if defined(MOSAS_OUTPUT_HAS_WIRELESS)
    std::lock_guard<std::mutex> lock(rtsp_mutex_);
    if (rtsp_streamer_ != nullptr) {
        rtsp_streamer_->cancel();
    }
#endif
}

void OutputController::reset() noexcept {
    cancelled_.store(false);
    last_error_.clear();
    if (!validate_config()) {
        return;
    }
    if (imu_csv_recorder_ != nullptr) {
        imu_csv_recorder_->start();
    }
    if (local_recorder_ != nullptr) {
        local_recorder_->reset();
    }
#if defined(MOSAS_OUTPUT_HAS_WIRELESS)
    std::lock_guard<std::mutex> lock(rtsp_mutex_);
    if (rtsp_streamer_ != nullptr) {
        rtsp_streamer_->reset();
    }
#endif
}

void OutputController::stop() noexcept {
    cancelled_.store(true);
    if (imu_csv_recorder_ != nullptr) {
        imu_csv_recorder_->stop();
    }
#if defined(MOSAS_OUTPUT_HAS_WIRELESS)
    std::unique_ptr<wireless::RtspStreamer> streamer;
    {
        std::lock_guard<std::mutex> lock(rtsp_mutex_);
        streamer = std::move(rtsp_streamer_);
    }
    if (streamer != nullptr) {
        streamer->stop();
    }
#endif
    if (local_recorder_ != nullptr) {
        local_recorder_->stop();
    }
}

bool OutputController::take_rearm_request() noexcept {
    return local_recorder_ != nullptr &&
           local_recorder_->take_rearm_request();
}

std::string OutputController::last_error() const {
    if (imu_csv_recorder_ != nullptr) {
        const std::string error = imu_csv_recorder_->last_error();
        if (!error.empty()) {
            return error;
        }
    }
    if (local_recorder_ != nullptr &&
        !local_recorder_->last_error().empty()) {
        return local_recorder_->last_error();
    }
    return last_error_;
}

}  // namespace mosas::output
