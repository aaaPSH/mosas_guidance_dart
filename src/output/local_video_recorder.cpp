#include <mosas/output/local_video_recorder.hpp>

#include <mosas/output/output_mapping.hpp>
#include <mosas/vision/opencv_image_frame.hpp>
#include <mosas/vision/vision_visualizer.hpp>

#include <exception>
#include <filesystem>
#include <opencv2/imgproc.hpp>
#include <system_error>
#include <utility>

namespace mosas::output {

LocalVideoRecorder::LocalVideoRecorder(
    std::shared_ptr<LocalRecordingSession> session,
    LocalVideoRecorderOptions options)
    : session_(std::move(session)), options_(std::move(options)) {
    if (session_ == nullptr) {
        set_error("local recording session is null");
    } else if (!session_->valid()) {
        set_error(session_->configuration_error());
    }
}

LocalVideoRecorder::~LocalVideoRecorder() { stop(); }

bool LocalVideoRecorder::start() {
    if (session_ == nullptr || !session_->valid()) {
        if (session_ != nullptr) {
            set_error(session_->configuration_error());
        }
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(error_mutex_);
        last_error_.clear();
    }
    failed_ = false;
    stopped_ = false;
    cancelled_.store(false, std::memory_order_release);
    started_ = true;
    return true;
}

void LocalVideoRecorder::reset() noexcept {
    stop();
    if (session_ != nullptr) {
        session_->reset();
    }
    recording_bgr_frame_.release();
    buffered_frames_.clear();
    buffered_pixel_bytes_ = 0;
    recording_frame_index_ = 0;
    frames_skipped_for_memory_limit_ = 0;
    recording_generation_ = 0;
    launch_window_active_ = false;
    started_ = false;
    stopped_ = false;
    cancelled_.store(false, std::memory_order_release);
    failed_ = false;
    {
        std::lock_guard<std::mutex> lock(error_mutex_);
        last_error_.clear();
    }
}

bool LocalVideoRecorder::record(const runtime::ProcessedFrame& frame) {
    if (cancelled_.load(std::memory_order_acquire)) {
        return true;
    }
    if (!started_ || stopped_ || failed_) {
        if (!started_ && !stopped_) {
            set_error("local video recorder is not started");
        }
        return stopped_;
    }
    try {
        if (session_->config().mode == LocalRecordingMode::continuous) {
            return record_continuous(frame);
        }
        return record_launch_window(frame);
    } catch (const cv::Exception& exception) {
        set_error(exception.what());
    } catch (const std::exception& exception) {
        set_error(exception.what());
    } catch (...) {
        set_error("local video recording failed");
    }
    release_recording();
    return false;
}

bool LocalVideoRecorder::record_continuous(
    const runtime::ProcessedFrame& frame) {
    if (!recording_writer_.isOpened() && !open_recording(0)) {
        return false;
    }
    if (!prepare_recording_frame(frame, &recording_bgr_frame_)) {
        return false;
    }
    if (!write_recording_frame(recording_bgr_frame_)) {
        release_recording();
        return false;
    }
    ++recording_frame_index_;
    return true;
}

bool LocalVideoRecorder::record_launch_window(
    const runtime::ProcessedFrame& frame) {
    const LocalWindowDecision decision =
        session_->classify(frame, LocalRecordingSinkKind::video);
    if (decision.action == LocalWindowAction::invalid) {
        set_error(decision.error);
        return false;
    }
    if (decision.action == LocalWindowAction::ignore) {
        return true;
    }

    if (!launch_window_active_) {
        launch_window_active_ = true;
        recording_generation_ = decision.generation;
        recording_frame_index_ = 0;
        buffered_frames_.clear();
        buffered_pixel_bytes_ = 0;
    }
    if (decision.action == LocalWindowAction::complete) {
        return persist_buffered_window(true);
    }

    const auto& config = session_->config();
    const std::uint64_t frame_bytes =
        static_cast<std::uint64_t>(config.width) *
        static_cast<std::uint64_t>(config.height) * 3ULL;
    if (frame_bytes > config.max_buffer_bytes - buffered_pixel_bytes_) {
        ++frames_skipped_for_memory_limit_;
        return true;
    }

    cv::Mat image;
    if (!prepare_recording_frame(frame, &image)) {
        return false;
    }
    if (image.total() * image.elemSize() != frame_bytes) {
        set_error(
            "recording frame byte size does not match configured output size");
        return false;
    }
    buffered_frames_.push_back(std::move(image));
    buffered_pixel_bytes_ += static_cast<std::size_t>(frame_bytes);
    ++recording_frame_index_;
    return true;
}

bool LocalVideoRecorder::prepare_recording_frame(
    const runtime::ProcessedFrame& frame, cv::Mat* output) {
    if (output == nullptr) {
        set_error("local video output frame is null");
        return false;
    }
    cv::Mat source = vision::make_bgr8_mat_view(frame.frame.image.view());
    if (source.empty()) {
        set_error("camera frame must be a valid BGR8 image");
        return false;
    }

    cv::Mat rendered;
    if (options_.draw_visualization) {
        source.copyTo(rendered);
        VisionVisualizer::draw_result(
            rendered, detail::make_legacy_vision_result(frame.vision_detection));
        VisionVisualizer::draw_guidance_overlay(
            rendered,
            detail::make_legacy_overlay(frame.overlay, frame.flight_state));
        source = rendered;
    }

    const auto& config = session_->config();
    if (source.cols != config.width || source.rows != config.height) {
        cv::resize(source, *output, cv::Size(config.width, config.height), 0.0,
                   0.0, cv::INTER_LINEAR);
    } else {
        source.copyTo(*output);
    }
    if (output->empty() || output->type() != CV_8UC3 ||
        output->size() != cv::Size(config.width, config.height)) {
        set_error("recording frame does not match configured output size");
        return false;
    }
    return true;
}

bool LocalVideoRecorder::open_recording(std::uint64_t generation) {
    const LocalRecordingPaths paths =
        session_->paths_for_generation(generation);
    if (!paths.video.parent_path().empty()) {
        std::error_code directory_error;
        std::filesystem::create_directories(paths.video.parent_path(),
                                            directory_error);
        if (directory_error) {
            set_error("create recording directory failed: " +
                      paths.video.parent_path().string() + " (" +
                      directory_error.message() + ")");
            return false;
        }
    }
    const auto& config = session_->config();
    if (!recording_writer_.open(
            paths.video.string(), cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
            config.fps,
            cv::Size(config.width + config.width % 2,
                     config.height + config.height % 2), true)) {
        set_error("open recording writer failed: " + paths.video.string());
        return false;
    }
    recording_generation_ = generation;
    recording_frame_index_ = 0;
    return true;
}

bool LocalVideoRecorder::write_recording_frame(const cv::Mat& image) {
    if (!recording_writer_.isOpened()) {
        set_error("recording output is not open");
        return false;
    }
    // MJPEG backends truncate odd sizes. Pad only the encoded copy, preserving
    // native sensor pixels, algorithm geometry and launch-window buffer accounting.
    cv::Mat padded;
    const cv::Mat* encoded = &image;
    if (image.cols % 2 || image.rows % 2) {
        cv::copyMakeBorder(image, padded, 0, image.rows % 2, 0, image.cols % 2,
                           cv::BORDER_REPLICATE);
        encoded = &padded;
    }
    recording_writer_.write(*encoded);
    if (!recording_writer_.isOpened()) {
        set_error("write recording frame failed");
        return false;
    }
    return true;
}

bool LocalVideoRecorder::finalize_recording(
    std::uint64_t expected_frame_count) {
    const LocalRecordingPaths paths =
        session_->paths_for_generation(recording_generation_);
    if (recording_writer_.isOpened()) {
        recording_writer_.release();
    }

    std::error_code file_error;
    const std::uintmax_t video_size =
        std::filesystem::file_size(paths.video, file_error);
    if (file_error || video_size == 0) {
        set_error("finalized recording AVI is missing or empty: " +
                  paths.video.string());
        return false;
    }

    cv::VideoCapture video(paths.video.string());
    if (!video.isOpened()) {
        set_error("finalized recording AVI cannot be reopened: " +
                  paths.video.string());
        return false;
    }
    std::uint64_t video_frame_count = 0;
    cv::Mat decoded_frame;
    const auto& config = session_->config();
    const cv::Size encoded_size(config.width + config.width % 2,
                                config.height + config.height % 2);
    while (video.read(decoded_frame)) {
        if (decoded_frame.size() != encoded_size) {
            set_error("finalized recording AVI dimensions mismatch: " +
                      paths.video.string());
            return false;
        }
        ++video_frame_count;
    }
    video.release();
    if (video_frame_count != expected_frame_count) {
        set_error("finalized recording AVI frame count mismatch: " +
                  paths.video.string());
        return false;
    }
    return true;
}

bool LocalVideoRecorder::persist_buffered_window(bool complete) {
    const std::uint64_t generation = recording_generation_;
    if (buffered_frames_.empty()) {
        launch_window_active_ = false;
        buffered_pixel_bytes_ = 0;
        if (complete) {
            session_->mark_completed(LocalRecordingSinkKind::video, generation);
        }
        return true;
    }
    if (!open_recording(generation)) {
        release_recording();
        buffered_frames_.clear();
        buffered_pixel_bytes_ = 0;
        launch_window_active_ = false;
        return false;
    }

    const std::uint64_t expected_frame_count = buffered_frames_.size();
    for (const cv::Mat& frame : buffered_frames_) {
        if (!write_recording_frame(frame)) {
            release_recording();
            buffered_frames_.clear();
            buffered_pixel_bytes_ = 0;
            launch_window_active_ = false;
            return false;
        }
    }
    if (!finalize_recording(expected_frame_count)) {
        release_recording();
        buffered_frames_.clear();
        buffered_pixel_bytes_ = 0;
        launch_window_active_ = false;
        return false;
    }
    buffered_frames_.clear();
    buffered_pixel_bytes_ = 0;
    launch_window_active_ = false;
    if (complete) {
        session_->mark_completed(LocalRecordingSinkKind::video, generation);
    }
    return true;
}

void LocalVideoRecorder::release_recording() noexcept {
    if (recording_writer_.isOpened()) {
        recording_writer_.release();
    }
}

bool LocalVideoRecorder::take_rearm_request() noexcept {
    return session_ != nullptr && session_->take_rearm_request();
}

void LocalVideoRecorder::cancel() noexcept {
    cancelled_.store(true, std::memory_order_release);
}

void LocalVideoRecorder::stop() noexcept {
    if (stopped_) {
        return;
    }
    stopped_ = true;
    if (session_ != nullptr &&
        session_->config().mode == LocalRecordingMode::launch_window &&
        launch_window_active_) {
        try {
            if (!persist_buffered_window(false)) {
                failed_ = true;
            }
        } catch (const std::exception& exception) {
            set_error(exception.what());
            release_recording();
            buffered_frames_.clear();
            buffered_pixel_bytes_ = 0;
            launch_window_active_ = false;
        } catch (...) {
            set_error("save partial launch video failed");
            release_recording();
            buffered_frames_.clear();
            buffered_pixel_bytes_ = 0;
            launch_window_active_ = false;
        }
    } else if (session_ != nullptr &&
               session_->config().mode == LocalRecordingMode::continuous &&
               recording_writer_.isOpened()) {
        try {
            if (!finalize_recording(recording_frame_index_)) {
                failed_ = true;
                release_recording();
            }
        } catch (const std::exception& exception) {
            set_error(exception.what());
            release_recording();
        } catch (...) {
            set_error("finalize continuous video failed");
            release_recording();
        }
    }
    release_recording();
}

std::string LocalVideoRecorder::last_error() const {
    std::lock_guard<std::mutex> lock(error_mutex_);
    return last_error_;
}

std::size_t LocalVideoRecorder::buffered_pixel_bytes() const noexcept {
    return buffered_pixel_bytes_;
}

std::size_t LocalVideoRecorder::buffered_frame_count() const noexcept {
    return buffered_frames_.size();
}

std::uint64_t LocalVideoRecorder::frames_skipped_for_memory_limit() const noexcept {
    return frames_skipped_for_memory_limit_;
}

bool LocalVideoRecorder::launch_window_active() const noexcept {
    return launch_window_active_;
}

void LocalVideoRecorder::set_error(std::string error) noexcept {
    failed_ = true;
    try {
        std::lock_guard<std::mutex> lock(error_mutex_);
        if (last_error_.empty()) {
            last_error_ = std::move(error);
        }
    } catch (...) {
    }
}

}  // namespace mosas::output
