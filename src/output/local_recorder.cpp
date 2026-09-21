#include <mosas/output/local_recorder.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <utility>

#include <opencv2/imgproc.hpp>

namespace mosas::output {
namespace {

constexpr std::string_view kFlightDataHeader =
    "frame_index,frame_timestamp_ns,frame_received_timestamp_ns,"
    "imu_timestamp_ns,vision_found,target_x,target_y,target_width,"
    "target_height,target_area,target_center_x,target_center_y,"
    "imu_acceleration_x_mps2,imu_acceleration_y_mps2,"
    "imu_acceleration_z_mps2,imu_angular_velocity_x_rad_s,"
    "imu_angular_velocity_y_rad_s,imu_angular_velocity_z_rad_s,"
    "imu_pitch_rad,imu_attitude_valid,imu_launched,los_valid,"
    "los_q_y_rad,los_q_z_rad,los_rate_valid,los_rate_q_y_rad_s,"
    "los_rate_q_z_rad_s,guidance_valid,"
    "navigation_acceleration_x_mps2,navigation_acceleration_y_mps2,"
    "navigation_acceleration_z_mps2,body_acceleration_x_mps2,"
    "body_acceleration_y_mps2,body_acceleration_z_mps2,"
    "body_overload_x_g,body_overload_y_g,body_overload_z_g,"
    "command_overload_g,command_phase_rad\n";

std::string make_session_id() {
    static std::atomic<std::uint64_t> next_id{0};
    const auto now = std::chrono::system_clock::now();
    const auto seconds = std::chrono::time_point_cast<std::chrono::seconds>(now);
    const auto micros = std::chrono::duration_cast<std::chrono::microseconds>(
                            now - seconds)
                            .count();
    const std::time_t time = std::chrono::system_clock::to_time_t(now);
    std::tm local_time{};
    localtime_r(&time, &local_time);

    std::ostringstream stream;
    stream << std::put_time(&local_time, "%Y%m%d_%H%M%S") << '_'
           << std::setfill('0') << std::setw(6) << micros << '_'
           << std::setfill('0') << std::setw(6) << next_id.fetch_add(1);
    return stream.str();
}

std::filesystem::path timestamped_video_path(
    const std::filesystem::path& configured_path,
    const std::string& session_id) {
    std::string stem = configured_path.stem().string();
    if (stem.empty()) {
        stem = "recording";
    }
    std::string extension = configured_path.extension().string();
    if (extension.empty()) {
        extension = ".avi";
    }
    return configured_path.parent_path() /
           (stem + "_" + session_id + extension);
}

}  // 匿名命名空间结束

LocalRecorder::LocalRecorder(LocalRecorderConfig config)
    : config_(std::move(config)) {
    validate_config();
}

LocalRecorder::~LocalRecorder() { stop(); }

bool LocalRecorder::validate_config() {
    if (config_.recording_path.empty()) {
        last_error_ = "recording path must not be empty";
        return false;
    }
    if (!std::isfinite(config_.fps) || config_.fps <= 0.0) {
        last_error_ = "recording FPS must be positive";
        return false;
    }
    if (config_.width <= 0 || config_.height <= 0) {
        last_error_ = "recording output size must be positive";
        return false;
    }
    if (config_.mode == LocalRecordingMode::launch_window &&
        config_.window_duration_ns <= 0) {
        last_error_ = "launch-window duration must be positive";
        return false;
    }
    if (config_.mode == LocalRecordingMode::launch_window &&
        (config_.max_buffer_bytes == 0 ||
         config_.max_buffer_bytes > kMatchRecordingMemoryLimitBytes)) {
        last_error_ = "launch-window buffer limit is outside the allowed range";
        return false;
    }
    last_error_.clear();
    return true;
}

bool LocalRecorder::publish(
    const runtime::CameraFrame& frame, const VisionResult& vision_result,
    const runtime::ImuStateSnapshot& imu_state,
    const PngGuidanceOutput& guidance, const VisionOverlayData& overlay) {
    if (stopped_) {
        return true;
    }
    if (failed_) {
        return false;
    }
    if (!last_error_.empty()) {
        failed_ = true;
        return false;
    }
    if (frame.image.empty() || frame.image.type() != CV_8UC3) {
        last_error_ = "camera frame must be a non-empty CV_8UC3 image";
        failed_ = true;
        return false;
    }

    try {
        if (config_.mode == LocalRecordingMode::continuous) {
            return publish_continuous(frame, vision_result, imu_state,
                                      guidance, overlay);
        }
        return publish_launch_window(frame, vision_result, imu_state,
                                     guidance, overlay);
    } catch (const cv::Exception& exception) {
        last_error_ = exception.what();
    } catch (const std::exception& exception) {
        last_error_ = exception.what();
    } catch (...) {
        last_error_ = "local recording failed";
    }
    failed_ = true;
    release_recording();
    return false;
}

bool LocalRecorder::publish_continuous(
    const runtime::CameraFrame& frame, const VisionResult& vision_result,
    const runtime::ImuStateSnapshot& imu_state,
    const PngGuidanceOutput& guidance, const VisionOverlayData& overlay) {
    if (!recording_writer_.isOpened() || !flight_data_writer_.is_open()) {
        if (!open_recording()) {
            failed_ = true;
            return false;
        }
    }
    if (!prepare_recording_frame(frame, &recording_bgr_frame_)) {
        failed_ = true;
        return false;
    }
    const std::string row = make_csv_row(recording_frame_index_, frame,
                                         vision_result, imu_state, guidance,
                                         overlay);
    if (!write_recording_frame(recording_bgr_frame_, row)) {
        failed_ = true;
        release_recording();
        return false;
    }
    ++recording_frame_index_;
    return true;
}

bool LocalRecorder::publish_launch_window(
    const runtime::CameraFrame& frame, const VisionResult& vision_result,
    const runtime::ImuStateSnapshot& imu_state,
    const PngGuidanceOutput& guidance, const VisionOverlayData& overlay) {
    if (launch_window_active_) {
        if (frame.timestamp_ns >= launch_window_end_ns_) {
            if (!persist_buffered_window(true)) {
                failed_ = true;
                return false;
            }
            return true;
        }
        if (frame.timestamp_ns < launch_window_start_ns_) {
            return true;
        }
    } else {
        if (!imu_state.launched || imu_state.launch_timestamp_ns < 0) {
            return true;
        }
        if (has_completed_launch_generation_ &&
            imu_state.launch_generation <= completed_launch_generation_) {
            return true;
        }
        if (imu_state.launch_timestamp_ns >
            std::numeric_limits<std::int64_t>::max() -
                config_.window_duration_ns) {
            last_error_ = "launch-window end timestamp overflow";
            failed_ = true;
            return false;
        }
        launch_window_start_ns_ = imu_state.launch_timestamp_ns;
        launch_window_end_ns_ = launch_window_start_ns_ +
                                config_.window_duration_ns;
        recording_launch_generation_ = imu_state.launch_generation;
        recording_frame_index_ = 0;
        buffered_frames_.clear();
        buffered_pixel_bytes_ = 0;
        launch_window_active_ = true;
        if (frame.timestamp_ns < launch_window_start_ns_) {
            return true;
        }
        if (frame.timestamp_ns >= launch_window_end_ns_) {
            return persist_buffered_window(true);
        }
    }

    const std::uint64_t frame_bytes =
        static_cast<std::uint64_t>(config_.width) *
        static_cast<std::uint64_t>(config_.height) * 3ULL;
    if (frame_bytes > config_.max_buffer_bytes - buffered_pixel_bytes_) {
        if (frames_skipped_for_memory_limit_ == 0) {
            std::clog << "起飞录制窗口达到内存上限，开始跳帧: 已缓存 "
                      << buffered_frames_.size() << " 帧 ("
                      << buffered_pixel_bytes_ << " 字节)\n";
        }
        ++frames_skipped_for_memory_limit_;
        return true;
    }

    cv::Mat image;
    if (!prepare_recording_frame(frame, &image)) {
        failed_ = true;
        return false;
    }
    if (image.total() * image.elemSize() != frame_bytes) {
        last_error_ = "recording frame byte size does not match configured output size";
        failed_ = true;
        return false;
    }
    BufferedFrame buffered;
    buffered.image = std::move(image);
    buffered.csv_row = make_csv_row(recording_frame_index_, frame,
                                    vision_result, imu_state, guidance, overlay);
    buffered_frames_.push_back(std::move(buffered));
    buffered_pixel_bytes_ += static_cast<std::size_t>(frame_bytes);
    ++recording_frame_index_;
    return true;
}

bool LocalRecorder::prepare_recording_frame(
    const runtime::CameraFrame& frame, cv::Mat* output) {
    if (output == nullptr) {
        return false;
    }
    if (frame.image.cols != config_.width || frame.image.rows != config_.height) {
        cv::resize(frame.image, *output, cv::Size(config_.width, config_.height),
                   0.0, 0.0, cv::INTER_LINEAR);
    } else {
        frame.image.copyTo(*output);
    }
    if (output->empty() || output->type() != CV_8UC3 ||
        output->size() != cv::Size(config_.width, config_.height)) {
        last_error_ = "recording frame does not match configured output size";
        return false;
    }
    return true;
}

std::string LocalRecorder::make_csv_row(
    std::uint64_t frame_index, const runtime::CameraFrame& frame,
    const VisionResult& vision_result,
    const runtime::ImuStateSnapshot& imu_state,
    const PngGuidanceOutput& guidance,
    const VisionOverlayData& overlay) const {
    std::ostringstream row;
    row << frame_index << ',' << frame.timestamp_ns << ','
        << frame.received_timestamp_ns << ',' << imu_state.timestamp_ns << ','
        << (vision_result.found ? 1 : 0) << ',' << vision_result.blob.x << ','
        << vision_result.blob.y << ',' << vision_result.blob.width << ','
        << vision_result.blob.height << ',' << vision_result.blob.area << ','
        << vision_result.blob.center_x << ',' << vision_result.blob.center_y
        << ',' << imu_state.acceleration.x << ','
        << imu_state.acceleration.y << ',' << imu_state.acceleration.z << ','
        << imu_state.angular_velocity.x << ','
        << imu_state.angular_velocity.y << ','
        << imu_state.angular_velocity.z << ',' << imu_state.pitch_rad << ','
        << (imu_state.attitude_valid ? 1 : 0) << ','
        << (imu_state.launched ? 1 : 0) << ','
        << (overlay.line_of_sight_valid ? 1 : 0) << ','
        << overlay.line_of_sight_q_y_rad << ','
        << overlay.line_of_sight_q_z_rad << ','
        << (overlay.line_of_sight_rate_valid ? 1 : 0) << ','
        << overlay.line_of_sight_rate_q_y_rad_s << ','
        << overlay.line_of_sight_rate_q_z_rad_s << ','
        << (guidance.valid ? 1 : 0) << ','
        << guidance.navigation_acceleration.x << ','
        << guidance.navigation_acceleration.y << ','
        << guidance.navigation_acceleration.z << ','
        << guidance.body_acceleration.x << ','
        << guidance.body_acceleration.y << ','
        << guidance.body_acceleration.z << ','
        << guidance.body_overload.x << ',' << guidance.body_overload.y << ','
        << guidance.body_overload.z << ',' << guidance.command_overload << ','
        << guidance.command_phase << '\n';
    return row.str();
}

bool LocalRecorder::open_recording() {
    if (config_.recording_path.empty()) {
        last_error_ = "recording path must not be empty";
        return false;
    }
    if (!std::isfinite(config_.fps) || config_.fps <= 0.0) {
        last_error_ = "recording FPS must be positive";
        return false;
    }
    if (config_.width <= 0 || config_.height <= 0) {
        last_error_ = "recording output size must be positive";
        return false;
    }

    release_recording();
    const std::filesystem::path configured_path(config_.recording_path);
    const std::filesystem::path video_path =
        timestamped_video_path(configured_path, make_session_id());
    std::filesystem::path csv_path = video_path;
    csv_path.replace_extension(".csv");

    if (!video_path.parent_path().empty()) {
        std::error_code directory_error;
        std::filesystem::create_directories(video_path.parent_path(),
                                            directory_error);
        if (directory_error) {
            last_error_ = "create recording directory failed: " +
                          video_path.parent_path().string() + " (" +
                          directory_error.message() + ")";
            return false;
        }
    }

    recording_video_path_ = video_path.string();
    flight_data_path_ = csv_path.string();
    recording_frame_index_ = 0;
    if (!recording_writer_.open(
            recording_video_path_, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
            config_.fps, cv::Size(config_.width, config_.height), true)) {
        last_error_ = "open recording writer failed: " + recording_video_path_;
        return false;
    }
    flight_data_writer_.open(flight_data_path_,
                             std::ios::out | std::ios::trunc);
    if (!flight_data_writer_.is_open()) {
        recording_writer_.release();
        last_error_ = "open flight data CSV failed: " + flight_data_path_;
        return false;
    }
    flight_data_writer_ << kFlightDataHeader;
    flight_data_writer_.flush();
    if (!flight_data_writer_) {
        release_recording();
        last_error_ = "write flight data CSV header failed: " +
                      flight_data_path_;
        return false;
    }
    last_error_.clear();
    return true;
}

bool LocalRecorder::write_recording_frame(const cv::Mat& image,
                                          const std::string& csv_row) {
    if (!recording_writer_.isOpened() || !flight_data_writer_.is_open()) {
        last_error_ = "recording output is not open";
        return false;
    }
    recording_writer_.write(image);
    if (!recording_writer_.isOpened()) {
        last_error_ = "write recording frame failed";
        return false;
    }
    flight_data_writer_ << csv_row;
    flight_data_writer_.flush();
    if (!flight_data_writer_) {
        last_error_ = "write flight data CSV row failed: " + flight_data_path_;
        return false;
    }
    return true;
}

bool LocalRecorder::finalize_recording(
    std::uint64_t expected_frame_count) {
    if (recording_writer_.isOpened()) {
        recording_writer_.release();
    }

    if (flight_data_writer_.is_open()) {
        flight_data_writer_.flush();
        if (!flight_data_writer_) {
            last_error_ = "flush flight data CSV failed: " + flight_data_path_;
        }
        flight_data_writer_.close();
        if (flight_data_writer_.fail() && last_error_.empty()) {
            last_error_ = "close flight data CSV failed: " + flight_data_path_;
        }
    }
    if (!last_error_.empty()) {
        return false;
    }

    std::error_code file_error;
    const std::uintmax_t video_size =
        std::filesystem::file_size(recording_video_path_, file_error);
    if (file_error || video_size == 0) {
        last_error_ = "finalized recording AVI is missing or empty: " +
                      recording_video_path_;
        return false;
    }

    cv::VideoCapture video(recording_video_path_);
    if (!video.isOpened()) {
        last_error_ = "finalized recording AVI cannot be reopened: " +
                      recording_video_path_;
        return false;
    }
    std::uint64_t video_frame_count = 0;
    cv::Mat decoded_frame;
    while (video.read(decoded_frame)) {
        ++video_frame_count;
    }
    video.release();
    if (video_frame_count != expected_frame_count) {
        last_error_ = "finalized recording AVI frame count mismatch: " +
                      recording_video_path_;
        return false;
    }

    std::ifstream csv(flight_data_path_);
    std::string csv_header;
    if (!csv.is_open() || !std::getline(csv, csv_header) ||
        csv_header != kFlightDataHeader.substr(0, kFlightDataHeader.size() - 1)) {
        last_error_ = "finalized flight data CSV header is invalid: " +
                      flight_data_path_;
        return false;
    }
    std::uint64_t csv_row_count = 0;
    std::string csv_row;
    while (std::getline(csv, csv_row)) {
        ++csv_row_count;
    }
    if (csv.bad() || csv_row_count != expected_frame_count) {
        last_error_ = "finalized flight data CSV row count mismatch: " +
                      flight_data_path_;
        return false;
    }
    return true;
}

bool LocalRecorder::persist_buffered_window(bool complete) {
    if (buffered_frames_.empty()) {
        launch_window_active_ = false;
        buffered_pixel_bytes_ = 0;
        if (complete) {
            completed_launch_generation_ = recording_launch_generation_;
            has_completed_launch_generation_ = true;
            rearm_request_pending_ = true;
        }
        return true;
    }
    if (!open_recording()) {
        release_recording();
        launch_window_active_ = false;
        buffered_frames_.clear();
        buffered_pixel_bytes_ = 0;
        return false;
    }

    const std::uint64_t expected_frame_count = buffered_frames_.size();
    for (const BufferedFrame& frame : buffered_frames_) {
        if (!write_recording_frame(frame.image, frame.csv_row)) {
            release_recording();
            launch_window_active_ = false;
            buffered_frames_.clear();
            buffered_pixel_bytes_ = 0;
            return false;
        }
    }
    if (!finalize_recording(expected_frame_count)) {
        failed_ = true;
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
        completed_launch_generation_ = recording_launch_generation_;
        has_completed_launch_generation_ = true;
        rearm_request_pending_ = true;
    }
    return true;
}

void LocalRecorder::release_recording() noexcept {
    if (recording_writer_.isOpened()) {
        recording_writer_.release();
    }
    if (flight_data_writer_.is_open()) {
        flight_data_writer_.flush();
        flight_data_writer_.close();
    }
}

void LocalRecorder::stop() noexcept {
    if (stopped_) {
        return;
    }
    stopped_ = true;
    if (config_.mode == LocalRecordingMode::launch_window &&
        launch_window_active_) {
        std::clog << "起飞录制窗口未完成，正在保存已缓存的部分数据\n";
        try {
            if (!persist_buffered_window(false)) {
                failed_ = true;
            }
        } catch (const std::exception& exception) {
            last_error_ = exception.what();
            failed_ = true;
            release_recording();
            buffered_frames_.clear();
            buffered_pixel_bytes_ = 0;
            launch_window_active_ = false;
        } catch (...) {
            last_error_ = "save partial launch recording failed";
            failed_ = true;
            release_recording();
            buffered_frames_.clear();
            buffered_pixel_bytes_ = 0;
            launch_window_active_ = false;
        }
    } else if (config_.mode == LocalRecordingMode::continuous &&
               recording_writer_.isOpened()) {
        try {
            if (!finalize_recording(recording_frame_index_)) {
                failed_ = true;
                release_recording();
            }
        } catch (const std::exception& exception) {
            last_error_ = exception.what();
            failed_ = true;
            release_recording();
        } catch (...) {
            last_error_ = "finalize continuous recording failed";
            failed_ = true;
            release_recording();
        }
    }
    release_recording();
}

void LocalRecorder::reset() noexcept {
    stop();
    buffered_frames_.clear();
    buffered_pixel_bytes_ = 0;
    recording_frame_index_ = 0;
    frames_skipped_for_memory_limit_ = 0;
    launch_window_start_ns_ = -1;
    launch_window_end_ns_ = -1;
    recording_launch_generation_ = 0;
    completed_launch_generation_ = 0;
    has_completed_launch_generation_ = false;
    launch_window_active_ = false;
    rearm_request_pending_ = false;
    stopped_ = false;
    failed_ = false;
    last_error_.clear();
    recording_video_path_.clear();
    flight_data_path_.clear();
    recording_bgr_frame_.release();
    validate_config();
}

bool LocalRecorder::take_rearm_request() noexcept {
    const bool pending = rearm_request_pending_;
    rearm_request_pending_ = false;
    return pending;
}

std::string LocalRecorder::last_error() const { return last_error_; }

LocalRecorderStats LocalRecorder::stats() const noexcept {
    return {buffered_pixel_bytes_, buffered_frames_.size(),
            frames_skipped_for_memory_limit_, launch_window_active_};
}

}  // namespace mosas::output
