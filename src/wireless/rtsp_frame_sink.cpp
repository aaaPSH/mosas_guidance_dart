#include <mosas/wireless/rtsp_frame_sink.hpp>

#include <utility>

#include <mosas/vision/vision_visualizer.hpp>

#include <cmath>
#include <filesystem>
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
    return publish_impl(frame, vision_result, imu_state, guidance, nullptr);
}

bool RtspFrameSink::publish(
    const mosas::runtime::CameraFrame& frame,
    const VisionResult& vision_result,
    const mosas::runtime::ImuStateSnapshot& imu_state,
    const PngGuidanceOutput& guidance,
    const VisionOverlayData& overlay) {
    return publish_impl(frame, vision_result, imu_state, guidance, &overlay);
}

bool RtspFrameSink::publish_impl(
    const mosas::runtime::CameraFrame& frame,
    const VisionResult& vision_result,
    const mosas::runtime::ImuStateSnapshot& imu_state,
    const PngGuidanceOutput& guidance,
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

        if (config_.enable_wireless_stream && !wireless_stream_failed_) {
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
                // 无线链路是可选输出；失败后停用重试，让本地内录继续。
                wireless_stream_failed_ = true;
            }
            if (!wireless_stream_failed_) {
                if (cancel_requested_.load()) {
                    return true;
                }
                if (!streamer->send_bgr(bgr_frame_, frame.timestamp_ns,
                                        &last_error_)) {
                    // send_bgr() 失败时 RtspStreamer 已释放自己的网络资源。
                    // 标记失败并继续执行下面的本地录像逻辑。
                    wireless_stream_failed_ = true;
                }
            }
        }
        if (config_.enable_wireless_stream && wireless_stream_failed_ &&
            !config_.enable_recording) {
            return false;
        }
        if (config_.enable_recording) {
            const std::filesystem::path recording_path(config_.recording_path);
            if (recording_path.has_parent_path()) {
                std::error_code directory_error;
                std::filesystem::create_directories(
                    recording_path.parent_path(), directory_error);
                if (directory_error) {
                    last_error_ = "create recording directory failed: " +
                                  recording_path.parent_path().string() +
                                  " (" + directory_error.message() + ")";
                    return false;
                }
            }
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

            if (!flight_data_writer_.is_open()) {
                std::filesystem::path csv_path(config_.recording_path);
                if (csv_path.has_extension()) {
                    csv_path.replace_extension(".csv");
                } else {
                    csv_path += ".csv";
                }
                flight_data_path_ = csv_path.string();
                flight_data_writer_.open(
                    flight_data_path_, std::ios::out | std::ios::trunc);
                if (!flight_data_writer_.is_open()) {
                    last_error_ = "open flight data CSV failed: " +
                                  flight_data_path_;
                    return false;
                }
                flight_data_writer_
                    << "frame_timestamp_ns,imu_timestamp_ns,flight_phase,"
                       "vision_found,target_x,target_y,target_width,target_height,"
                       "target_area,target_center_x,target_center_y,"
                       "imu_acceleration_x_mps2,imu_acceleration_y_mps2,"
                       "imu_acceleration_z_mps2,imu_angular_velocity_x_rad_s,"
                       "imu_angular_velocity_y_rad_s,imu_angular_velocity_z_rad_s,"
                       "attitude_pitch_rad,attitude_yaw_rad,attitude_roll_rad,"
                       "velocity_x_mps,velocity_y_mps,velocity_z_mps,"
                       "los_valid,los_q_y_rad,los_q_z_rad,los_rate_valid,"
                       "los_rate_q_y_rad_s,los_rate_q_z_rad_s,guidance_valid,"
                       "navigation_acceleration_x_mps2,navigation_acceleration_y_mps2,"
                       "navigation_acceleration_z_mps2,body_acceleration_x_mps2,"
                       "body_acceleration_y_mps2,body_acceleration_z_mps2,"
                       "body_overload_x_g,body_overload_y_g,body_overload_z_g,"
                       "command_overload_g,command_phase_rad\n";
                if (!flight_data_writer_) {
                    last_error_ = "write flight data CSV header failed: " +
                                  flight_data_path_;
                    return false;
                }
            }

            const auto phase_name = [](mosas::runtime::FlightPhase phase) {
                switch (phase) {
                    case mosas::runtime::FlightPhase::pre_launch:
                        return "pre_launch";
                    case mosas::runtime::FlightPhase::ejection:
                        return "ejection";
                    case mosas::runtime::FlightPhase::free_flight:
                        return "free_flight";
                }
                return "unknown";
            };
            const VisionOverlayData empty_overlay{};
            const VisionOverlayData& data =
                overlay == nullptr ? empty_overlay : *overlay;
            flight_data_writer_
                << frame.timestamp_ns << ',' << imu_state.timestamp_ns << ','
                << phase_name(imu_state.phase) << ','
                << (vision_result.found ? 1 : 0) << ',' << vision_result.blob.x
                << ',' << vision_result.blob.y << ',' << vision_result.blob.width
                << ',' << vision_result.blob.height << ','
                << vision_result.blob.area << ',' << vision_result.blob.center_x
                << ',' << vision_result.blob.center_y << ','
                << imu_state.acceleration.x << ',' << imu_state.acceleration.y
                << ',' << imu_state.acceleration.z << ','
                << imu_state.angular_velocity.x << ','
                << imu_state.angular_velocity.y << ','
                << imu_state.angular_velocity.z << ',' << imu_state.attitude.pitch
                << ',' << imu_state.attitude.yaw << ',' << imu_state.attitude.roll
                << ',' << imu_state.velocity.x << ',' << imu_state.velocity.y
                << ',' << imu_state.velocity.z << ','
                << (data.line_of_sight_valid ? 1 : 0) << ','
                << data.line_of_sight_q_y_rad << ',' << data.line_of_sight_q_z_rad
                << ',' << (data.line_of_sight_rate_valid ? 1 : 0) << ','
                << data.line_of_sight_rate_q_y_rad_s << ','
                << data.line_of_sight_rate_q_z_rad_s << ','
                << (guidance.valid ? 1 : 0) << ','
                << guidance.navigation_acceleration.x << ','
                << guidance.navigation_acceleration.y << ','
                << guidance.navigation_acceleration.z << ','
                << guidance.body_acceleration.x << ','
                << guidance.body_acceleration.y << ','
                << guidance.body_acceleration.z << ','
                << guidance.body_overload.x << ',' << guidance.body_overload.y
                << ',' << guidance.body_overload.z << ','
                << guidance.command_overload << ',' << guidance.command_phase
                << '\n';
            flight_data_writer_.flush();
            if (!flight_data_writer_) {
                last_error_ = "write flight data CSV row failed: " +
                              flight_data_path_;
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
    if (flight_data_writer_.is_open()) {
        flight_data_writer_.close();
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
    wireless_stream_failed_ = false;
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
