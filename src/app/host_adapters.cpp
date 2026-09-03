#include <mosas/app/host_adapters.hpp>

#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <utility>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace mosas::app {
namespace {

runtime::TimestampNs steady_timestamp_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

}  // namespace

OpenCvCameraSource::OpenCvCameraSource(CameraAppConfig config)
    : config_(std::move(config)) {}

OpenCvCameraSource::~OpenCvCameraSource() {
    cancel();
}

bool OpenCvCameraSource::configure(const runtime::CameraCaptureConfig& config) {
    std::lock_guard<std::mutex> lock(mutex_);
    cancelled_ = false;
    configured_ = false;
    camera_.release();
    config_.capture_mode = config.mode;
    if (config_.device.empty() || config_.width <= 0 || config_.height <= 0 ||
        config_.fps <= 0) {
        return false;
    }
    if (!camera_.open(config_.device, cv::CAP_V4L2)) {
        return false;
    }
    camera_.set(cv::CAP_PROP_FRAME_WIDTH, config_.width);
    camera_.set(cv::CAP_PROP_FRAME_HEIGHT, config_.height);
    camera_.set(cv::CAP_PROP_FPS, config_.fps);
    camera_.set(cv::CAP_PROP_EXPOSURE, config_.exposure_us);
    camera_.set(cv::CAP_PROP_GAIN, config_.gain);
    // V4L2 常用 0.75 表示自动曝光、0.25 表示手动曝光。
    camera_.set(cv::CAP_PROP_AUTO_EXPOSURE,
                config_.auto_exposure ? 0.75 : 0.25);
    undistort_map_x_.release();
    undistort_map_y_.release();
    if (config_.undistort) {
        const cv::Mat camera_matrix =
            (cv::Mat_<double>(3, 3) << config_.camera_matrix.m00,
             config_.camera_matrix.m01, config_.camera_matrix.m02,
             config_.camera_matrix.m10, config_.camera_matrix.m11,
             config_.camera_matrix.m12, config_.camera_matrix.m20,
             config_.camera_matrix.m21, config_.camera_matrix.m22);
        const cv::Mat distortion =
            (cv::Mat_<double>(1, 5) << config_.distortion.k1,
             config_.distortion.k2, config_.distortion.p1,
             config_.distortion.p2, config_.distortion.k3);
        cv::initUndistortRectifyMap(
            camera_matrix, distortion, cv::Mat(), camera_matrix,
            cv::Size(config_.width, config_.height), CV_32FC1,
            undistort_map_x_, undistort_map_y_);
    }
    configured_ = true;
    return true;
}

bool OpenCvCameraSource::capture(runtime::CameraFrame* frame) {
    if (frame == nullptr) {
        return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (cancelled_ || !configured_ || !camera_.isOpened() ||
        !camera_.read(frame->image) || frame->image.empty()) {
        return false;
    }
    // 在图像格式转换、缩放和去畸变之前记录时间，尽量接近取帧时刻。
    frame->timestamp_ns = steady_timestamp_ns();
    if (frame->image.channels() == 1) {
        cv::cvtColor(frame->image, frame->image, cv::COLOR_GRAY2BGR);
    } else if (frame->image.channels() == 4) {
        cv::cvtColor(frame->image, frame->image, cv::COLOR_BGRA2BGR);
    }
    if (frame->image.cols != config_.width || frame->image.rows != config_.height) {
        cv::resize(frame->image, frame->image,
                   cv::Size(config_.width, config_.height));
    }
    if (config_.undistort) {
        cv::remap(frame->image, undistorted_frame_, undistort_map_x_,
                  undistort_map_y_, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
        frame->image = undistorted_frame_;
    }
    return true;
}

void OpenCvCameraSource::cancel() noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    cancelled_ = true;
    configured_ = false;
    camera_.release();
}

SimulatedImuSource::SimulatedImuSource(double sample_rate_hz)
    : period_(std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(1.0 / sample_rate_hz))) {
    if (sample_rate_hz <= 0.0 || !std::isfinite(sample_rate_hz) ||
        period_.count() <= 0) {
        period_ = std::chrono::nanoseconds(1);
    }
}

bool SimulatedImuSource::read(runtime::ImuSample* sample) {
    if (sample == nullptr) {
        return false;
    }
    std::unique_lock<std::mutex> lock(mutex_);
    if (cancelled_) {
        return false;
    }
    const auto now = std::chrono::steady_clock::now();
    if (!started_) {
        next_deadline_ = now;
        started_ = true;
    }
    if (condition_.wait_until(lock, next_deadline_,
                              [this] { return cancelled_; })) {
        return false;
    }
    const auto timestamp = steady_timestamp_ns();
    next_deadline_ = std::chrono::steady_clock::now() + period_;
    sample->timestamp_ns = timestamp;
    sample->acceleration = {0.0, 0.0, 0.0};
    sample->angular_velocity = {0.0, 0.0, 0.0};
    return true;
}

void SimulatedImuSource::cancel() noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    cancelled_ = true;
    condition_.notify_all();
}

LoggingCommandSink::LoggingCommandSink() : output_(&std::cout) {}

LoggingCommandSink::LoggingCommandSink(std::ostream& output) : output_(&output) {}

bool LoggingCommandSink::send(const runtime::GuidanceCommand& command) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (output_ == nullptr) {
        return false;
    }
    *output_ << "[日志] timestamp_ns=" << command.timestamp_ns
             << " valid=" << (command.output.valid ? "true" : "false")
             << " command_overload=" << command.output.command_overload << '\n';
    return static_cast<bool>(*output_);
}

}  // namespace mosas::app
