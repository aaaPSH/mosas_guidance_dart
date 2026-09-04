#include <mosas/app/host_adapters.hpp>

#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
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

int fourcc_code(const std::string& value) {
    return cv::VideoWriter::fourcc(value[0], value[1], value[2], value[3]);
}

std::string fourcc_text(double value) {
    if (!std::isfinite(value) || value <= 0.0) {
        return "unknown";
    }
    const auto code = static_cast<std::uint32_t>(value);
    std::string result(4, ' ');
    for (int index = 0; index < 4; ++index) {
        const auto character = static_cast<unsigned char>(
            (code >> (8 * index)) & 0xffU);
        result[index] = std::isprint(character) ?
            static_cast<char>(character) : '.';
    }
    return result;
}

void set_and_report(cv::VideoCapture* camera, int property,
                    const char* name, double requested) {
    const bool accepted = camera->set(property, requested);
    const double actual = camera->get(property);
    std::cerr << "[相机] " << name << ": requested=" << std::fixed
              << std::setprecision(2) << requested
              << " set=" << (accepted ? "ok" : "failed")
              << " actual=" << actual << '\n';
}

void set_fourcc_and_report(cv::VideoCapture* camera,
                           const std::string& requested) {
    const int code = fourcc_code(requested);
    const bool accepted = camera->set(cv::CAP_PROP_FOURCC, code);
    const double actual = camera->get(cv::CAP_PROP_FOURCC);
    std::cerr << "[相机] pixel_format: requested=" << requested
              << " set=" << (accepted ? "ok" : "failed")
              << " actual=" << fourcc_text(actual) << '\n';
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
        config_.fps <= 0 ||
        (config_.pixel_format != "auto" && config_.pixel_format.size() != 4)) {
        return false;
    }
    if (!camera_.open(config_.device, cv::CAP_V4L2)) {
        return false;
    }
    if (config_.pixel_format != "auto") {
        set_fourcc_and_report(&camera_, config_.pixel_format);
    } else {
        std::cerr << "[相机] pixel_format: auto，使用驱动默认格式\n";
    }
    set_and_report(&camera_, cv::CAP_PROP_FRAME_WIDTH, "width", config_.width);
    set_and_report(&camera_, cv::CAP_PROP_FRAME_HEIGHT, "height", config_.height);
    set_and_report(&camera_, cv::CAP_PROP_FPS, "fps", config_.fps);
    // V4L2 常用 0.75 表示自动曝光、0.25 表示手动曝光。
    set_and_report(&camera_, cv::CAP_PROP_AUTO_EXPOSURE, "auto_exposure",
                   config_.auto_exposure ? 0.75 : 0.25);
    set_and_report(&camera_, cv::CAP_PROP_EXPOSURE, "exposure_us",
                   config_.exposure_us);
    set_and_report(&camera_, cv::CAP_PROP_GAIN, "gain", config_.gain);
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
    // 在所有图像转换之前记录取帧时间，采集 FPS 不包含后续预处理耗时。
    frame->timestamp_ns = steady_timestamp_ns();
    return true;
}

bool OpenCvCameraSource::prepare(runtime::CameraFrame* frame) {
    if (frame == nullptr || frame->image.empty()) {
        return false;
    }
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
    return frame->image.type() == CV_8UC3;
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
