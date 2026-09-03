#ifndef MOSAS_APP_HOST_ADAPTERS_HPP
#define MOSAS_APP_HOST_ADAPTERS_HPP

#include <mosas/app/app_config.hpp>

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <iosfwd>
#include <mutex>

#include <opencv2/videoio.hpp>

namespace mosas::app {

class OpenCvCameraSource final : public runtime::CameraSource {
public:
    explicit OpenCvCameraSource(CameraAppConfig config);
    ~OpenCvCameraSource() override;

    bool configure(const runtime::CameraCaptureConfig& config) override;
    bool capture(runtime::CameraFrame* frame) override;
    void cancel() noexcept override;

private:
    CameraAppConfig config_;
    cv::VideoCapture camera_;
    cv::Mat undistort_map_x_;
    cv::Mat undistort_map_y_;
    cv::Mat undistorted_frame_;
    std::mutex mutex_;
    bool cancelled_ = false;
    bool configured_ = false;
};

class SimulatedImuSource final : public runtime::ImuSource {
public:
    explicit SimulatedImuSource(double sample_rate_hz);

    bool read(runtime::ImuSample* sample) override;
    void cancel() noexcept override;

private:
    std::chrono::nanoseconds period_;
    std::mutex mutex_;
    std::condition_variable condition_;
    std::chrono::steady_clock::time_point next_deadline_{};
    bool started_ = false;
    bool cancelled_ = false;
};

class LoggingCommandSink final : public runtime::CommandSink {
public:
    LoggingCommandSink();
    explicit LoggingCommandSink(std::ostream& output);

    bool send(const runtime::GuidanceCommand& command) override;

private:
    std::ostream* output_;
    std::mutex mutex_;
};

}  // namespace mosas::app

#endif  // MOSAS_APP_HOST_ADAPTERS_HPP
