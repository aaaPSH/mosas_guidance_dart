#ifndef MOSAS_APP_CONFIG_HPP
#define MOSAS_APP_CONFIG_HPP

#include <mosas/runtime/guidance_runtime.hpp>
#include <mosas/wireless/rtsp_frame_sink.hpp>

#include <filesystem>
#include <string>

namespace mosas::app {

struct CameraMatrix3 {
    double m00 = 1.0;
    double m01 = 0.0;
    double m02 = 0.0;
    double m10 = 0.0;
    double m11 = 1.0;
    double m12 = 0.0;
    double m20 = 0.0;
    double m21 = 0.0;
    double m22 = 1.0;
};

struct DistortionCoefficients {
    double k1 = 0.0;
    double k2 = 0.0;
    double p1 = 0.0;
    double p2 = 0.0;
    double k3 = 0.0;
};

struct CameraAppConfig {
    std::string device = "/dev/video0";
    int width = 640;
    int height = 480;
    int fps = 30;
    // 摄像头像素格式；auto 表示不强制格式，其他值必须是四字符 FOURCC。
    std::string pixel_format = "MJPG";
    bool auto_exposure = true;
    double exposure_us = 10000.0;
    double gain = 0.0;
    bool undistort = false;
    CameraMatrix3 camera_matrix{};
    DistortionCoefficients distortion{};
    bool camera_matrix_configured = false;
    runtime::CameraCaptureMode capture_mode =
        runtime::CameraCaptureMode::free_run;
};

enum class ImuMode {
    simulated,
};

struct ImuAppConfig {
    ImuMode mode = ImuMode::simulated;
    double sample_rate_hz = 200.0;
    // 测试模式：使用假基准跳过 IMU 静止自检。
    bool skip_self_check = false;
};

enum class CommandMode {
    log,
};

struct CommandAppConfig {
    CommandMode mode = CommandMode::log;
};

struct AppConfig {
    CameraAppConfig camera{};
    ImuAppConfig imu{};
    CommandAppConfig command{};
    runtime::GuidanceRuntimeConfig guidance{};
    wireless::RtspFrameSinkConfig wireless{};
};

class AppConfigLoader {
public:
    static bool load(const std::filesystem::path& path, AppConfig* config,
                     std::string* error);
};

}  // namespace mosas::app

#endif  // MOSAS_APP_CONFIG_HPP
