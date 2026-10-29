#ifndef MOSAS_APP_CONFIG_HPP
#define MOSAS_APP_CONFIG_HPP

#include <mosas/guidance/ballistic_png_guidance.hpp>
#include <mosas/guidance/command_output_mapper.hpp>
#include <mosas/vision/vision_types.hpp>
#include <mosas/guidance/inertial_vector_los_filter.hpp>
#include <mosas/runtime/attitude_estimator.hpp>
#include <mosas/runtime/runtime_supervisor.hpp>
#include <mosas/output/output_config.hpp>
#include <mosas/wireless/rtsp_streamer.hpp>
#include <serial_package/serial_port.hpp>

#include <filesystem>
#include <cstddef>
#include <cstdint>
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

struct CameraAppConfig {
    std::string device = "/dev/video0";
    // Nori SDK 设备索引；示例程序默认使用 0。
    uint32_t device_id = 0;
    int width = 640;
    int height = 480;
    int fps = 30;
    // 摄像头像素格式；支持 MJPEG（SDK 映射为 MJPG）、YUYV 和 auto。
    std::string pixel_format = "MJPEG";
    bool auto_exposure = true;
    double exposure_us = 10000.0;
    double gain = 0.0;
    // SDK 回调与处理线程之间的帧缓存容量；队列满时丢弃最旧帧。
    std::size_t callback_queue_capacity = 2;
    // Frame_Time 的时钟域必须由设备/驱动实测确认；默认按显式配置的 realtime 处理。
    runtime::CameraTimestampDomain frame_timestamp_domain =
        runtime::CameraTimestampDomain::realtime;
    // 对 SDK 采集时刻到曝光参考时刻的固定修正，单位纳秒。
    runtime::TimestampNs camera_timestamp_bias_ns = 0;
    CameraMatrix3 camera_matrix{};
    bool camera_matrix_configured = false;
    runtime::CameraCaptureMode capture_mode =
        runtime::CameraCaptureMode::free_run;
};

enum class ImuMode {
    simulated,
    serial,
};

struct ImuAppConfig {
    ImuMode mode = ImuMode::simulated;
    // IMU 实际发送频率，仅用于兼容配置校验；帧时间戳来自串口接收时间轴。
    double sample_rate_hz = 200.0;
    // 下位机测量时刻相对串口帧时间的固定延迟，单位纳秒；需由硬件实测。
    runtime::TimestampNs measurement_latency_ns = 0;
    // 串口原始接收线程的调度策略；实时优先级申请失败时是否拒绝启动。
    serial_package::ReceiveThreadConfig receive_thread{256, 20, false};
};

enum class CommandMode {
    log,
    serial,
};

/** @brief 命令输出模式及串口周期发送参数；日志输出仍按新结果记录。 */
struct CommandAppConfig {
    CommandMode mode = CommandMode::log;
    /** @brief 串口发送频率，单位 Hz，允许 1～1000；不改变图像处理频率。 */
    int send_rate_hz = 100;
    /** @brief 新命令提交后可重复发送的最长时间，单位 ms；必须大于 0。 */
    int command_timeout_ms = 100;
};

/** @brief 唯一惯性矢量制导及发送端输出配置。 */
struct GuidanceAlgorithmConfig {
    VisionConfig vision_config{
        {0, 0, 320, 240}, {35, 85, 80, 255, 100, 255}, 40, 40, 8, 0.5,
        0.35};
    guidance::InertialVectorLosFilterConfig inertial_filter{};
    guidance::BallisticPngConfig ballistic_png{};
    guidance::CommandMappingConfig command_mapping{};
    runtime::AttitudeEstimatorConfig attitude{};
};

/**
 * @brief 有界标定记录设置；图像抽样不改变相机采集帧率。
 * @note AppConfigLoader 按配置文件所在目录解析相对记录路径。
 */
struct CameraImuSyncOptions {
    std::filesystem::path directory;
    double seconds = 45.0;
    int fps = 30;
    std::uint64_t max_bytes = 2048ULL * 1024 * 1024;
    std::size_t image_queue_capacity = 8;
    std::size_t imu_queue_capacity = 4096;
};

struct AppConfig {
    /** @brief 退出时输出流水线及相机耗时统计；两个默认配置文件分别控制。 */
    bool enable_timing_statistics = false;
    CameraAppConfig camera{};
    ImuAppConfig imu{};
    CommandAppConfig command{};
    serial_package::SerialPortConfig serial{};
    // 是否在程序退出时输出 IMU 数据帧间隔统计摘要。
    bool enable_serial_interval_statistics = false;
    runtime::RuntimeSupervisorConfig guidance{};
    GuidanceAlgorithmConfig algorithms{};
    output::OutputConfig output{};
    wireless::RtspStreamConfig wireless{};
    /** @brief 仅由标定入口使用并执行采集前校验。 */
    CameraImuSyncOptions sync_calibration{};
};

class AppConfigLoader {
public:
    /**
     * @brief 读取、解析并校验应用配置文件。
     * @param path 配置文件路径；相对录像路径和标定目录按配置文件所在目录解析。
     * @param config 非空输出指针；成功时写入完整配置。
     * @param error 可选错误输出；失败时包含文件/行或全局校验信息。
     * @return 文件解析和全部配置校验成功时为 true。
     * @note 输出 config 仅在成功时替换；函数不启动硬件或输出组件。
     *       标定场景的设备要求、采集范围与目录存在性由标定入口额外校验。
     */
    static bool load(const std::filesystem::path& path, AppConfig* config,
                     std::string* error);
};

}  // namespace mosas::app

#endif  // MOSAS_APP_CONFIG_HPP
