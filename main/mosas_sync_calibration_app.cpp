#include <mosas/app/camera_imu_sync_sources.hpp>
#include <mosas/app/device_application.hpp>

#include <chrono>
#include <exception>
#include <iostream>
#include <memory>
#include <thread>
#include <utility>

namespace {

/**
 * @brief 在指定采集时长内监控原运行时及记录器健康状态。
 * @param components 已启动的设备运行时。
 * @param session 正在记录的标定会话。
 * @param seconds 从运行时成功启动后计算的采集时长，单位秒。
 * @return 停止原因；只有时长正常届满才可能生成完整数据。
 */
mosas::app::CameraImuSyncEnd monitor_capture(
    const mosas::app::DeviceApplicationComponents& components,
    const mosas::app::CameraImuSyncSession& session, double seconds) {
    mosas::app::print_device_startup(components);
    std::cout << "[标定] 先静止至少 3 秒，再变速多轴转动整机；棋盘格静止。\n"
              << "[标定] 本次台架场景不保证执行发射后的全部分支；等待定时结束。\n";
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
    while (!mosas::app::device_stop_requested() && !components.runtime->faulted()) {
        if (session.error() != mosas::app::CameraImuSyncError::none ||
            std::chrono::steady_clock::now() >= deadline) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (mosas::app::device_stop_requested()) return mosas::app::CameraImuSyncEnd::interrupted;
    if (components.runtime->faulted() || session.error() != mosas::app::CameraImuSyncError::none) {
        return mosas::app::CameraImuSyncEnd::runtime_fault;
    }
    return mosas::app::CameraImuSyncEnd::duration_elapsed;
}

/**
 * @brief 在生产者退出后收集最终运行时及设备诊断。
 * @param components 已停止的组件，相机及串口 IMU 观察指针仍须有效；串口模式由标定预检保证。
 * @return 包含原流水线统计及 SDK 参数读回值的记录摘要。
 */
mosas::app::CameraImuSyncSummary capture_summary(
    const mosas::app::DeviceApplicationComponents& components) {
    mosas::app::CameraImuSyncSummary summary;
    summary.timing = components.runtime->timing();
    summary.fps = components.runtime->statistics();
    summary.imu = components.serial_imu_source->diagnostics();
    const auto camera = components.camera_source_status->camera_timing();
    const auto sdk = components.camera_source_status->sdk_configuration();
    summary.camera_callback_wait = camera.callback_wait;
    summary.camera_queue_delay = camera.callback_queue_delay;
    summary.camera_decode = camera.decode_convert;
    summary.camera_queue_dropped = camera.callback_queue_dropped;
    summary.camera_timestamp_dropped = camera.timestamp_dropped;
    summary.sdk_width = sdk.width;
    summary.sdk_height = sdk.height;
    summary.sdk_fps = sdk.fps;
    summary.exposure_us = sdk.exposure_us;
    summary.gain = sdk.gain;
    return summary;
}

/**
 * @brief 等待已启动的生产者退出，再将启动失败的标定会话标记为不完整。
 * @param components 可能仅部分初始化的设备组件。
 * @param session 待关闭的标定会话，可为空。
 */
void finish_failed_start(mosas::app::DeviceApplicationComponents& components,
                         const std::shared_ptr<mosas::app::CameraImuSyncSession>& session) {
    if (components.runtime) components.runtime->stop();
    if (session) session->finish(mosas::app::CameraImuSyncEnd::startup_failure, {});
}

}  // namespace

/**
 * @brief 在原运行时负载下记录相机与 IMU 标定数据，始终不执行命令输出 I/O。
 * @return 完整采集返回 0，启动、运行或记录失败返回 1，配置错误返回 2。
 * @note 无命令行参数；源码版读取 config/mosas_sync_calibration.conf，安装版读取安装配置目录中的同名文件。
 */
int main() {
    mosas::app::AppConfig config;
    const int configuration_result =
        mosas::app::load_default_device_configuration("mosas_sync_calibration.conf", config);
    if (configuration_result != 0) return configuration_result;
    const auto& options = config.sync_calibration;
    try {
        mosas::app::validate_camera_imu_sync_options(options, config);
    } catch (const std::exception& error) {
        std::cerr << "标定配置错误: " << error.what() << '\n';
        return 2;
    }

    // 保证运行时及其记录装饰器先析构，记录会话最后释放。
    std::shared_ptr<mosas::app::CameraImuSyncSession> session;
    mosas::app::DeviceApplicationComponents components;
    mosas::app::install_device_signal_handlers();
    try {
        session = std::make_shared<mosas::app::CameraImuSyncSession>(options, config);
        session->start();
        std::cout << "[标定] 控制输出已强制禁用；请确保执行器物理断能，停发不保证下位机清除旧指令。\n"
                  << "[标定] 仅使用原程序实际负载，无人工压力；无串口 TX 负载，记录另有开销。\n";
        const int source_result = mosas::app::initialize_device_sources(config, components);
        if (source_result != 0) {
            finish_failed_start(components, session);
            return source_result;
        }
        // 标定入口只构造无设备资源的命令接收端，不调用生产命令工厂。
        auto ports = mosas::app::make_camera_imu_sync_ports(
            std::move(components.camera_source), std::move(components.imu_source), session);
        components.camera_source = std::move(ports.camera);
        components.imu_source = std::move(ports.imu);
        const int start_result = mosas::app::start_device_runtime(config, components, std::move(ports.command));
        if (start_result != 0) {
            finish_failed_start(components, session);
            return start_result;
        }
        const auto end = monitor_capture(components, *session, options.seconds);
        const int runtime_result = mosas::app::stop_and_report_device_runtime(components, config);
        const bool complete = session->finish(
            runtime_result != 0 ? mosas::app::CameraImuSyncEnd::runtime_fault : end,
            capture_summary(components));
        std::cout << "[标定] " << (complete ? "采集完整: " : "采集无效/未完成: ")
                  << options.directory << "，recording_error="
                  << mosas::app::camera_imu_sync_error_name(session->error()) << '\n';
        return complete ? runtime_result : 1;
    } catch (const std::exception& error) {
        std::cerr << "标定启动/记录失败: " << error.what() << '\n';
        finish_failed_start(components, session);
        return 1;
    }
}
