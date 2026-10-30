#ifndef MOSAS_APP_DEVICE_APPLICATION_HPP
#define MOSAS_APP_DEVICE_APPLICATION_HPP

#include <mosas/adapters/nori_camera_source.hpp>
#include <mosas/adapters/serial_imu_source.hpp>
#include <mosas/app/app_config.hpp>
#include <mosas/runtime/runtime_supervisor.hpp>
#include <serial_package/serial_port.hpp>

#include <memory>
#include <string>

namespace mosas::app {

/**
 * @brief 持有设备源和运行时；逆序析构保证共享串口最后释放。
 * @note 启动时数据源转交给运行时，观察指针在运行时析构前有效。
 *       命令输出端由各入口显式选择，公共设备支持不决定是否允许控制输出。
 */
struct DeviceApplicationComponents {
    /** @brief Stop producers and report timing even on early return or exception unwinding. */
    ~DeviceApplicationComponents() noexcept;
    /** @brief Synchronously emit final timing diagnostics once; call only after stopping producers. */
    void report_acquisition_time() noexcept;
    bool acquisition_time_reported = false;
    std::shared_ptr<serial_package::SerialPort> serial_port;
    SerialImuSource* serial_imu_source = nullptr;
    NoriSdkCameraSource* camera_source_status = nullptr;
    std::unique_ptr<runtime::ImuSource> imu_source;
    std::unique_ptr<runtime::CameraSource> camera_source;
    std::unique_ptr<runtime::RuntimeSupervisor> runtime;
};

/**
 * @brief 加载指定应用配置，并在打开设备前拒绝当前构建不支持的输出模式。
 * @param path 配置文件路径。
 * @param config 接收校验后的配置。
 * @return 成功返回 0，配置错误返回 2，并向标准错误流报告原因。
 */
int load_device_configuration(const std::string& path, AppConfig& config);

/**
 * @brief 为无参数入口定位并加载默认配置；源码启动与安装启动使用各自的目录约定。
 * @param filename 固定的配置文件名，不允许包含目录分量。
 * @param config 接收校验后的配置。
 * @return 成功返回 0，定位或配置错误返回 2；不打开设备。
 * @note Linux 安装版通过 /proc/self/exe 定位 CMake 配置的安装目录，缺失时不回退到当前目录。
 *       源码构建版从当前工作目录的 config/ 加载；启动脚本负责切换到项目根目录。
 */
int load_default_device_configuration(const std::string& filename, AppConfig& config);

/** @brief 在启动设备或记录线程之前安装 SIGINT/SIGTERM 停止信号处理器。 */
void install_device_signal_handlers();

/** @brief 查询进程停止标志。 @return 收到 SIGINT 或 SIGTERM 后返回 true。 */
bool device_stop_requested() noexcept;

/**
 * @brief 打开唯一的共享串口并创建原 IMU、相机源，不构造命令输出端。
 * @param config 已校验的应用配置。
 * @param components 接收设备所有权，调用前应为空。
 * @return 成功返回 0，串口失败返回 1，未知 IMU 模式返回 2。
 * @note 调用前须安装信号处理器；调用方可在启动运行时前装饰数据源。
 */
int initialize_device_sources(const AppConfig& config, DeviceApplicationComponents& components);

/**
 * @brief 组装原算法及配置输出，转移数据源所有权并启动运行时。
 * @param config 已校验的应用配置。
 * @param components 持有唯一的数据源，成功组装后由其 runtime 接管。
 * @param command_sink 由入口显式选择的非空命令输出端。
 * @return 成功返回 0，启动失败返回 1；完成标定文件之前必须先停止运行时。
 */
int start_device_runtime(const AppConfig& config, DeviceApplicationComponents& components,
                         std::unique_ptr<runtime::CommandSink> command_sink);

/**
 * @brief 输出一次启动状态及相机 SDK 参数读回结果。
 * @param components 已成功启动且相机观察指针有效的组件。
 */
void print_device_startup(const DeviceApplicationComponents& components);

/**
 * @brief 停止并等待生产者退出，报告故障、输出健康状态和可选统计。
 * @param components 已启动的组件；数据源在退出统计期间保持存活。
 * @param config 决定是否输出串口间隔及各阶段耗时。
 * @return 运行时故障返回 1，否则返回 0；不完成入口专属的标定记录。
 */
int stop_and_report_device_runtime(DeviceApplicationComponents& components,
                                   const AppConfig& config);

}  // namespace mosas::app
#endif
