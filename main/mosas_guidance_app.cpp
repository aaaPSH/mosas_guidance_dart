#include <mosas/adapters/host_adapters.hpp>
#include <mosas/app/device_application.hpp>

#include <chrono>
#include <exception>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <thread>

namespace {

/**
 * @brief 选择场上运行的生产命令输出端；标定入口不调用此工厂。
 * @param config 命令模式及串口设置。
 * @param port 初始化阶段打开的共享串口。
 * @return 配置要求的命令输出端。
 * @throws std::invalid_argument 命令模式未知时抛出。
 */
std::unique_ptr<mosas::runtime::CommandSink> make_command_sink(
    const mosas::app::AppConfig& config,
    const std::shared_ptr<serial_package::SerialPort>& port) {
    if (config.command.mode == mosas::app::CommandMode::log) {
        return std::make_unique<mosas::app::LoggingCommandSink>();
    }
    if (config.command.mode == mosas::app::CommandMode::serial) {
        return std::make_unique<mosas::app::SerialCommandSink>(port, config.command, std::cerr);
    }
    throw std::invalid_argument("未知的指令输出模式");
}

}  // namespace

/**
 * @brief 按生产命令配置运行场上制导，直到收到停止信号或发生故障。
 * @return 正常退出返回 0，启动或运行失败返回 1，配置错误返回 2。
 * @note 无命令行参数；源码版读取 config/mosas_guidance.conf，安装版读取安装配置目录中的同名文件。
 */
int main() {
    mosas::app::AppConfig config;
    const int configuration_result =
        mosas::app::load_default_device_configuration("mosas_guidance.conf", config);
    if (configuration_result != 0) return configuration_result;

    mosas::app::DeviceApplicationComponents components;
    mosas::app::install_device_signal_handlers();
    try {
        const int source_result = mosas::app::initialize_device_sources(config, components);
        if (source_result != 0) return source_result;
        const int start_result = mosas::app::start_device_runtime(
            config, components, make_command_sink(config, components.serial_port));
        if (start_result != 0) {
            components.runtime->stop();
            return start_result;
        }
        mosas::app::print_device_startup(components);
        while (!mosas::app::device_stop_requested() && !components.runtime->faulted()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        return mosas::app::stop_and_report_device_runtime(components, config);
    } catch (const std::exception& error) {
        std::cerr << "程序启动/运行失败: " << error.what() << '\n';
        if (components.runtime) components.runtime->stop();
        return 1;
    }
}
