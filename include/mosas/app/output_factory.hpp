#ifndef MOSAS_APP_OUTPUT_FACTORY_HPP
#define MOSAS_APP_OUTPUT_FACTORY_HPP

#include <mosas/app/app_config.hpp>
#include <mosas/runtime/output_ports.hpp>

#include <iosfwd>
#include <memory>

namespace mosas::app {

/**
 * @brief 根据 output.mode 创建该次运行的录像、遥测、可视化、IMU 记录和日志端口。
 * @param config 已解析并校验的应用配置。
 * @param log_output 异步日志 worker 使用的输出流引用；其生命周期须覆盖输出端口。
 * @param visualization_override 可选的 RTSP 模式替代 sink，常用于测试或自定义组装。
 * @return 由 RuntimeSupervisor 接管的输出端口集合。
 * @throws std::invalid_argument 输出模式不可用，或本地录像模式未配置必需路径。
 * @note 此函数只组装组件；RTSP 网络启动延后到首帧发布，函数本身不启动 worker。
 */
runtime::RuntimeOutputPorts make_runtime_output_ports(
    const AppConfig& config, std::ostream& log_output,
    std::unique_ptr<runtime::IVisualizationSink> visualization_override =
        nullptr);

}  // namespace mosas::app

#endif  // MOSAS_APP_OUTPUT_FACTORY_HPP
