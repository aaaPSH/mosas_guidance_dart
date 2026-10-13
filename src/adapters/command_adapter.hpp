#ifndef MOSAS_ADAPTERS_COMMAND_ADAPTER_HPP
#define MOSAS_ADAPTERS_COMMAND_ADAPTER_HPP

#include <mosas/runtime/runtime_types.hpp>
#include <serial_package/serial_protocol.hpp>

namespace mosas::app::detail {

/**
 * @brief 将镖体系制导命令映射为串口协议的 roll/yaw/pitch 字段。
 * @param command 当前命令；无效命令的输出字段全部置零。
 * @return 协议字段值，body-z 映射到 yaw，body-y 映射到 pitch，roll 固定为零。
 * @note 过载分量单位为 g；字段顺序和符号映射属于下位机协议契约。
 */
serial_package::ControlFrameValues make_control_frame_values(
    const runtime::GuidanceCommand& command) noexcept;

}  // namespace mosas::app::detail

#endif  // MOSAS_ADAPTERS_COMMAND_ADAPTER_HPP
