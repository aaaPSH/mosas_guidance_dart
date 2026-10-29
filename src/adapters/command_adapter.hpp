#ifndef MOSAS_ADAPTERS_COMMAND_ADAPTER_HPP
#define MOSAS_ADAPTERS_COMMAND_ADAPTER_HPP

#include <mosas/domain/command_output.hpp>
#include <serial_package/serial_protocol.hpp>

namespace mosas::app::detail {

/**
 * @brief 将镖体系制导命令映射为串口协议的状态标志和 yaw/pitch 字段。
 * @param command 当前命令；无效命令仅将过载置零，不清除发射标志和识别位。
 * @return 协议字段值，状态标志原样保留，body-z 映射到 yaw，body-y 映射到 pitch。
 * @note 过载分量单位为 g；字段顺序和符号映射属于下位机协议契约。
 */
serial_package::ControlFrameValues make_control_frame_values(
    const domain::MappedGuidanceCommand& command) noexcept;

}  // namespace mosas::app::detail

#endif  // MOSAS_ADAPTERS_COMMAND_ADAPTER_HPP
