#ifndef MOSAS_ADAPTERS_COMMAND_ADAPTER_HPP
#define MOSAS_ADAPTERS_COMMAND_ADAPTER_HPP

#include <mosas/runtime/runtime_types.hpp>
#include <serial_package/serial_protocol.hpp>

namespace mosas::app::detail {

serial_package::ControlFrameValues make_control_frame_values(
    const runtime::GuidanceCommand& command) noexcept;

}  // namespace mosas::app::detail

#endif  // MOSAS_ADAPTERS_COMMAND_ADAPTER_HPP
