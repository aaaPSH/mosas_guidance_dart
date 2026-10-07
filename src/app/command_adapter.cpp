#include <mosas/app/command_adapter.hpp>

namespace mosas::app::detail {

serial_package::ControlFrameValues make_control_frame_values(
    const runtime::GuidanceCommand& command) noexcept {
    return {0.0, command.valid ? command.overload_g.z : 0.0,
            command.valid ? command.overload_g.y : 0.0};
}

}  // namespace mosas::app::detail
