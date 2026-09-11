#ifndef MOSAS_CONTROL_COMMAND_HPP
#define MOSAS_CONTROL_COMMAND_HPP

#include <cstdint>
#include <type_traits>

namespace mosas::runtime {

// The mailbox carries continuous control setpoints only. One-shot events such
// as launch, arm, disarm, reset, and emergency must use a separate reliable
// bounded event channel when they are introduced.
inline constexpr std::uint32_t kControlCommandValid = 1u << 0;

struct ControlCommand {
    std::uint64_t sequence = 0;
    std::int64_t timestamp_ns = -1;
    float overload_y = 0.0F;
    float overload_z = 0.0F;
    std::uint32_t flags = 0;
};

static_assert(std::is_trivially_copyable_v<ControlCommand>);
static_assert(sizeof(ControlCommand) == 32);

}  // namespace mosas::runtime

#endif  // MOSAS_CONTROL_COMMAND_HPP
