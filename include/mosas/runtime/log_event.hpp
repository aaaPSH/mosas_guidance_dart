#ifndef MOSAS_RUNTIME_LOG_EVENT_HPP
#define MOSAS_RUNTIME_LOG_EVENT_HPP

#include <array>
#include <cstdint>

namespace mosas::runtime {

enum class RuntimeLogEventKind {
    guidance_command,
    diagnostic,
    text,
};

// Fixed-size payload shared by producers and asynchronous logger adapters.
struct RuntimeLogEvent {
    RuntimeLogEventKind kind = RuntimeLogEventKind::diagnostic;
    std::int64_t timestamp_ns = -1;
    std::uint64_t count = 0;
    double value0 = 0.0;
    double value1 = 0.0;
    double value2 = 0.0;
    std::array<char, 192> text{};
};

}  // namespace mosas::runtime

#endif  // MOSAS_RUNTIME_LOG_EVENT_HPP
