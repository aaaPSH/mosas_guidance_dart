#ifndef MOSAS_RUNTIME_CLOCK_HPP
#define MOSAS_RUNTIME_CLOCK_HPP

#include <cstdint>
#include <optional>

namespace mosas::runtime {

using TimestampNs = std::int64_t;

TimestampNs monotonic_timestamp_ns() noexcept;
TimestampNs realtime_timestamp_ns() noexcept;

std::optional<TimestampNs> timeval_to_nanoseconds(
    std::int64_t seconds, std::int64_t microseconds) noexcept;

std::optional<TimestampNs> realtime_to_monotonic(
    TimestampNs source_ns, TimestampNs source_now_ns,
    TimestampNs target_now_ns) noexcept;

}  // namespace mosas::runtime

#endif  // MOSAS_RUNTIME_CLOCK_HPP
