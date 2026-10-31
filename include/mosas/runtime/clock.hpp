#ifndef MOSAS_RUNTIME_CLOCK_HPP
#define MOSAS_RUNTIME_CLOCK_HPP

#include <mosas/ports/clock.hpp>

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

class SystemMonotonicClock final : public ports::IMonotonicClock {
public:
    TimestampNs now_ns() const noexcept override;
    /** @return 主机单调时间轴。 */
    domain::ClockDomain clock_domain() const noexcept override { return domain::ClockDomain::monotonic; }
};

}  // namespace mosas::runtime

#endif  // MOSAS_RUNTIME_CLOCK_HPP
