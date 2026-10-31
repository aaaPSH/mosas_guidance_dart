#include <mosas/runtime/clock.hpp>

#include <ctime>
#include <limits>

namespace mosas::runtime {
namespace {

std::optional<TimestampNs> checked_add(TimestampNs left,
                                       TimestampNs right) noexcept {
    if ((right > 0 &&
         left > std::numeric_limits<TimestampNs>::max() - right) ||
        (right < 0 &&
         left < std::numeric_limits<TimestampNs>::min() - right)) {
        return std::nullopt;
    }
    return left + right;
}

std::optional<TimestampNs> checked_subtract(TimestampNs left,
                                            TimestampNs right) noexcept {
    if ((right < 0 &&
         left > std::numeric_limits<TimestampNs>::max() + right) ||
        (right > 0 &&
         left < std::numeric_limits<TimestampNs>::min() + right)) {
        return std::nullopt;
    }
    return left - right;
}

TimestampNs read_clock(clockid_t clock_id) noexcept {
    timespec time{};
    if (clock_gettime(clock_id, &time) != 0 || time.tv_sec < 0 ||
        time.tv_nsec < 0 || time.tv_nsec >= 1'000'000'000) {
        return -1;
    }

    if (time.tv_sec >
        std::numeric_limits<TimestampNs>::max() / 1'000'000'000) {
        return -1;
    }
    const TimestampNs seconds_ns =
        static_cast<TimestampNs>(time.tv_sec) * 1'000'000'000;
    return checked_add(seconds_ns, static_cast<TimestampNs>(time.tv_nsec))
        .value_or(-1);
}

}  // namespace

TimestampNs monotonic_timestamp_ns() noexcept {
    return read_clock(CLOCK_MONOTONIC);
}

TimestampNs realtime_timestamp_ns() noexcept {
    return read_clock(CLOCK_REALTIME);
}

TimestampNs SystemMonotonicClock::now_ns() const noexcept {
    return monotonic_timestamp_ns();
}

std::optional<TimestampNs> timeval_to_nanoseconds(
    std::int64_t seconds, std::int64_t microseconds) noexcept {
    if (seconds < 0 || microseconds < 0 || microseconds >= 1'000'000 ||
        seconds > std::numeric_limits<TimestampNs>::max() / 1'000'000'000) {
        return std::nullopt;
    }

    const TimestampNs seconds_ns = seconds * 1'000'000'000;
    const TimestampNs microseconds_ns = microseconds * 1'000;
    return checked_add(seconds_ns, microseconds_ns);
}

std::optional<TimestampNs> realtime_to_monotonic(
    TimestampNs source_ns, TimestampNs source_now_ns,
    TimestampNs target_now_ns) noexcept {
    if (source_ns < 0 || source_now_ns < 0 || target_now_ns < 0) {
        return std::nullopt;
    }

    const auto offset = checked_subtract(source_ns, source_now_ns);
    if (!offset.has_value()) {
        return std::nullopt;
    }
    return checked_add(target_now_ns, offset.value());
}

}  // namespace mosas::runtime
