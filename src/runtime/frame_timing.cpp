#include <mosas/runtime/frame_timing.hpp>

#include <limits>

namespace mosas::runtime {
namespace {

std::optional<TimestampNs> checked_add(TimestampNs left,
                                       TimestampNs right) noexcept {
    if (right > 0 &&
        left > std::numeric_limits<TimestampNs>::max() - right) {
        return std::nullopt;
    }
    if (right < 0 &&
        left < std::numeric_limits<TimestampNs>::min() - right) {
        return std::nullopt;
    }
    return left + right;
}

}  // namespace

std::optional<TimestampNs> convert_camera_timeval(
    std::int64_t seconds, std::int64_t microseconds,
    CameraTimestampDomain domain, const CameraClockMapping& mapping,
    TimestampNs bias_ns) noexcept {
    const auto source_ns = timeval_to_nanoseconds(seconds, microseconds);
    if (!source_ns.has_value()) {
        return std::nullopt;
    }

    std::optional<TimestampNs> monotonic_ns;
    switch (domain) {
        case CameraTimestampDomain::monotonic:
            monotonic_ns = source_ns;
            break;
        case CameraTimestampDomain::realtime: {
            monotonic_ns = realtime_to_monotonic(
                *source_ns, mapping.realtime_now_ns,
                mapping.monotonic_now_ns);
            break;
        }
        case CameraTimestampDomain::unknown:
            return std::nullopt;
    }
    if (!monotonic_ns.has_value()) {
        return std::nullopt;
    }
    return checked_add(*monotonic_ns, bias_ns);
}

bool validate_camera_timestamp(TimestampNs timestamp_ns,
                               TimestampNs received_timestamp_ns,
                               TimestampNs previous_timestamp_ns,
                               TimestampNs max_future_ns,
                               TimestampNs max_age_ns) noexcept {
    if (timestamp_ns < 0 || received_timestamp_ns < 0 ||
        max_future_ns < 0 || max_age_ns < 0 ||
        (previous_timestamp_ns >= 0 &&
         timestamp_ns <= previous_timestamp_ns)) {
        return false;
    }
    if (timestamp_ns > received_timestamp_ns &&
        timestamp_ns - received_timestamp_ns > max_future_ns) {
        return false;
    }
    if (received_timestamp_ns > timestamp_ns &&
        received_timestamp_ns - timestamp_ns > max_age_ns) {
        return false;
    }
    return true;
}

}  // namespace mosas::runtime
