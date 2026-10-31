#include <mosas/domain/imu_time_model.hpp>

#include <limits>

namespace {

std::int64_t saturating_subtract(std::int64_t left,
                                 std::int64_t right) noexcept {
    const auto min = std::numeric_limits<std::int64_t>::min();
    const auto max = std::numeric_limits<std::int64_t>::max();
    if (right == min) {
        return left >= 0 ? max : left - right;
    }
    if (right > 0 && left < min + right) {
        return min;
    }
    if (right < 0 && left > max + right) {
        return max;
    }
    return left - right;
}

}  // namespace

namespace mosas::domain {

SensorTime estimate_imu_sample_time(
    std::int64_t first_byte_receive_monotonic_ns,
    std::int64_t measurement_latency_ns) noexcept {
    SensorTime result{};
    result.monotonic_ns = saturating_subtract(
        first_byte_receive_monotonic_ns, measurement_latency_ns);
    result.source_time_ns = -1;
    result.uncertainty_ns = -1;
    result.source_domain = ClockDomain::monotonic;
    result.source = TimestampSource::imu_uart_receive_estimate;
    result.quality = TimestampQuality::estimated;
    return result;
}

}  // namespace mosas::domain
