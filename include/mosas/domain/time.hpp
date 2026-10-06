#ifndef MOSAS_DOMAIN_TIME_HPP
#define MOSAS_DOMAIN_TIME_HPP

#include <cstdint>

namespace mosas::domain {

enum class ClockDomain { unknown, monotonic, realtime, device };
enum class TimestampSource {
    unknown,
    camera_sdk_capture,
    host_callback_receive,
    imu_uart_receive_estimate,
    hardware_sync,
};
enum class TimestampQuality {
    unknown,
    estimated,
    calibrated,
    hardware_synchronized,
};

struct SensorTime {
    std::int64_t monotonic_ns = -1;
    std::int64_t source_time_ns = -1;
    std::int64_t uncertainty_ns = -1;
    ClockDomain source_domain = ClockDomain::unknown;
    TimestampSource source = TimestampSource::unknown;
    TimestampQuality quality = TimestampQuality::unknown;

    bool is_valid() const noexcept { return monotonic_ns >= 0; }
};

}  // namespace mosas::domain

#endif  // MOSAS_DOMAIN_TIME_HPP
