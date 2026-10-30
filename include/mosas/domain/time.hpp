#ifndef MOSAS_DOMAIN_TIME_HPP
#define MOSAS_DOMAIN_TIME_HPP

#include <cstdint>

namespace mosas::domain {

enum class ClockDomain { unknown, monotonic, realtime, device, simulation };
enum class TimestampSource {
    unknown,
    camera_sdk_capture,
    host_callback_receive,
    imu_uart_receive_estimate,
    sequence_fixed_period,
    hardware_sync,
    simulation,
};
enum class TimestampQuality {
    unknown,
    estimated,
    calibrated,
    hardware_synchronized,
    simulation,
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

/** @brief Unconverted session-local simulation acquisition time; never hardware synchronization.
 * @param timestamp_ns Simulation nanoseconds; a negative value remains invalid.
 * @return Explicit simulation source/domain/quality metadata with zero conversion uncertainty.
 */
inline SensorTime make_simulation_time(std::int64_t timestamp_ns) noexcept {
    return {timestamp_ns, timestamp_ns, 0, ClockDomain::simulation,
            TimestampSource::simulation, TimestampQuality::simulation};
}

}  // namespace mosas::domain

#endif  // MOSAS_DOMAIN_TIME_HPP
