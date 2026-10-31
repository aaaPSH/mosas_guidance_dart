#ifndef MOSAS_RUNTIME_FRAME_TIMING_HPP
#define MOSAS_RUNTIME_FRAME_TIMING_HPP

#include <mosas/runtime/clock.hpp>

#include <cstdint>
#include <optional>

namespace mosas::runtime {

enum class CameraTimestampDomain {
    unknown,
    monotonic,
    realtime,
};

enum class CameraTimestampSource {
    unknown,
    sdk_monotonic,
    sdk_realtime_converted,
    simulation,
};

struct CameraClockMapping {
    TimestampNs realtime_now_ns = -1;
    TimestampNs monotonic_now_ns = -1;
};

/** @brief Apply a fixed camera bias without wrapping, clamping or admitting negative timestamps.
 * @param timestamp_ns Nonnegative, uncompensated monotonic timestamp.
 * @param bias_ns Signed fixed compensation; applied exactly once.
 * @return Compensated timestamp, or no value if either time is invalid/unrepresentable.
 */
std::optional<TimestampNs> apply_camera_timestamp_bias(TimestampNs timestamp_ns,
                                                     TimestampNs bias_ns) noexcept;

/** @brief Convert an SDK timeval into the camera's compensated monotonic domain.
 * @param seconds SDK timestamp seconds. @param microseconds SDK fractional microseconds.
 * @param domain Explicit SDK clock domain. @param mapping Realtime-to-monotonic mapping.
 * @param bias_ns Fixed compensation; zero requests the uncompensated SDK evidence.
 * @return Nonnegative converted timestamp, or no value for invalid time, domain or arithmetic.
 */
std::optional<TimestampNs> convert_camera_timeval(
    std::int64_t seconds, std::int64_t microseconds,
    CameraTimestampDomain domain, const CameraClockMapping& mapping,
    TimestampNs bias_ns) noexcept;

/** @brief Validate published capture time against the original, uncompensated host reception.
 * @param timestamp_ns Published capture time, including camera bias.
 * @param received_timestamp_ns Original host callback time; never shift this safety boundary.
 * @param previous_timestamp_ns Previous capture time, or negative if unavailable.
 * @param max_future_ns Maximum allowed future lead. @param max_age_ns Maximum allowed age.
 * @return True for nonnegative, increasing timestamps within both configured bounds.
 */
bool validate_camera_timestamp(TimestampNs timestamp_ns,
                               TimestampNs received_timestamp_ns,
                               TimestampNs previous_timestamp_ns,
                               TimestampNs max_future_ns,
                               TimestampNs max_age_ns) noexcept;

}  // namespace mosas::runtime

#endif  // MOSAS_RUNTIME_FRAME_TIMING_HPP
