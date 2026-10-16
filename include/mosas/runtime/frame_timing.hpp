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

// 将 SDK timeval32 按明确的时钟域转换为 CLOCK_MONOTONIC 纳秒。
std::optional<TimestampNs> convert_camera_timeval(
    std::int64_t seconds, std::int64_t microseconds,
    CameraTimestampDomain domain, const CameraClockMapping& mapping,
    TimestampNs bias_ns) noexcept;

// 检查采集时间是否仍在当前单调时间窗口内且相对历史递增。
bool validate_camera_timestamp(TimestampNs timestamp_ns,
                               TimestampNs received_timestamp_ns,
                               TimestampNs previous_timestamp_ns,
                               TimestampNs max_future_ns,
                               TimestampNs max_age_ns) noexcept;

}  // namespace mosas::runtime

#endif  // MOSAS_RUNTIME_FRAME_TIMING_HPP
