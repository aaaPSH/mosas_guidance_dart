#ifndef MOSAS_RUNTIME_TYPES_HPP
#define MOSAS_RUNTIME_TYPES_HPP

#include <mosas/guidance/guidance_estimator.hpp>
#include <mosas/runtime/frame_timing.hpp>
#include <mosas/runtime/vector3.hpp>

#include <cstdint>

#include <opencv2/core/mat.hpp>

namespace mosas::runtime {

using TimestampNs = std::int64_t;

// 旧式同步 IMU 接口兼容类型；当前运行时主路径只使用角速度样本。
struct ImuSample {
    TimestampNs timestamp_ns;
    Vector3 acceleration;
    Vector3 angular_velocity;
};

struct CameraFrame {
    TimestampNs timestamp_ns = -1;
    TimestampNs received_timestamp_ns = -1;
    CameraTimestampSource timestamp_source = CameraTimestampSource::unknown;
    cv::Mat image;
};

struct ImuStateSnapshot {
    TimestampNs timestamp_ns = -1;
    Vector3 angular_velocity{0.0, 0.0, 0.0};
};

struct GuidanceCommand {
    TimestampNs timestamp_ns;
    PngGuidanceOutput output;
};

}  // namespace mosas::runtime

#endif  // MOSAS_RUNTIME_TYPES_HPP
