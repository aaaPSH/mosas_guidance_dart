#ifndef MOSAS_RUNTIME_TYPES_HPP
#define MOSAS_RUNTIME_TYPES_HPP

#include <mosas/guidance/guidance_estimator.hpp>

#include <cstdint>

#include <opencv2/core/mat.hpp>

namespace mosas::runtime {

using TimestampNs = std::int64_t;

enum class FlightPhase {
    pre_launch,
    ejection,
    free_flight,
};

struct ImuSample {
    TimestampNs timestamp_ns;
    Vector3 acceleration;
    Vector3 angular_velocity;
};

struct CameraFrame {
    TimestampNs timestamp_ns;
    cv::Mat image;
};

struct ImuStateSnapshot {
    TimestampNs timestamp_ns;
    FlightPhase phase;
    EulerAngles attitude;
    Vector3 velocity;
    Vector3 acceleration;
    Vector3 angular_velocity;
};

struct GuidanceCommand {
    TimestampNs timestamp_ns;
    PngGuidanceOutput output;
};

}  // namespace mosas::runtime

#endif  // MOSAS_RUNTIME_TYPES_HPP
