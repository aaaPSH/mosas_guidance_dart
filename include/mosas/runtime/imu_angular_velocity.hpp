#ifndef MOSAS_RUNTIME_IMU_ANGULAR_VELOCITY_HPP
#define MOSAS_RUNTIME_IMU_ANGULAR_VELOCITY_HPP

#include <mosas/runtime/clock.hpp>
#include <mosas/runtime/vector3.hpp>

namespace mosas::runtime {

struct ImuAngularVelocitySample {
    TimestampNs timestamp_ns = -1;
    Vector3 acceleration{0.0, 0.0, 0.0};
    Vector3 angular_velocity{0.0, 0.0, 0.0};
};

}  // namespace mosas::runtime

#endif  // MOSAS_RUNTIME_IMU_ANGULAR_VELOCITY_HPP
