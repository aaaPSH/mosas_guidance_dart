#ifndef MOSAS_RUNTIME_IMU_ANGULAR_VELOCITY_HPP
#define MOSAS_RUNTIME_IMU_ANGULAR_VELOCITY_HPP

#include <mosas/runtime/clock.hpp>
#include <mosas/runtime/vector3.hpp>

namespace mosas::runtime {

struct ImuAngularVelocitySample {
    TimestampNs timestamp_ns = -1;
    // 加速度单位为 m/s²，角速度单位为 rad/s。
    Vector3 acceleration{0.0, 0.0, 0.0};
    Vector3 angular_velocity{0.0, 0.0, 0.0};
    // 舵机 P/Y 控制参数，范围 [-1, 1]。
    double servo_control_p_normalized = 0.0;
    double servo_control_y_normalized = 0.0;
};

}  // namespace mosas::runtime

#endif  // MOSAS_RUNTIME_IMU_ANGULAR_VELOCITY_HPP
