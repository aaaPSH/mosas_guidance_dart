#ifndef MOSAS_RUNTIME_IMU_ANGULAR_VELOCITY_HPP
#define MOSAS_RUNTIME_IMU_ANGULAR_VELOCITY_HPP

#include <mosas/domain/time.hpp>
#include <mosas/runtime/clock.hpp>
#include <optional>
#include <mosas/runtime/vector3.hpp>

namespace mosas::runtime {

/** @brief 单个 IMU 采样及原始上行元数据；触发标志和外环值不参与姿态或控制计算。 */
struct ImuAngularVelocitySample {
    TimestampNs timestamp_ns = -1;
    // 加速度单位为 m/s²，角速度单位为 rad/s。
    Vector3 acceleration{0.0, 0.0, 0.0};
    Vector3 angular_velocity{0.0, 0.0, 0.0};
    // 舵机 P/Y 控制参数，范围 [-1, 1]。
    double servo_control_p_normalized = 0.0;
    double servo_control_y_normalized = 0.0;
    std::optional<domain::SensorTime> sensor_time = std::nullopt;
    /** @brief 采集代际及解回绕后的设备采样槽；序号差保留丢失帧占用的时间。 */
    std::uint64_t acquisition_epoch = 0;
    std::uint64_t sample_sequence = 0;
    double outer_loop_y_rad_s = 0.0;
    double outer_loop_p_rad_s = 0.0;
    /** @brief 原始设备帧号；非串口数据源未提供时为空。 */
    std::optional<std::uint32_t> imu_index = std::nullopt;
    std::uint8_t trigger_flag = 0;
};

}  // namespace mosas::runtime

#endif  // MOSAS_RUNTIME_IMU_ANGULAR_VELOCITY_HPP
