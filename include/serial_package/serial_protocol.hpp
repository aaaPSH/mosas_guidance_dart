#ifndef SERIAL_PACKAGE_SERIAL_PROTOCOL_HPP
#define SERIAL_PACKAGE_SERIAL_PROTOCOL_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

namespace serial_package {

constexpr std::size_t kControlFrameSize = 14;
constexpr std::size_t kImuFrameSize = 20;

struct ControlFrameValues {
    bool launched = false;
    bool target_found = false;
    double yaw_overload_g = 0.0;
    double pitch_overload_g = 0.0;
};

struct ImuFrameValues {
    double acceleration_x_mps2 = 0.0;
    double acceleration_y_mps2 = 0.0;
    double acceleration_z_mps2 = 0.0;
    double angular_velocity_x_rad_s = 0.0;
    double angular_velocity_y_rad_s = 0.0;
    double angular_velocity_z_rad_s = 0.0;
    // 舵机 P/Y 控制参数，已从单字节 [0, 255] 归一化到 [-1, 1]。
    double servo_control_p_normalized = 0.0;
    double servo_control_y_normalized = 0.0;
    std::uint16_t crc = 0;
};

// 下行帧共 14 字节：AA 55、发射标志(1B，未发射 0 / 已发射 1)、识别位(1B，0/1)、
// Yaw(2B)、Pitch(2B)、保留(2B)、CRC(2B)、0D 0A。
// Yaw/Pitch 为小端 int16，单位 0.01 g；CRC 覆盖 bytes 2..9。
bool encode_control_frame(
    const ControlFrameValues& values,
    std::array<std::uint8_t, kControlFrameSize>* frame,
    std::string* error = nullptr);

// 计算 IMU 上行帧 bytes 2..15 的 CRC-16/CCITT-FALSE。
std::uint16_t calculate_imu_frame_crc(
    const std::array<std::uint8_t, kImuFrameSize>& frame) noexcept;

// 校验并解码一帧 IMU 上行数据；角速度统一输出为 rad/s。
bool decode_imu_frame(const std::uint8_t* data, std::size_t size,
                      ImuFrameValues* values,
                      std::string* error = nullptr);

}  // namespace serial_package

#endif  // SERIAL_PACKAGE_SERIAL_PROTOCOL_HPP
