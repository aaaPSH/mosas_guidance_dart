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
    double roll_overload_g = 0.0;
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
    std::uint16_t initialization_g_raw = 0;
    std::uint16_t crc = 0;
};

// 将下行控制量编码为固定长度的二进制控制帧。
bool encode_control_frame(
    const ControlFrameValues& values,
    std::array<std::uint8_t, kControlFrameSize>* frame,
    std::string* error = nullptr);

// 计算 IMU 上行帧 bytes 2..15 的 CRC-16/CCITT-FALSE。
std::uint16_t calculate_imu_frame_crc(
    const std::array<std::uint8_t, kImuFrameSize>& frame) noexcept;

// 校验并解码一帧 IMU 上行数据。
bool decode_imu_frame(const std::uint8_t* data, std::size_t size,
                      ImuFrameValues* values,
                      std::string* error = nullptr);

}  // namespace serial_package

#endif  // SERIAL_PACKAGE_SERIAL_PROTOCOL_HPP
