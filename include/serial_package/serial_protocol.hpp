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
    // 舵机 P/Y 控制参数，已从单字节 [0, 255] 归一化到 [-1, 1]。
    double servo_control_p_normalized = 0.0;
    double servo_control_y_normalized = 0.0;
    std::uint16_t crc = 0;
};

/**
 * @brief 将过载值编码为固定 14 字节的下行控制帧。
 * @param values roll/yaw/pitch 过载值，单位为 g，按该顺序写入。
 * @param frame 非空输出数组；成功时写入协议帧。
 * @param error 可选错误输出；失败时写入原因，成功时清空。
 * @return 输入可编码且输出指针有效时为 true。
 * @note 帧头为 AA 55；roll/yaw/pitch 依次位于字节 2/4/6，均编码为乘以 100 的
 *       有符号 16 位小端整数。字节 8..9 保留为零，字节 10..11 为
 *       CRC-16/CCITT-FALSE，帧尾为 0D 0A。
 */
bool encode_control_frame(
    const ControlFrameValues& values,
    std::array<std::uint8_t, kControlFrameSize>* frame,
    std::string* error = nullptr);

/**
 * @brief 计算 IMU 上行帧数据区的 CRC-16/CCITT-FALSE。
 * @param frame 完整 20 字节 IMU 上行帧。
 * @return 覆盖字节 2 至 15 的 CRC 值。
 */
std::uint16_t calculate_imu_frame_crc(
    const std::array<std::uint8_t, kImuFrameSize>& frame) noexcept;

/**
 * @brief 校验并解码一帧固定格式 IMU 上行数据。
 * @param data 输入缓冲区，至少须指向 size 个可读字节。
 * @param size 输入长度；必须恰为 kImuFrameSize。
 * @param values 非空输出指针；仅在帧头、帧尾和 CRC 全部有效时写入。
 * @param error 可选错误输出；失败时写入原因，成功时清空。
 * @return 帧结构和 CRC 均有效时为 true。
 * @note 加速度输出 m/s^2，角速度输出 rad/s；舵机 P/Y 字节归一化到 [-1, 1]。
 */
bool decode_imu_frame(const std::uint8_t* data, std::size_t size,
                      ImuFrameValues* values,
                      std::string* error = nullptr);

}  // namespace serial_package

#endif  // SERIAL_PACKAGE_SERIAL_PROTOCOL_HPP
