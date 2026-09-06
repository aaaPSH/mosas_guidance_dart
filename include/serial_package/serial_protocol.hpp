#ifndef SERIAL_PACKAGE_SERIAL_PROTOCOL_HPP
#define SERIAL_PACKAGE_SERIAL_PROTOCOL_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

namespace serial_package {

constexpr std::size_t kControlFrameSize = 14;

struct ControlFrameValues {
    double roll_overload_g = 0.0;
    double yaw_overload_g = 0.0;
    double pitch_overload_g = 0.0;
};

// 将下行控制量编码为固定长度的二进制控制帧。
bool encode_control_frame(
    const ControlFrameValues& values,
    std::array<std::uint8_t, kControlFrameSize>* frame,
    std::string* error = nullptr);

}  // namespace serial_package

#endif  // SERIAL_PACKAGE_SERIAL_PROTOCOL_HPP
