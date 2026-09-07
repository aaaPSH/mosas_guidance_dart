#include <serial_package/serial_protocol.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <sstream>

namespace serial_package {
namespace {

constexpr double kOverloadScale = 100.0;

void set_error(const std::string& message, std::string* error) {
    if (error != nullptr) {
        *error = message;
    }
}

std::string hex_byte(std::uint8_t value) {
    std::ostringstream stream;
    stream << "0x" << std::hex << std::uppercase << std::setfill('0')
           << std::setw(2) << static_cast<unsigned int>(value);
    return stream.str();
}

std::string hex_word(std::uint16_t value) {
    std::ostringstream stream;
    stream << "0x" << std::hex << std::uppercase << std::setfill('0')
           << std::setw(4) << static_cast<unsigned int>(value);
    return stream.str();
}

std::uint16_t calculate_crc16_ccitt_false(const std::uint8_t* data,
                                          std::size_t size) noexcept {
    std::uint16_t crc = 0xFFFFu;
    for (std::size_t index = 0; index < size; ++index) {
        crc = static_cast<std::uint16_t>(
            crc ^ (static_cast<std::uint16_t>(data[index]) << 8u));
        for (int bit = 0; bit < 8; ++bit) {
            if ((crc & 0x8000u) != 0) {
                crc = static_cast<std::uint16_t>((crc << 1u) ^ 0x1021u);
            } else {
                crc = static_cast<std::uint16_t>(crc << 1u);
            }
        }
    }
    return crc;
}

std::int16_t read_int16_le(const std::uint8_t* data) noexcept {
    const std::uint16_t encoded =
        static_cast<std::uint16_t>(data[0]) |
        static_cast<std::uint16_t>(static_cast<std::uint16_t>(data[1]) << 8u);
    if (encoded >= 0x8000u) {
        return static_cast<std::int16_t>(
            static_cast<std::int32_t>(encoded) - 0x10000);
    }
    return static_cast<std::int16_t>(encoded);
}

bool encode_overload(double value, std::uint8_t* destination,
                     std::string* error) {
    if (destination == nullptr) {
        set_error("控制帧字段输出指针为空", error);
        return false;
    }
    if (!std::isfinite(value)) {
        set_error("控制过载必须是有限数值", error);
        return false;
    }

    const double scaled = std::round(value * kOverloadScale);
    if (!std::isfinite(scaled) ||
        scaled < static_cast<double>(std::numeric_limits<std::int16_t>::min()) ||
        scaled > static_cast<double>(std::numeric_limits<std::int16_t>::max())) {
        set_error("控制过载超出协议范围", error);
        return false;
    }

    const auto quantized = static_cast<std::int16_t>(scaled);
    const auto encoded = static_cast<std::uint16_t>(quantized);
    destination[0] = static_cast<std::uint8_t>(encoded & 0xFFu);
    destination[1] = static_cast<std::uint8_t>((encoded >> 8u) & 0xFFu);
    return true;
}

}  // namespace

bool encode_control_frame(
    const ControlFrameValues& values,
    std::array<std::uint8_t, kControlFrameSize>* frame,
    std::string* error) {
    if (frame == nullptr) {
        set_error("控制帧输出指针为空", error);
        return false;
    }

    frame->fill(0);
    (*frame)[0] = 0xAA;
    (*frame)[1] = 0x55;
    if (!encode_overload(values.roll_overload_g, frame->data() + 2, error) ||
        !encode_overload(values.yaw_overload_g, frame->data() + 4, error) ||
        !encode_overload(values.pitch_overload_g, frame->data() + 6, error)) {
        return false;
    }

    const std::uint16_t crc =
        calculate_crc16_ccitt_false(frame->data() + 2, 8);
    (*frame)[10] = static_cast<std::uint8_t>(crc & 0xFFu);
    (*frame)[11] = static_cast<std::uint8_t>((crc >> 8u) & 0xFFu);
    (*frame)[12] = 0x0D;
    (*frame)[13] = 0x0A;
    if (error != nullptr) {
        error->clear();
    }
    return true;
}

std::uint16_t calculate_imu_frame_crc(
    const std::array<std::uint8_t, kImuFrameSize>& frame) noexcept {
    return calculate_crc16_ccitt_false(frame.data() + 2, 14);
}

bool decode_imu_frame(const std::uint8_t* data, std::size_t size,
                      ImuFrameValues* values, std::string* error) {
    if (data == nullptr) {
        set_error("IMU 帧数据指针为空", error);
        return false;
    }
    if (size != kImuFrameSize) {
        std::ostringstream message;
        message << "IMU 帧长度错误: received=" << size
                << ", expected=" << kImuFrameSize;
        set_error(message.str(), error);
        return false;
    }
    if (values == nullptr) {
        set_error("IMU 帧输出指针为空", error);
        return false;
    }
    if (data[0] != 0x55 || data[1] != 0xAA) {
        std::ostringstream message;
        message << "IMU 帧头错误: received=" << hex_byte(data[0]) << ' '
                << hex_byte(data[1]) << ", expected=0x55 0xAA";
        set_error(message.str(), error);
        return false;
    }
    if (data[18] != 0x0D || data[19] != 0x0A) {
        std::ostringstream message;
        message << "IMU 帧尾错误: received=" << hex_byte(data[18]) << ' '
                << hex_byte(data[19]) << ", expected=0x0D 0x0A";
        set_error(message.str(), error);
        return false;
    }

    std::array<std::uint8_t, kImuFrameSize> frame{};
    std::copy(data, data + kImuFrameSize, frame.begin());
    const std::uint16_t expected_crc = calculate_imu_frame_crc(frame);
    const std::uint16_t received_crc =
        static_cast<std::uint16_t>(data[16]) |
        static_cast<std::uint16_t>(static_cast<std::uint16_t>(data[17]) << 8u);
    if (received_crc != expected_crc) {
        std::ostringstream message;
        message << "IMU CRC 错误: received=" << hex_word(received_crc)
                << ", expected=" << hex_word(expected_crc);
        set_error(message.str(), error);
        return false;
    }

    ImuFrameValues decoded;
    decoded.acceleration_x_mps2 =
        static_cast<double>(read_int16_le(data + 2)) * 0.01;
    decoded.acceleration_y_mps2 =
        static_cast<double>(read_int16_le(data + 4)) * 0.01;
    decoded.acceleration_z_mps2 =
        static_cast<double>(read_int16_le(data + 6)) * 0.01;
    decoded.angular_velocity_x_rad_s =
        static_cast<double>(read_int16_le(data + 8)) * 0.1;
    decoded.angular_velocity_y_rad_s =
        static_cast<double>(read_int16_le(data + 10)) * 0.1;
    decoded.angular_velocity_z_rad_s =
        static_cast<double>(read_int16_le(data + 12)) * 0.1;
    decoded.initialization_g_raw =
        static_cast<std::uint16_t>(data[14]) |
        static_cast<std::uint16_t>(static_cast<std::uint16_t>(data[15]) << 8u);
    decoded.crc = received_crc;
    *values = decoded;
    if (error != nullptr) {
        error->clear();
    }
    return true;
}

}  // namespace serial_package
