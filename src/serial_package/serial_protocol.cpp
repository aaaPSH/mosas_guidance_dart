#include <serial_package/serial_protocol.hpp>

#include <cmath>
#include <cstdint>
#include <limits>

namespace serial_package {
namespace {

constexpr double kOverloadScale = 100.0;

void set_error(const char* message, std::string* error) {
    if (error != nullptr) {
        *error = message;
    }
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

    std::uint16_t checksum = 0;
    for (std::size_t index = 2; index < 10; ++index) {
        checksum = static_cast<std::uint16_t>(checksum + (*frame)[index]);
    }
    (*frame)[10] = static_cast<std::uint8_t>(checksum & 0xFFu);
    (*frame)[11] = static_cast<std::uint8_t>((checksum >> 8u) & 0xFFu);
    (*frame)[12] = 0x0D;
    (*frame)[13] = 0x0A;
    if (error != nullptr) {
        error->clear();
    }
    return true;
}

}  // namespace serial_package
