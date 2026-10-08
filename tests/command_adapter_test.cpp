#include "command_adapter.hpp"

#include "test_support.hpp"

#include <array>
#include <cstdint>

int main() {
    mosas::runtime::GuidanceCommand command{};
    command.valid = true;
    command.overload_g = {1.5, 0.005, -2.5};
    const auto values = mosas::app::detail::make_control_frame_values(command);
    CHECK_NEAR(values.roll_overload_g, 0.0, 1e-12);
    CHECK_NEAR(values.yaw_overload_g, -2.5, 1e-12);
    CHECK_NEAR(values.pitch_overload_g, 0.005, 1e-12);

    std::array<std::uint8_t, serial_package::kControlFrameSize> frame{};
    CHECK(serial_package::encode_control_frame(values, &frame));
    constexpr std::array<std::uint8_t, serial_package::kControlFrameSize>
        expected_valid_frame{0xAA, 0x55, 0x00, 0x00, 0x06, 0xFF, 0x01,
                             0x00, 0x00, 0x00, 0xC4, 0x90, 0x0D, 0x0A};
    CHECK(frame == expected_valid_frame);

    command.valid = false;
    const auto invalid_values =
        mosas::app::detail::make_control_frame_values(command);
    CHECK_NEAR(invalid_values.roll_overload_g, 0.0, 1e-12);
    CHECK_NEAR(invalid_values.yaw_overload_g, 0.0, 1e-12);
    CHECK_NEAR(invalid_values.pitch_overload_g, 0.0, 1e-12);
    CHECK(serial_package::encode_control_frame(invalid_values, &frame));
    constexpr std::array<std::uint8_t, serial_package::kControlFrameSize>
        expected_invalid_frame{0xAA, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00,
                               0x00, 0x00, 0x00, 0x3E, 0x31, 0x0D, 0x0A};
    CHECK(frame == expected_invalid_frame);
    return 0;
}
