#include <serial_package/serial_protocol.hpp>

#include "imu_frame_fixture.hpp"
#include "test_support.hpp"

#include <array>

int main() {
    serial_package::ImuFrameValues imu{};
    CHECK(serial_package::decode_imu_frame(
        mosas::test::kValidImuFrame.data(), mosas::test::kValidImuFrame.size(),
        &imu));
    CHECK_NEAR(imu.acceleration_x_mps2, 10.0, 1e-12);
    CHECK_NEAR(imu.acceleration_y_mps2, -2.0, 1e-12);
    CHECK_NEAR(imu.acceleration_z_mps2, 0.5, 1e-12);
    CHECK_NEAR(imu.angular_velocity_x_rad_s, 0.017453292519943295, 1e-12);
    CHECK_NEAR(imu.angular_velocity_y_rad_s, -0.03490658503988659, 1e-12);
    CHECK_NEAR(imu.angular_velocity_z_rad_s, 0.05235987755982989, 1e-12);
    CHECK(imu.crc == 0x4541);

    auto bad_crc = mosas::test::kValidImuFrame;
    bad_crc[2] ^= 0x01;
    CHECK(!serial_package::decode_imu_frame(
        bad_crc.data(), bad_crc.size(), &imu));
    auto bad_header = mosas::test::kValidImuFrame;
    bad_header[0] = 0x00;
    CHECK(!serial_package::decode_imu_frame(
        bad_header.data(), bad_header.size(), &imu));
    auto bad_footer = mosas::test::kValidImuFrame;
    bad_footer[18] = 0x00;
    CHECK(!serial_package::decode_imu_frame(
        bad_footer.data(), bad_footer.size(), &imu));
    CHECK(!serial_package::decode_imu_frame(
        mosas::test::kValidImuFrame.data(), mosas::test::kValidImuFrame.size() - 1,
        &imu));

    serial_package::ControlFrameValues command{1.25, -2.5, 0.005};
    std::array<std::uint8_t, serial_package::kControlFrameSize> encoded{};
    CHECK(serial_package::encode_control_frame(command, &encoded));
    constexpr std::array<std::uint8_t, serial_package::kControlFrameSize> expected{
        0xAA, 0x55, 0x7D, 0x00, 0x06, 0xFF, 0x01,
        0x00, 0x00, 0x00, 0xAC, 0x76, 0x0D, 0x0A};
    CHECK(encoded == expected);
    return 0;
}
