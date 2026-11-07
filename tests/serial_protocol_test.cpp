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
    CHECK(serial_package::kImuFrameSize == 29);
    CHECK_NEAR(imu.servo_control_p_normalized, 1.0 / 255.0, 1e-12);
    CHECK_NEAR(imu.servo_control_y_normalized, -127.0 / 255.0, 1e-12);
    CHECK_NEAR(imu.outer_loop_y_rad_s, -1.234, 1e-12);
    CHECK_NEAR(imu.outer_loop_p_rad_s, 2.345, 1e-12);
    CHECK(imu.imu_index == 0x12345678u);
    CHECK(imu.trigger_flag == 1);
    CHECK(imu.crc == 0xFA3B);

    // 每个数据字节（包括外环、序号和触发位）均必须参与 CRC，失败不改写输出。
    for (std::size_t offset = 2; offset <= 26; ++offset) {
        auto bad_crc = mosas::test::kValidImuFrame;
        bad_crc[offset] ^= 0x01;
        CHECK(!serial_package::decode_imu_frame(
            bad_crc.data(), bad_crc.size(), &imu));
        CHECK(imu.imu_index == 0x12345678u);
    }
    auto extremes = mosas::test::make_imu_frame(UINT32_MAX);
    extremes[16] = 0x00; extremes[17] = 0x80;
    extremes[18] = 0xFF; extremes[19] = 0x7F;
    extremes[24] = 0;
    const auto crc = serial_package::calculate_imu_frame_crc(extremes);
    extremes[25] = static_cast<std::uint8_t>(crc);
    extremes[26] = static_cast<std::uint8_t>(crc >> 8);
    CHECK(serial_package::decode_imu_frame(extremes.data(), extremes.size(), &imu));
    CHECK_NEAR(imu.outer_loop_y_rad_s, -32.768, 1e-12);
    CHECK_NEAR(imu.outer_loop_p_rad_s, 32.767, 1e-12);
    CHECK(imu.imu_index == UINT32_MAX);
    CHECK(imu.trigger_flag == 0);
    auto bad_header = mosas::test::kValidImuFrame;
    bad_header[0] = 0x00;
    CHECK(!serial_package::decode_imu_frame(
        bad_header.data(), bad_header.size(), &imu));
    auto bad_footer = mosas::test::kValidImuFrame;
    bad_footer[27] = 0x00;
    CHECK(!serial_package::decode_imu_frame(
        bad_footer.data(), bad_footer.size(), &imu));
    CHECK(!serial_package::decode_imu_frame(
        mosas::test::kValidImuFrame.data(), mosas::test::kValidImuFrame.size() - 1,
        &imu));
    CHECK(!serial_package::decode_imu_frame(
        mosas::test::kValidImuFrame.data(), 20, &imu));

    // 固定参考帧验证两个标志独立编码，并且都参与 CRC。
    constexpr std::array<std::array<std::uint8_t,
                                    serial_package::kControlFrameSize>, 4>
        expected_frames{{
            {0xAA, 0x55, 0x00, 0x00, 0x06, 0xFF, 0x01,
             0x00, 0x00, 0x00, 0xC4, 0x90, 0x0D, 0x0A},
            {0xAA, 0x55, 0x00, 0x01, 0x06, 0xFF, 0x01,
             0x00, 0x00, 0x00, 0xA5, 0x28, 0x0D, 0x0A},
            {0xAA, 0x55, 0x01, 0x00, 0x06, 0xFF, 0x01,
             0x00, 0x00, 0x00, 0x17, 0xD7, 0x0D, 0x0A},
            {0xAA, 0x55, 0x01, 0x01, 0x06, 0xFF, 0x01,
             0x00, 0x00, 0x00, 0x76, 0x6F, 0x0D, 0x0A},
        }};
    for (const auto& expected : expected_frames) {
        const serial_package::ControlFrameValues command{
            expected[2] != 0, expected[3] != 0, -2.5, 0.005};
        std::array<std::uint8_t, serial_package::kControlFrameSize> encoded{};
        CHECK(serial_package::encode_control_frame(command, &encoded));
        CHECK(encoded == expected);
    }
    return 0;
}
