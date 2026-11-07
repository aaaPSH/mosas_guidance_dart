#ifndef MOSAS_TEST_IMU_FRAME_FIXTURE_HPP
#define MOSAS_TEST_IMU_FRAME_FIXTURE_HPP

#include <serial_package/serial_protocol.hpp>

#include <array>
#include <cstdint>

namespace mosas::test {

inline constexpr std::array<std::uint8_t, serial_package::kImuFrameSize> kValidImuFrame{
    0x55, 0xAA, 0xE8, 0x03, 0x38, 0xFF, 0x32, 0x00, 0x0A, 0x00,
    0xEC, 0xFF, 0x1E, 0x00, 0x80, 0x40, 0x2E, 0xFB, 0x29, 0x09,
    0x78, 0x56, 0x34, 0x12, 0x01, 0x3B, 0xFA, 0x0D, 0x0A};

/**
 * @brief 为连续采集测试生成指定设备序号的有效上行帧。
 * @param imu_index 每帧递增的原始 uint32 设备计数器。
 * @return 更新序号和 CRC 后的 29 字节参考帧。
 */
inline auto make_imu_frame(std::uint32_t imu_index) {
    auto frame = kValidImuFrame;
    for (unsigned byte = 0; byte < 4; ++byte) {
        frame[20 + byte] = static_cast<std::uint8_t>(imu_index >> (byte * 8));
    }
    const auto crc = serial_package::calculate_imu_frame_crc(frame);
    frame[25] = static_cast<std::uint8_t>(crc);
    frame[26] = static_cast<std::uint8_t>(crc >> 8);
    return frame;
}

}  // namespace mosas::test

#endif  // MOSAS_TEST_IMU_FRAME_FIXTURE_HPP
