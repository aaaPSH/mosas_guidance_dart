#ifndef MOSAS_TEST_IMU_FRAME_FIXTURE_HPP
#define MOSAS_TEST_IMU_FRAME_FIXTURE_HPP

#include <array>
#include <cstdint>

namespace mosas::test {

inline constexpr std::array<std::uint8_t, 20> kValidImuFrame{
    0x55, 0xAA, 0xE8, 0x03, 0x38, 0xFF, 0x32, 0x00, 0x0A, 0x00,
    0xEC, 0xFF, 0x1E, 0x00, 0x80, 0x40, 0x41, 0x45, 0x0D, 0x0A};

}  // namespace mosas::test

#endif  // MOSAS_TEST_IMU_FRAME_FIXTURE_HPP
