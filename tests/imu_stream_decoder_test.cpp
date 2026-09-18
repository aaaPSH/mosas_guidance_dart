#include <mosas/app/imu_stream_decoder.hpp>

#include <serial_package/serial_protocol.hpp>

#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>

namespace {

constexpr mosas::runtime::TimestampNs kBytePeriodNs = 100;

std::array<std::uint8_t, serial_package::kImuFrameSize> make_frame(
    std::int16_t angular_x, std::int16_t angular_y,
    std::int16_t angular_z) {
    std::array<std::uint8_t, serial_package::kImuFrameSize> frame{};
    frame[0] = 0x55;
    frame[1] = 0xAA;
    auto write_int16 = [&frame](std::size_t offset, std::int16_t value) {
        const auto encoded = static_cast<std::uint16_t>(value);
        frame[offset] = static_cast<std::uint8_t>(encoded & 0xFFU);
        frame[offset + 1] = static_cast<std::uint8_t>(encoded >> 8U);
    };
    write_int16(8, angular_x);
    write_int16(10, angular_y);
    write_int16(12, angular_z);
    frame[18] = 0x0D;
    frame[19] = 0x0A;
    const auto crc = serial_package::calculate_imu_frame_crc(frame);
    frame[16] = static_cast<std::uint8_t>(crc & 0xFFU);
    frame[17] = static_cast<std::uint8_t>(crc >> 8U);
    return frame;
}

mosas::runtime::RawSerialChunk make_chunk(const std::uint8_t* data,
                                           std::size_t size,
                                           mosas::runtime::TimestampNs timestamp,
                                           std::uint64_t sequence) {
    mosas::runtime::RawSerialChunk chunk{};
    chunk.first_byte_timestamp_ns = timestamp;
    chunk.sequence = sequence;
    assert(size <= chunk.bytes.size());
    chunk.size = static_cast<std::uint16_t>(size);
    std::memcpy(chunk.bytes.data(), data, size);
    return chunk;
}

void test_split_frame_preserves_frame_start_timestamp() {
    mosas::app::ImuStreamDecoder decoder(kBytePeriodNs, 0, 4);
    const auto frame = make_frame(10, -20, 30);

    decoder.consume(make_chunk(frame.data(), 7, 900'000, 1));
    mosas::runtime::ImuAngularVelocitySample sample{};
    assert(!decoder.try_pop(&sample));

    decoder.consume(make_chunk(frame.data() + 7, frame.size() - 7,
                               900'000 + 7 * kBytePeriodNs, 2));
    assert(decoder.try_pop(&sample));
    assert(sample.timestamp_ns == 900'000);
    assert(std::abs(sample.angular_velocity.x - 1.0) < 1e-12);
    assert(std::abs(sample.angular_velocity.y + 2.0) < 1e-12);
    assert(std::abs(sample.angular_velocity.z - 3.0) < 1e-12);
}

void test_sequence_gap_discards_partial_frame_before_resynchronizing() {
    mosas::app::ImuStreamDecoder decoder(kBytePeriodNs, 0, 4);
    const auto frame = make_frame(10, 20, 30);
    decoder.consume(make_chunk(frame.data(), 8, 1'000, 10));
    decoder.consume(make_chunk(frame.data() + 8, frame.size() - 8, 1'800, 12));
    assert(decoder.sequence_gap_count() == 1);

    const auto later = make_frame(-10, -20, -30);
    decoder.consume(make_chunk(later.data(), later.size(), 3'000, 13));
    mosas::runtime::ImuAngularVelocitySample sample{};
    assert(decoder.try_pop(&sample));
    assert(sample.timestamp_ns == 3'000);
    assert(sample.angular_velocity.x < 0.0);
}

void test_bad_crc_does_not_reach_angle_rate_output() {
    mosas::app::ImuStreamDecoder decoder(kBytePeriodNs, 0, 4);
    auto frame = make_frame(10, 20, 30);
    frame[16] ^= 0xFFU;
    decoder.consume(make_chunk(frame.data(), frame.size(), 2'000, 1));
    mosas::runtime::ImuAngularVelocitySample sample{};
    assert(!decoder.try_pop(&sample));
    assert(decoder.parse_error_count() == 1);
}

}  // namespace

int main() {
    test_split_frame_preserves_frame_start_timestamp();
    test_sequence_gap_discards_partial_frame_before_resynchronizing();
    test_bad_crc_does_not_reach_angle_rate_output();
    return 0;
}
