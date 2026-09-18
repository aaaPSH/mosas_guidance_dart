#include <mosas/app/raw_imu_receive_buffer.hpp>

#include <cassert>
#include <cstdint>
#include <cstring>

namespace {

void test_callback_copies_invalid_bytes_without_parsing() {
    mosas::app::RawImuReceiveBuffer buffer(2);
    const std::uint8_t bytes[] = {0x01, 0x02, 0x03, 0x04, 0x05};

    mosas::app::RawImuReceiveBuffer::receive_callback(
        &buffer, bytes, sizeof(bytes), 123'456);

    mosas::runtime::RawSerialChunk chunk{};
    assert(buffer.try_pop(&chunk));
    assert(chunk.first_byte_timestamp_ns == 123'456);
    assert(chunk.size == sizeof(bytes));
    assert(std::memcmp(chunk.bytes.data(), bytes, sizeof(bytes)) == 0);
}

void test_callback_drops_oversized_chunk_without_waiting() {
    mosas::app::RawImuReceiveBuffer buffer(1);
    const std::uint8_t bytes[] = {0x55, 0xAA};

    mosas::app::RawImuReceiveBuffer::receive_callback(
        &buffer, bytes, mosas::runtime::RawSerialChunk::kMaxChunkBytes + 1,
        1);
    assert(buffer.dropped_chunk_count() == 1);
}

}  // namespace

int main() {
    test_callback_copies_invalid_bytes_without_parsing();
    test_callback_drops_oversized_chunk_without_waiting();
    return 0;
}
