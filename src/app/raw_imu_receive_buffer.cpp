#include <mosas/app/raw_imu_receive_buffer.hpp>

#include <cstring>

namespace mosas::app {

void RawImuReceiveBuffer::receive_callback(
    void* context, const std::uint8_t* data, std::size_t size,
    std::int64_t first_byte_timestamp_ns) noexcept {
    if (context == nullptr) {
        return;
    }
    static_cast<RawImuReceiveBuffer*>(context)->receive(
        data, size, first_byte_timestamp_ns);
}

void RawImuReceiveBuffer::receive(
    const std::uint8_t* data, std::size_t size,
    runtime::TimestampNs first_byte_timestamp_ns) noexcept {
    received_chunk_count_.fetch_add(1, std::memory_order_relaxed);
    if (data == nullptr || size == 0 ||
        size > runtime::RawSerialChunk::kMaxChunkBytes ||
        first_byte_timestamp_ns < 0) {
        dropped_chunk_count_.fetch_add(1, std::memory_order_relaxed);
        return;
    }

    runtime::RawSerialChunk chunk{};
    chunk.first_byte_timestamp_ns = first_byte_timestamp_ns;
    chunk.sequence = next_sequence_++;
    chunk.size = static_cast<std::uint16_t>(size);
    std::memcpy(chunk.bytes.data(), data, size);
    if (!queue_.try_push(std::move(chunk))) {
        dropped_chunk_count_.fetch_add(1, std::memory_order_relaxed);
    }
}

}  // namespace mosas::app
