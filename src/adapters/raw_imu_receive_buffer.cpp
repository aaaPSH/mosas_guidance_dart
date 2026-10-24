#include <mosas/adapters/raw_imu_receive_buffer.hpp>

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
    {
        std::lock_guard<std::mutex> lock(wait_mutex_);
        // 与 close() 串行化接收和统计；已发起的串口读取可能在取消后才交付最后一个回调，
        // 该回调不再属于本轮采集。
        if (closed_) {
            return;
        }
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
            return;
        }
    }
    wait_condition_.notify_one();
}

bool RawImuReceiveBuffer::wait_for_data() noexcept {
    std::unique_lock<std::mutex> lock(wait_mutex_);
    const std::uint64_t generation = wait_generation_;
    wait_condition_.wait(lock, [this, generation] {
        return closed_ || wait_generation_ != generation || !queue_.empty();
    });
    return !closed_ && wait_generation_ == generation;
}

void RawImuReceiveBuffer::close() noexcept {
    {
        std::lock_guard<std::mutex> lock(wait_mutex_);
        closed_ = true;
        ++wait_generation_;
    }
    wait_condition_.notify_all();
}

void RawImuReceiveBuffer::reset() noexcept {
    {
        std::lock_guard<std::mutex> lock(wait_mutex_);
        queue_.reset();
        next_sequence_ = 0;
        received_chunk_count_.store(0, std::memory_order_relaxed);
        dropped_chunk_count_.store(0, std::memory_order_relaxed);
        closed_ = false;
        ++wait_generation_;
    }
    wait_condition_.notify_all();
}

}  // namespace mosas::app
