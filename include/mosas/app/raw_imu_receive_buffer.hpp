#ifndef MOSAS_APP_RAW_IMU_RECEIVE_BUFFER_HPP
#define MOSAS_APP_RAW_IMU_RECEIVE_BUFFER_HPP

#include <mosas/runtime/raw_serial_chunk.hpp>
#include <mosas/runtime/spsc_ring.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <mutex>

namespace mosas::app {

class RawImuReceiveBuffer final {
public:
    explicit RawImuReceiveBuffer(std::size_t capacity)
        : queue_(capacity) {}

    RawImuReceiveBuffer(const RawImuReceiveBuffer&) = delete;
    RawImuReceiveBuffer& operator=(const RawImuReceiveBuffer&) = delete;

    // 串口回调只复制原始字节和时间戳，不解析协议。
    static void receive_callback(
        void* context, const std::uint8_t* data, std::size_t size,
        std::int64_t first_byte_timestamp_ns) noexcept;

    void receive(const std::uint8_t* data, std::size_t size,
                 runtime::TimestampNs first_byte_timestamp_ns) noexcept;

    bool try_pop(runtime::RawSerialChunk* chunk) noexcept {
        return queue_.try_pop(chunk);
    }

    bool wait_for_data() noexcept;
    bool wait_for_data_until(
        std::chrono::steady_clock::time_point deadline) noexcept;
    void close() noexcept;

    // 仅允许在串口接收线程和消费线程停止后调用。
    void reset() noexcept;

    std::uint64_t dropped_chunk_count() const noexcept {
        return dropped_chunk_count_.load(std::memory_order_relaxed);
    }

    std::uint64_t received_chunk_count() const noexcept {
        return received_chunk_count_.load(std::memory_order_relaxed);
    }

    std::size_t queued_chunk_count() const noexcept { return queue_.size(); }

    std::size_t queue_capacity() const noexcept { return queue_.capacity(); }

private:
    runtime::SpscRing<runtime::RawSerialChunk> queue_;
    std::mutex wait_mutex_;
    std::condition_variable wait_condition_;
    bool closed_ = false;
    std::uint64_t wait_generation_ = 0;
    std::uint64_t next_sequence_ = 0;
    std::atomic<std::uint64_t> received_chunk_count_{0};
    std::atomic<std::uint64_t> dropped_chunk_count_{0};
};

}  // namespace mosas::app

#endif  // MOSAS_APP_RAW_IMU_RECEIVE_BUFFER_HPP
