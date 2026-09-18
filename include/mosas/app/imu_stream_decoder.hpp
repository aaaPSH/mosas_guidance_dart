#ifndef MOSAS_APP_IMU_STREAM_DECODER_HPP
#define MOSAS_APP_IMU_STREAM_DECODER_HPP

#include <mosas/runtime/imu_angular_velocity.hpp>
#include <mosas/runtime/raw_serial_chunk.hpp>
#include <mosas/runtime/spsc_ring.hpp>

#include <cstddef>
#include <cstdint>
#include <atomic>
#include <array>

namespace mosas::app {

class ImuStreamDecoder final {
public:
    ImuStreamDecoder(runtime::TimestampNs wire_byte_period_ns,
                     runtime::TimestampNs measurement_latency_ns,
                     std::size_t output_capacity);

    ImuStreamDecoder(const ImuStreamDecoder&) = delete;
    ImuStreamDecoder& operator=(const ImuStreamDecoder&) = delete;

    // 该函数只在普通 IMU 消费线程调用，回调线程不得调用。
    void consume(const runtime::RawSerialChunk& chunk) noexcept;

    bool try_pop(runtime::ImuAngularVelocitySample* sample) noexcept;

    // 仅允许在调用线程停止消费输出后调用。
    void reset() noexcept;

    std::uint64_t parse_error_count() const noexcept {
        return parse_error_count_.load(std::memory_order_relaxed);
    }

    std::uint64_t sequence_gap_count() const noexcept {
        return sequence_gap_count_.load(std::memory_order_relaxed);
    }

    std::uint64_t timestamp_rollback_count() const noexcept {
        return timestamp_rollback_count_.load(std::memory_order_relaxed);
    }

    std::uint64_t dropped_sample_count() const noexcept {
        return dropped_sample_count_.load(std::memory_order_relaxed);
    }

private:
    void clear_buffer() noexcept;
    void discard_prefix(std::size_t count) noexcept;
    void align_buffer() noexcept;
    bool append(const runtime::RawSerialChunk& chunk) noexcept;
    bool publish(const runtime::ImuAngularVelocitySample& sample) noexcept;

    static constexpr std::size_t kRawBufferSize = 4096;

    runtime::TimestampNs wire_byte_period_ns_ = 0;
    runtime::TimestampNs measurement_latency_ns_ = 0;
    runtime::SpscRing<runtime::ImuAngularVelocitySample> output_queue_;
    std::array<std::uint8_t, kRawBufferSize> buffer_{};
    std::size_t buffered_size_ = 0;
    runtime::TimestampNs buffer_first_timestamp_ns_ = -1;
    bool has_sequence_ = false;
    std::uint64_t expected_sequence_ = 0;
    bool has_timestamp_ = false;
    runtime::TimestampNs last_timestamp_ns_ = -1;
    std::atomic<std::uint64_t> parse_error_count_{0};
    std::atomic<std::uint64_t> sequence_gap_count_{0};
    std::atomic<std::uint64_t> timestamp_rollback_count_{0};
    std::atomic<std::uint64_t> dropped_sample_count_{0};
};

}  // namespace mosas::app

#endif  // MOSAS_APP_IMU_STREAM_DECODER_HPP
