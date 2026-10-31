#ifndef MOSAS_ADAPTERS_IMU_STREAM_DECODER_HPP
#define MOSAS_ADAPTERS_IMU_STREAM_DECODER_HPP

#include <mosas/runtime/imu_angular_velocity.hpp>
#include <mosas/domain/sequence_time.hpp>
#include <mosas/runtime/raw_serial_chunk.hpp>
#include <mosas/runtime/spsc_ring.hpp>

#include <cstddef>
#include <cstdint>
#include <atomic>
#include <array>

namespace mosas::app {

/** @brief 将串口帧解码为固定逻辑采样序号时间轴；接收时间只用于基线和诊断。 */
class ImuStreamDecoder final {
public:
    /**
     * @brief 创建固定协议解码器和有界样本输出队列。
     * @param wire_byte_period_ns 线上单字节传输周期估值，用于恢复帧首字节时刻（ns）。
     * @param measurement_latency_ns IMU 测量时刻相对串口接收时刻的延迟补偿（ns）。
     * @param output_capacity 解码样本队列容量；队列满时新样本会被丢弃并计数。
     * @param timing 实际频率和启动同步窗口。
     */
    ImuStreamDecoder(runtime::TimestampNs wire_byte_period_ns,
                     runtime::TimestampNs measurement_latency_ns,
                     std::size_t output_capacity, domain::SequenceTimeConfig timing);

    ImuStreamDecoder(const ImuStreamDecoder&) = delete;
    ImuStreamDecoder& operator=(const ImuStreamDecoder&) = delete;

    /**
     * @brief 将一个接收块追加到解析缓冲并解码其中完整的 IMU 协议帧。
     * @param chunk 带首字节单调接收时刻和接收序号的原始串口块。
     * @param now_ns 当前单调时间，用于识别应用缓冲积压。
     * @note 仅由 IMU 消费线程调用；坏帧和未知失步分别统计，不按CRC失败次数猜号。
     */
    void consume(const runtime::RawSerialChunk& chunk, runtime::TimestampNs now_ns) noexcept;

    /** @param sync 运行时同步屏障，须在启动前绑定。 */
    void bind_sync(std::shared_ptr<domain::AcquisitionSync> sync) { timeline_.bind(std::move(sync), 2); }
    /** @return 固定时间轴诊断快照。 */
    domain::SequenceTimeDiagnostics time_diagnostics() const noexcept { return timeline_.diagnostics(); }
    /** @return 帧头/尾完整但CRC错误的候选帧数。 */
    std::uint64_t crc_error_count() const noexcept { return crc_error_count_.load(); }

    /**
     * @brief 非阻塞地取出一个已解码 IMU 样本。
     * @param sample 非空输出指针；成功时写入样本。
     * @return 有样本可取时为 true，否则为 false。
     */
    bool try_pop(runtime::ImuAngularVelocitySample* sample) noexcept;

    /**
     * @brief 清除解析缓冲、样本队列和诊断计数。
     * @param now_ns 本次采集开始的单调时间。
     * @note 仅在线程停止消费输出后调用。
     */
    void reset(runtime::TimestampNs now_ns) noexcept;

    std::uint64_t parse_error_count() const noexcept {
        return parse_error_count_.load(std::memory_order_relaxed);
    }

    std::uint64_t sequence_gap_count() const noexcept {
        return sequence_gap_count_.load(std::memory_order_relaxed);
    }

    std::uint64_t dropped_sample_count() const noexcept {
        return dropped_sample_count_.load(std::memory_order_relaxed);
    }

private:
    void clear_buffer() noexcept;
    void discard_prefix(std::size_t count) noexcept;
    void align_buffer() noexcept;
    void lose_alignment() noexcept;
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
    domain::SequenceTime timeline_;
    std::uint64_t sample_sequence_ = 0;
    std::uint64_t pending_bad_frames_ = 0;
    bool boundary_trusted_ = false;
    bool seeking_ = false;
    std::atomic<std::uint64_t> parse_error_count_{0};
    std::atomic<std::uint64_t> sequence_gap_count_{0};
    std::atomic<std::uint64_t> crc_error_count_{0};
    std::atomic<std::uint64_t> dropped_sample_count_{0};
};

}  // namespace mosas::app

#endif  // MOSAS_ADAPTERS_IMU_STREAM_DECODER_HPP
