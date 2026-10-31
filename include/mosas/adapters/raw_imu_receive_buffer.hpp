#ifndef MOSAS_ADAPTERS_RAW_IMU_RECEIVE_BUFFER_HPP
#define MOSAS_ADAPTERS_RAW_IMU_RECEIVE_BUFFER_HPP

#include <mosas/runtime/raw_serial_chunk.hpp>
#include <mosas/runtime/spsc_ring.hpp>

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <mutex>

namespace mosas::app {

/**
 * @brief 单生产者、单消费者的有界原始串口缓冲区。
 * @note close() 可并发调用，它定义本轮接收及诊断统计的结束边界。
 */
class RawImuReceiveBuffer final {
public:
    /** @brief 创建开放接收的缓冲区。 @param capacity 可排队块数上限，必须为正数。 */
    explicit RawImuReceiveBuffer(std::size_t capacity)
        : queue_(capacity) {}

    RawImuReceiveBuffer(const RawImuReceiveBuffer&) = delete;
    RawImuReceiveBuffer& operator=(const RawImuReceiveBuffer&) = delete;

    /**
     * @brief 转发串口回调，不在回调中解析协议。
     * @param context 缓冲区实例；为空时忽略调用。
     * @param data 由 receive() 复制的原始字节。
     * @param size 输入字节数。
     * @param first_byte_timestamp_ns 首字节接收单调时间的估计值，单位 ns。
     */
    static void receive_callback(
        void* context, const std::uint8_t* data, std::size_t size,
        std::int64_t first_byte_timestamp_ns) noexcept;

    /**
     * @brief 仅在接收开放期间复制并入队原始字节块。
     * @param data 可读的输入字节；接收开放时不允许为空。
     * @param size 字节数，范围为 [1, RawSerialChunk::kMaxChunkBytes]。
     * @param first_byte_timestamp_ns 非负的单调时间估计值，单位 ns。
     * @note 无效块及队列溢出均计入丢弃；close() 之后的调用忽略且不再改变统计。
     *       仅允许一个生产者，与关闭操作的边界通过内部互斥锁串行化。
     */
    void receive(const std::uint8_t* data, std::size_t size,
                 runtime::TimestampNs first_byte_timestamp_ns) noexcept;

    /**
     * @brief 取出已保留的块，允许在 close() 之后继续排空。
     * @param chunk 非空输出指针。
     * @return 成功取出时为 true；队列为空或输出为空时为 false。
     */
    bool try_pop(runtime::RawSerialChunk* chunk) noexcept {
        return queue_.try_pop(chunk);
    }

    /**
     * @brief 等待数据到达或当前接收会话结束。
     * @return 本轮接收仍开放且已有数据时为 true；关闭或本轮代际已被重置时为 false。
     */
    bool wait_for_data() noexcept;
    /**
     * @brief 结束接收并唤醒等待者，不丢弃已经保留的块。
     * @note 幂等；晚到的回调不会改变接收及丢弃计数。
     */
    void close() noexcept;

    /** @brief 清空数据、计数及序号后重新开放接收；调用前生产者和消费者均须停止。 */
    void reset() noexcept;

    /** @brief 查询本轮接收丢失。 @return 关闭前无效或队列溢出的块数。 */
    std::uint64_t dropped_chunk_count() const noexcept {
        return dropped_chunk_count_.load(std::memory_order_relaxed);
    }

    /** @brief 查询本轮接收尝试。 @return 关闭前收到的全部块数，包含被拒绝的块。 */
    std::uint64_t received_chunk_count() const noexcept {
        return received_chunk_count_.load(std::memory_order_relaxed);
    }

    /** @brief 查询队列占用。 @return 当前保留的块数。 */
    std::size_t queued_chunk_count() const noexcept { return queue_.size(); }

    /** @brief 查询固定容量。 @return 最多可保留的块数。 */
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

#endif  // MOSAS_ADAPTERS_RAW_IMU_RECEIVE_BUFFER_HPP
