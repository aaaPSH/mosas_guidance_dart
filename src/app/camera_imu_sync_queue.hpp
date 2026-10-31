#ifndef MOSAS_APP_CAMERA_IMU_SYNC_QUEUE_HPP
#define MOSAS_APP_CAMERA_IMU_SYNC_QUEUE_HPP

#include <mosas/app/camera_imu_sync_session.hpp>
#include <mosas/runtime/spsc_ring.hpp>

#include <algorithm>
#include <atomic>
#include <utility>

namespace mosas::app::detail {

/**
 * @brief 有界 SPSC 记录队列；溢出使采集无效，不静默修复或覆盖数据。
 * @note 计数由唯一生产者写入，仅在生产者退出后读取；共享错误标志为原子变量。
 *       溢出策略集中在队列层，便于在没有磁盘消费线程时确定性验证。
 */
template <typename T>
class SyncRecordQueue {
public:
    /** @param capacity 固定记录容量。 @param failure 共享首次错误。 @param overflow 本队列的溢出错误码。 */
    SyncRecordQueue(std::size_t capacity, std::atomic<CameraImuSyncError>& failure, CameraImuSyncError overflow)
        : queue_(capacity), failure_(failure), overflow_(overflow) {}
    /** @brief 不等待地入队记录。 @param value 待转移的记录。 @return 队列满时锁存错误并返回 false。 */
    bool try_push(T&& value) noexcept {
        const auto depth = std::min(queue_.size() + 1, queue_.capacity());
        if (!queue_.try_push(std::move(value))) { reject(); return false; }
        high_water_ = std::max(high_water_, depth);
        return true;
    }
    /** @brief 队列已满时在昂贵的图像拷贝之前拒绝记录并计数。 */
    void reject() noexcept {
        ++dropped_;
        auto expected = CameraImuSyncError::none;
        failure_.compare_exchange_strong(expected, overflow_, std::memory_order_relaxed);
    }
    bool try_pop(T* value) noexcept { return queue_.try_pop(value); }
    bool empty() const noexcept { return queue_.empty(); }
    bool full() const noexcept { return queue_.size() == queue_.capacity(); }
    std::size_t capacity() const noexcept { return queue_.capacity(); }
    std::size_t high_water() const noexcept { return high_water_; }
    std::uint64_t dropped() const noexcept { return dropped_; }
private:
    runtime::SpscRing<T> queue_;
    std::atomic<CameraImuSyncError>& failure_;
    CameraImuSyncError overflow_;
    std::size_t high_water_ = 0;
    std::uint64_t dropped_ = 0;
};
}  // namespace mosas::app::detail
#endif
