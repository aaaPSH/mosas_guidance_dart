#ifndef MOSAS_RUNTIME_IMU_ANGULAR_VELOCITY_HISTORY_HPP
#define MOSAS_RUNTIME_IMU_ANGULAR_VELOCITY_HISTORY_HPP

#include <mosas/runtime/imu_angular_velocity.hpp>

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>

namespace mosas::runtime {

struct ImuAngularVelocityLookup {
    ImuAngularVelocitySample sample{};
    bool interpolated = false;
    TimestampNs lower_timestamp_ns = -1;
    TimestampNs upper_timestamp_ns = -1;
};

// 单写入者、多读取者的固定容量角速度历史；写入后不分配、不加锁。
class ImuAngularVelocityHistory final {
public:
    explicit ImuAngularVelocityHistory(std::size_t capacity)
        : capacity_(capacity), slots_(std::make_unique<Slot[]>(capacity)) {
        if (capacity_ == 0) {
            throw std::invalid_argument(
                "ImuAngularVelocityHistory capacity must be positive");
        }
    }

    ImuAngularVelocityHistory(const ImuAngularVelocityHistory&) = delete;
    ImuAngularVelocityHistory& operator=(const ImuAngularVelocityHistory&) =
        delete;

    bool push(const ImuAngularVelocitySample& sample) noexcept;

    std::optional<ImuAngularVelocityLookup> find_interpolated(
        TimestampNs timestamp_ns) const noexcept;

    std::size_t size() const noexcept {
        return size_.load(std::memory_order_acquire);
    }

    std::size_t capacity() const noexcept { return capacity_; }

    // 仅允许在写入者和读取者均停止时调用。
    void reset() noexcept;

private:
    struct Slot {
        std::atomic<std::uint64_t> sequence{0};
        std::atomic<TimestampNs> timestamp_ns{-1};
        std::atomic<std::uint64_t> angular_velocity_x_bits{0};
        std::atomic<std::uint64_t> angular_velocity_y_bits{0};
        std::atomic<std::uint64_t> angular_velocity_z_bits{0};
    };

    bool read_slot(std::size_t index,
                   ImuAngularVelocitySample* sample) const noexcept;

    const std::size_t capacity_;
    std::unique_ptr<Slot[]> slots_;
    std::size_t next_index_ = 0;
    std::size_t size_writer_ = 0;
    TimestampNs last_timestamp_ns_ = -1;
    bool has_timestamp_ = false;
    std::atomic<std::size_t> size_{0};
};

}  // namespace mosas::runtime

#endif  // MOSAS_RUNTIME_IMU_ANGULAR_VELOCITY_HISTORY_HPP
