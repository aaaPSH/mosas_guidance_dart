#ifndef MOSAS_RUNTIME_ATTITUDE_HISTORY_HPP
#define MOSAS_RUNTIME_ATTITUDE_HISTORY_HPP

#include <mosas/guidance/guidance_estimator.hpp>
#include <mosas/runtime/clock.hpp>

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>

namespace mosas::runtime {

struct AttitudeSample {
    TimestampNs timestamp_ns = -1;
    double pitch_rad = 0.0;
    RotationMatrix3 body_to_navigation{
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0};
    bool launched = false;
};

struct AttitudeLookup {
    double pitch_rad = 0.0;
    RotationMatrix3 body_to_navigation{
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0};
    bool interpolated = false;
    TimestampNs lower_timestamp_ns = -1;
    TimestampNs upper_timestamp_ns = -1;
    bool launched = false;
};

// 单写入者、多读取者的固定容量姿态历史；不外推。
class AttitudeHistory final {
public:
    explicit AttitudeHistory(std::size_t capacity)
        : capacity_(capacity), slots_(std::make_unique<Slot[]>(capacity)) {
        if (capacity_ == 0) {
            throw std::invalid_argument(
                "AttitudeHistory capacity must be positive");
        }
    }

    AttitudeHistory(const AttitudeHistory&) = delete;
    AttitudeHistory& operator=(const AttitudeHistory&) = delete;

    bool push(const AttitudeSample& sample) noexcept;

    std::optional<AttitudeLookup> find_interpolated(
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
        std::atomic<std::uint64_t> pitch_bits{0};
        std::atomic<std::uint64_t> matrix_m00_bits{0};
        std::atomic<std::uint64_t> matrix_m01_bits{0};
        std::atomic<std::uint64_t> matrix_m02_bits{0};
        std::atomic<std::uint64_t> matrix_m10_bits{0};
        std::atomic<std::uint64_t> matrix_m11_bits{0};
        std::atomic<std::uint64_t> matrix_m12_bits{0};
        std::atomic<std::uint64_t> matrix_m20_bits{0};
        std::atomic<std::uint64_t> matrix_m21_bits{0};
        std::atomic<std::uint64_t> matrix_m22_bits{0};
        std::atomic<bool> launched{false};
    };

    static std::uint64_t double_bits(double value) noexcept;
    static double bits_double(std::uint64_t bits) noexcept;
    bool read_slot(std::size_t index, AttitudeSample* sample) const noexcept;

    const std::size_t capacity_;
    std::unique_ptr<Slot[]> slots_;
    std::size_t next_index_ = 0;
    std::size_t size_writer_ = 0;
    TimestampNs last_timestamp_ns_ = -1;
    bool has_timestamp_ = false;
    std::atomic<std::size_t> size_{0};
};

}  // namespace mosas::runtime

#endif  // MOSAS_RUNTIME_ATTITUDE_HISTORY_HPP
