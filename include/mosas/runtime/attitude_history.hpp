#ifndef MOSAS_RUNTIME_ATTITUDE_HISTORY_HPP
#define MOSAS_RUNTIME_ATTITUDE_HISTORY_HPP

#include <mosas/runtime/clock.hpp>
#include <mosas/runtime/rotation_matrix3.hpp>

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>

namespace mosas::runtime {

/** @brief 同一 IMU 时刻的姿态、纵向速度（m/s）和起飞状态。 */
struct AttitudeSample {
    TimestampNs timestamp_ns = -1;
    double pitch_rad = 0.0;
    double roll_rad = 0.0;
    double speed_mps = 0.0;
    RotationMatrix3 body_to_navigation{
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0};
    bool launched = false;
    TimestampNs launch_timestamp_ns = -1;
    std::uint64_t launch_generation = 0;
};

/** @brief 精确匹配或同一 IMU 区间内插值的姿态与纵向速度（m/s）。 */
struct AttitudeLookup {
    double pitch_rad = 0.0;
    double roll_rad = 0.0;
    double speed_mps = 0.0;
    RotationMatrix3 body_to_navigation{
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0};
    bool interpolated = false;
    TimestampNs lower_timestamp_ns = -1;
    TimestampNs upper_timestamp_ns = -1;
    bool launched = false;
    TimestampNs launch_timestamp_ns = -1;
    std::uint64_t launch_generation = 0;
};

/** @brief 单写入者、多读取者的固定容量姿态/速度历史；不外推。 */
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

    /**
     * @brief 写入同一时刻的完整姿态/速度状态。
     * @param sample 时间戳严格递增、姿态与速度有限的 IMU 状态。
     * @return 写入成功时为 true；无效输入不改变历史。
     */
    bool push(const AttitudeSample& sample) noexcept;

    /**
     * @brief 按时间匹配姿态并线性插值速度，不跨起飞状态或代际边界。
     * @param timestamp_ns 查询时刻，单位 ns。
     * @return 同步状态；没有完整匹配区间时返回空值，不做外推。
     */
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
        std::atomic<std::uint64_t> roll_bits{0};
        std::atomic<std::uint64_t> speed_bits{0};
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
        std::atomic<TimestampNs> launch_timestamp_ns{-1};
        std::atomic<std::uint64_t> launch_generation{0};
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
