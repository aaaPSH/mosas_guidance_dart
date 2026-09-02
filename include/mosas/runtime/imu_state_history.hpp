#ifndef MOSAS_IMU_STATE_HISTORY_HPP
#define MOSAS_IMU_STATE_HISTORY_HPP

#include <mosas/runtime/runtime_types.hpp>

#include <cstddef>
#include <deque>
#include <mutex>
#include <optional>

namespace mosas::runtime {

class ImuStateHistory {
public:
    explicit ImuStateHistory(std::size_t capacity);

    bool push(const ImuStateSnapshot& snapshot);
    std::optional<ImuStateSnapshot> find_at_or_before(
        TimestampNs timestamp_ns) const;
    std::size_t size() const;
    void clear();

private:
    const std::size_t capacity_;
    mutable std::mutex mutex_;
    std::deque<ImuStateSnapshot> snapshots_;
};

}  // namespace mosas::runtime

#endif  // MOSAS_IMU_STATE_HISTORY_HPP
