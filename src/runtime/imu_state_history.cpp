#include <mosas/runtime/imu_state_history.hpp>

namespace mosas::runtime {

ImuStateHistory::ImuStateHistory(std::size_t capacity) : capacity_(capacity) {}

bool ImuStateHistory::push(const ImuStateSnapshot& snapshot) {
    if (snapshot.timestamp_ns < 0) {
        return false;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (capacity_ == 0 ||
        (!snapshots_.empty() &&
         snapshot.timestamp_ns <= snapshots_.back().timestamp_ns)) {
        return false;
    }

    snapshots_.push_back(snapshot);
    while (snapshots_.size() > capacity_) {
        snapshots_.pop_front();
    }
    return true;
}

std::optional<ImuStateSnapshot> ImuStateHistory::find_at_or_before(
    TimestampNs timestamp_ns) const {
    if (timestamp_ns < 0) {
        return std::nullopt;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    for (auto iterator = snapshots_.rbegin(); iterator != snapshots_.rend();
         ++iterator) {
        if (iterator->timestamp_ns <= timestamp_ns) {
            return *iterator;
        }
    }
    return std::nullopt;
}

std::size_t ImuStateHistory::size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return snapshots_.size();
}

void ImuStateHistory::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    snapshots_.clear();
}

}  // namespace mosas::runtime
