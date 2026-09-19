#ifndef MOSAS_FRAME_RATE_METER_HPP
#define MOSAS_FRAME_RATE_METER_HPP

#include <chrono>
#include <deque>
#include <mutex>

namespace mosas::runtime {

// 使用最近一秒样本的时间间隔计算速率；读取不会清空样本。
class FrameRateMeter {
public:
    void reset() {
        std::lock_guard<std::mutex> lock(mutex_);
        samples_.clear();
    }

    void record() {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto now = std::chrono::steady_clock::now();
        samples_.push_back(now);
        remove_expired(now);
    }

    double snapshot() const {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto now = std::chrono::steady_clock::now();
        remove_expired(now);
        if (samples_.size() < 2) {
            return 0.0;
        }
        const double elapsed_seconds = std::chrono::duration<double>(
            samples_.back() - samples_.front()).count();
        if (elapsed_seconds <= 0.0) {
            return 0.0;
        }
        return static_cast<double>(samples_.size() - 1) / elapsed_seconds;
    }

private:
    void remove_expired(
        std::chrono::steady_clock::time_point now) const {
        const auto window = std::chrono::seconds(1);
        while (!samples_.empty() && now - samples_.front() > window) {
            samples_.pop_front();
        }
    }

    mutable std::mutex mutex_;
    mutable std::deque<std::chrono::steady_clock::time_point> samples_;
};

}  // namespace mosas::runtime

#endif  // MOSAS_FRAME_RATE_METER_HPP
