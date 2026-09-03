#ifndef MOSAS_LATEST_FRAME_QUEUE_HPP
#define MOSAS_LATEST_FRAME_QUEUE_HPP

#include <condition_variable>
#include <cstddef>
#include <deque>
#include <mutex>
#include <utility>

namespace mosas::runtime {

// 线程安全的有界最新帧队列：默认只保留一个待处理对象。
template <typename T>
class LatestFrameQueue {
public:
    explicit LatestFrameQueue(std::size_t capacity = 1)
        : capacity_(capacity == 0 ? 1 : capacity) {}

    LatestFrameQueue(const LatestFrameQueue&) = delete;
    LatestFrameQueue& operator=(const LatestFrameQueue&) = delete;

    bool push(T value) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (closed_) {
                return false;
            }
            if (items_.size() >= capacity_) {
                ++dropped_count_;
                items_.pop_front();
            }
            items_.push_back(std::move(value));
        }
        condition_.notify_one();
        return true;
    }

    bool pop(T* value) {
        if (value == nullptr) {
            return false;
        }

        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait(lock, [this] {
            return closed_ || !items_.empty();
        });
        if (items_.empty()) {
            return false;
        }

        *value = std::move(items_.front());
        items_.pop_front();
        return true;
    }

    void close() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            closed_ = true;
        }
        condition_.notify_all();
    }

    void reset() {
        std::lock_guard<std::mutex> lock(mutex_);
        items_.clear();
        dropped_count_ = 0;
        closed_ = false;
    }

    std::size_t dropped_count() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return dropped_count_;
    }

private:
    mutable std::mutex mutex_;
    std::condition_variable condition_;
    const std::size_t capacity_;
    std::deque<T> items_;
    std::size_t dropped_count_ = 0;
    bool closed_ = false;
};

}  // namespace mosas::runtime

#endif  // MOSAS_LATEST_FRAME_QUEUE_HPP
