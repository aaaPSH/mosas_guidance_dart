#ifndef MOSAS_RUNTIME_SPSC_RING_HPP
#define MOSAS_RUNTIME_SPSC_RING_HPP

#include <atomic>
#include <cstddef>
#include <memory>
#include <stdexcept>
#include <utility>

namespace mosas::runtime {

// 单生产者、单消费者固定容量环形队列；构造完成后不分配内存、不加锁。
template <typename T>
class SpscRing {
public:
    static_assert(std::atomic<std::size_t>::is_always_lock_free,
                  "SpscRing requires lock-free size_t atomics");

    explicit SpscRing(std::size_t capacity)
        : capacity_(capacity), storage_(std::make_unique<T[]>(capacity)) {
        if (capacity_ == 0) {
            throw std::invalid_argument("SpscRing capacity must be positive");
        }
    }

    SpscRing(const SpscRing&) = delete;
    SpscRing& operator=(const SpscRing&) = delete;

    bool try_push(const T& value) noexcept {
        return try_push_impl(value);
    }

    bool try_push(T&& value) noexcept {
        return try_push_impl(std::move(value));
    }

    bool try_pop(T* value) noexcept {
        if (value == nullptr) {
            return false;
        }

        const std::size_t tail = tail_.load(std::memory_order_relaxed);
        const std::size_t head = head_.load(std::memory_order_acquire);
        if (tail == head) {
            return false;
        }

        *value = std::move(storage_[tail % capacity_]);
        tail_.store(tail + 1, std::memory_order_release);
        return true;
    }

    bool empty() const noexcept {
        return head_.load(std::memory_order_acquire) ==
               tail_.load(std::memory_order_acquire);
    }

    // 仅允许在生产者和消费者均已停止时调用。
    void reset() noexcept {
        head_.store(0, std::memory_order_relaxed);
        tail_.store(0, std::memory_order_relaxed);
    }

    std::size_t size() const noexcept {
        const std::size_t head = head_.load(std::memory_order_acquire);
        const std::size_t tail = tail_.load(std::memory_order_acquire);
        const std::size_t distance = head - tail;
        return distance > capacity_ ? capacity_ : distance;
    }

    std::size_t capacity() const noexcept { return capacity_; }

private:
    template <typename U>
    bool try_push_impl(U&& value) noexcept {
        const std::size_t head = head_.load(std::memory_order_relaxed);
        const std::size_t tail = tail_.load(std::memory_order_acquire);
        if (head - tail >= capacity_) {
            return false;
        }

        storage_[head % capacity_] = std::forward<U>(value);
        head_.store(head + 1, std::memory_order_release);
        return true;
    }

    const std::size_t capacity_;
    std::unique_ptr<T[]> storage_;
    alignas(64) std::atomic<std::size_t> head_{0};
    alignas(64) std::atomic<std::size_t> tail_{0};
};

}  // namespace mosas::runtime

#endif  // MOSAS_RUNTIME_SPSC_RING_HPP
