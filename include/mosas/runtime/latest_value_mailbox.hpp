#ifndef MOSAS_RUNTIME_LATEST_VALUE_MAILBOX_HPP
#define MOSAS_RUNTIME_LATEST_VALUE_MAILBOX_HPP

#include <atomic>
#include <cstdint>

namespace mosas::runtime {

// 单生产者、单消费者的最新值槽；新值发布时覆盖尚未消费的旧值。
template <typename T>
class LatestValueMailbox final {
public:
    LatestValueMailbox() = default;

    LatestValueMailbox(const LatestValueMailbox&) = delete;
    LatestValueMailbox& operator=(const LatestValueMailbox&) = delete;

    bool try_publish(const T& value) noexcept {
        if (closed_.load(std::memory_order_acquire)) {
            return false;
        }
        const std::uint64_t published =
            published_sequence_.load(std::memory_order_relaxed);
        if (published != consumed_sequence_.load(std::memory_order_acquire)) {
            dropped_count_.fetch_add(1, std::memory_order_relaxed);
        }
        value_.store(value, std::memory_order_release);
        published_sequence_.store(published + 1,
                                  std::memory_order_release);
        return true;
    }

    bool try_consume(T* value) noexcept {
        if (value == nullptr) {
            return false;
        }
        const std::uint64_t published =
            published_sequence_.load(std::memory_order_acquire);
        const std::uint64_t consumed =
            consumed_sequence_.load(std::memory_order_relaxed);
        if (published == consumed) {
            return false;
        }
        *value = value_.load(std::memory_order_acquire);
        consumed_sequence_.store(published, std::memory_order_release);
        return true;
    }

    void close() noexcept { closed_.store(true, std::memory_order_release); }

    // 仅允许在生产者和消费者均停止后调用。
    void reset() noexcept {
        value_.store(T{}, std::memory_order_relaxed);
        published_sequence_.store(0, std::memory_order_relaxed);
        consumed_sequence_.store(0, std::memory_order_relaxed);
        dropped_count_.store(0, std::memory_order_relaxed);
        closed_.store(false, std::memory_order_release);
    }

    std::uint64_t dropped_count() const noexcept {
        return dropped_count_.load(std::memory_order_relaxed);
    }

private:
    std::atomic<T> value_{T{}};
    std::atomic<std::uint64_t> published_sequence_{0};
    std::atomic<std::uint64_t> consumed_sequence_{0};
    std::atomic<std::uint64_t> dropped_count_{0};
    std::atomic<bool> closed_{false};
};

}  // namespace mosas::runtime

#endif  // MOSAS_RUNTIME_LATEST_VALUE_MAILBOX_HPP
