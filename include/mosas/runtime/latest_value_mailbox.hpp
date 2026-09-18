#ifndef MOSAS_RUNTIME_LATEST_VALUE_MAILBOX_HPP
#define MOSAS_RUNTIME_LATEST_VALUE_MAILBOX_HPP

#include <array>
#include <atomic>
#include <cstdint>
#include <limits>
#include <type_traits>

namespace mosas::runtime {

// 单生产者、单消费者的最新值槽；固定双槽、无锁状态转换，不分配内存。
// 当消费者来不及处理时，生产者只覆盖尚未被消费者占用的旧值。
template <typename T>
class LatestValueMailbox final {
public:
    static_assert(std::is_trivially_copyable_v<T>,
                  "LatestValueMailbox requires trivially copyable values");
    static_assert(std::atomic<std::uint8_t>::is_always_lock_free,
                  "LatestValueMailbox requires lock-free byte atomics");
    static_assert(std::atomic<std::uint64_t>::is_always_lock_free,
                  "LatestValueMailbox requires lock-free sequence atomics");
    static_assert(std::atomic<bool>::is_always_lock_free,
                  "LatestValueMailbox requires lock-free bool atomics");

    LatestValueMailbox() = default;

    LatestValueMailbox(const LatestValueMailbox&) = delete;
    LatestValueMailbox& operator=(const LatestValueMailbox&) = delete;

    bool is_lock_free() const noexcept { return true; }

    bool try_publish(const T& value) noexcept {
        if (closed_.load(std::memory_order_acquire)) {
            return false;
        }

        const std::uint64_t sequence = next_sequence_ + 1;
        const std::size_t slot_index = acquire_slot_for_publish();
        if (slot_index == kSlotCount) {
            dropped_count_.fetch_add(1, std::memory_order_relaxed);
            return false;
        }

        Slot& slot = slots_[slot_index];
        slot.value = value;
        slot.sequence.store(sequence, std::memory_order_relaxed);
        slot.state.store(kPublished, std::memory_order_release);
        next_sequence_ = sequence;
        return true;
    }

    bool try_consume(T* value) noexcept {
        if (value == nullptr) {
            return false;
        }

        for (std::size_t attempt = 0; attempt < kSlotCount; ++attempt) {
            const std::size_t slot_index = newest_published_slot();
            if (slot_index == kSlotCount) {
                return false;
            }

            Slot& slot = slots_[slot_index];
            std::uint8_t expected = kPublished;
            if (!slot.state.compare_exchange_weak(
                    expected, kReading, std::memory_order_acquire,
                    std::memory_order_relaxed)) {
                continue;
            }

            const std::uint64_t consumed_sequence =
                slot.sequence.load(std::memory_order_relaxed);
            if (has_newer_published_slot(slot_index, consumed_sequence)) {
                slot.state.store(kFree, std::memory_order_release);
                dropped_count_.fetch_add(1, std::memory_order_relaxed);
                continue;
            }

            *value = slot.value;
            slot.state.store(kFree, std::memory_order_release);
            discard_older_published_slots(slot_index, consumed_sequence);
            return true;
        }
        return false;
    }

    void close() noexcept { closed_.store(true, std::memory_order_release); }

    // 仅允许在生产者和消费者均停止后调用。
    void reset() noexcept {
        for (Slot& slot : slots_) {
            slot.state.store(kFree, std::memory_order_relaxed);
            slot.sequence.store(0, std::memory_order_relaxed);
            slot.value = T{};
        }
        next_sequence_ = 0;
        dropped_count_.store(0, std::memory_order_relaxed);
        closed_.store(false, std::memory_order_release);
    }

    std::uint64_t dropped_count() const noexcept {
        return dropped_count_.load(std::memory_order_relaxed);
    }

private:
    static constexpr std::size_t kSlotCount = 2;
    static constexpr std::uint8_t kFree = 0;
    static constexpr std::uint8_t kWriting = 1;
    static constexpr std::uint8_t kPublished = 2;
    static constexpr std::uint8_t kReading = 3;

    struct Slot {
        std::atomic<std::uint8_t> state{kFree};
        std::atomic<std::uint64_t> sequence{0};
        T value{};
    };

    std::size_t acquire_slot_for_publish() noexcept {
        for (std::size_t index = 0; index < kSlotCount; ++index) {
            std::uint8_t expected = kFree;
            if (slots_[index].state.compare_exchange_weak(
                    expected, kWriting, std::memory_order_acquire,
                    std::memory_order_relaxed)) {
                return index;
            }
        }

        std::size_t oldest_index = kSlotCount;
        std::uint64_t oldest_sequence =
            std::numeric_limits<std::uint64_t>::max();
        for (std::size_t index = 0; index < kSlotCount; ++index) {
            if (slots_[index].state.load(std::memory_order_acquire) !=
                kPublished) {
                continue;
            }
            const std::uint64_t sequence =
                slots_[index].sequence.load(std::memory_order_relaxed);
            if (oldest_index == kSlotCount || sequence < oldest_sequence) {
                oldest_index = index;
                oldest_sequence = sequence;
            }
        }
        if (oldest_index == kSlotCount) {
            return kSlotCount;
        }

        std::uint8_t expected = kPublished;
        if (!slots_[oldest_index].state.compare_exchange_strong(
                expected, kWriting, std::memory_order_acquire,
                std::memory_order_relaxed)) {
            return kSlotCount;
        }
        dropped_count_.fetch_add(1, std::memory_order_relaxed);
        return oldest_index;
    }

    std::size_t newest_published_slot() const noexcept {
        std::size_t newest_index = kSlotCount;
        std::uint64_t newest_sequence = 0;
        for (std::size_t index = 0; index < kSlotCount; ++index) {
            if (slots_[index].state.load(std::memory_order_acquire) !=
                kPublished) {
                continue;
            }
            const std::uint64_t sequence =
                slots_[index].sequence.load(std::memory_order_relaxed);
            if (newest_index == kSlotCount || sequence > newest_sequence) {
                newest_index = index;
                newest_sequence = sequence;
            }
        }
        return newest_index;
    }

    bool has_newer_published_slot(std::size_t consumed_index,
                                  std::uint64_t consumed_sequence) const noexcept {
        for (std::size_t index = 0; index < kSlotCount; ++index) {
            if (index == consumed_index ||
                slots_[index].state.load(std::memory_order_acquire) !=
                    kPublished) {
                continue;
            }
            if (slots_[index].sequence.load(std::memory_order_relaxed) >
                consumed_sequence) {
                return true;
            }
        }
        return false;
    }

    void discard_older_published_slots(
        std::size_t consumed_index,
        std::uint64_t consumed_sequence) noexcept {
        for (std::size_t index = 0; index < kSlotCount; ++index) {
            if (index == consumed_index) {
                continue;
            }
            if (slots_[index].state.load(std::memory_order_acquire) !=
                    kPublished ||
                slots_[index].sequence.load(std::memory_order_relaxed) >
                    consumed_sequence) {
                continue;
            }
            std::uint8_t expected = kPublished;
            if (slots_[index].state.compare_exchange_strong(
                    expected, kFree, std::memory_order_acquire,
                    std::memory_order_relaxed)) {
                dropped_count_.fetch_add(1, std::memory_order_relaxed);
            }
        }
    }

    std::array<Slot, kSlotCount> slots_{};
    std::uint64_t next_sequence_ = 0;
    std::atomic<std::uint64_t> dropped_count_{0};
    std::atomic<bool> closed_{false};
};

}  // namespace mosas::runtime

#endif  // MOSAS_RUNTIME_LATEST_VALUE_MAILBOX_HPP
