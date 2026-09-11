#ifndef MOSAS_CONTROL_MAILBOX_HPP
#define MOSAS_CONTROL_MAILBOX_HPP

#include <mosas/runtime/control_command.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <mutex>

namespace mosas::runtime {

// Single-producer/single-consumer latest-value mailbox for continuous control
// setpoints. Every slot field is atomic because the producer may overwrite it
// while the consumer is taking a snapshot. The producer never waits for the
// consumer or for device I/O.
class ControlMailbox {
public:
    enum class ReceiveResult {
        empty,
        command,
        closed,
    };

    ControlMailbox() = default;

    ControlMailbox(const ControlMailbox&) = delete;
    ControlMailbox& operator=(const ControlMailbox&) = delete;

    // Returns false when the mailbox is closed or the sequence is not newer
    // than the currently published sequence.
    bool submit(const ControlCommand& command) noexcept {
        if (closed_.load(std::memory_order_acquire) || command.sequence == 0) {
            return false;
        }

        const std::uint64_t previous_sequence =
            published_sequence_.load(std::memory_order_relaxed);
        if (command.sequence <= previous_sequence) {
            return false;
        }

        // The sequentially-consistent version markers make the payload read
        // interval explicit without taking a mutex or exposing non-atomic
        // storage to the two workers.
        write_version_.fetch_add(1, std::memory_order_seq_cst);
        timestamp_ns_.store(command.timestamp_ns, std::memory_order_seq_cst);
        overload_y_bits_.store(float_to_bits(command.overload_y),
                               std::memory_order_seq_cst);
        overload_z_bits_.store(float_to_bits(command.overload_z),
                               std::memory_order_seq_cst);
        flags_.store(command.flags, std::memory_order_seq_cst);
        published_sequence_.store(command.sequence,
                                  std::memory_order_seq_cst);
        write_version_.fetch_add(1, std::memory_order_seq_cst);

        const std::uint64_t consumed_sequence =
            consumed_sequence_.load(std::memory_order_acquire);
        if (previous_sequence != 0 &&
            previous_sequence != consumed_sequence) {
            superseded_count_.fetch_add(1, std::memory_order_relaxed);
        }
        update_condition_.notify_one();
        return true;
    }

    // Returns the newest stable command. The producer version is odd while
    // payload writes are in progress; any version change during the snapshot
    // causes a retry, so an overlapping overwrite cannot produce a mixed
    // command.
    ReceiveResult receive_latest(ControlCommand* command) noexcept {
        if (command == nullptr) {
            return ReceiveResult::empty;
        }

        constexpr int kSnapshotAttempts = 8;
        for (int attempt = 0; attempt < kSnapshotAttempts; ++attempt) {
            const std::uint64_t version =
                write_version_.load(std::memory_order_seq_cst);
            if ((version & 1U) != 0) {
                continue;
            }
            const std::uint64_t sequence =
                published_sequence_.load(std::memory_order_seq_cst);
            const std::uint64_t consumed_sequence =
                consumed_sequence_.load(std::memory_order_relaxed);
            if (sequence == 0 || sequence <= consumed_sequence) {
                return closed_.load(std::memory_order_acquire)
                           ? ReceiveResult::closed
                           : ReceiveResult::empty;
            }

            const std::int64_t timestamp_ns =
                timestamp_ns_.load(std::memory_order_seq_cst);
            const float overload_y =
                bits_to_float(overload_y_bits_.load(std::memory_order_seq_cst));
            const float overload_z =
                bits_to_float(overload_z_bits_.load(std::memory_order_seq_cst));
            const std::uint32_t flags =
                flags_.load(std::memory_order_seq_cst);
            const std::uint64_t sequence_after_payload =
                published_sequence_.load(std::memory_order_seq_cst);
            const std::uint64_t version_after =
                write_version_.load(std::memory_order_seq_cst);
            if (sequence != sequence_after_payload ||
                version != version_after || (version_after & 1U) != 0) {
                continue;
            }

            // Acknowledge before copying into the caller's private object so
            // a concurrent submit does not count a command already captured
            // by this consumer as still pending.
            consumed_sequence_.store(sequence, std::memory_order_release);
            command->sequence = sequence;
            command->timestamp_ns = timestamp_ns;
            command->overload_y = overload_y;
            command->overload_z = overload_z;
            command->flags = flags;
            return ReceiveResult::command;
        }

        return closed_.load(std::memory_order_acquire)
                   ? ReceiveResult::closed
                   : ReceiveResult::empty;
    }

    // A timed wait keeps submit lock-free with respect to the consumer. The
    // timeout also bounds a notification race because submit does not take
    // wait_mutex_.
    bool wait_for_update(std::chrono::milliseconds timeout) {
        std::unique_lock<std::mutex> lock(wait_mutex_);
        const auto has_update = [this] {
            return closed_.load(std::memory_order_acquire) ||
                   published_sequence_.load(std::memory_order_acquire) >
                       consumed_sequence_.load(std::memory_order_acquire);
        };
        if (has_update()) {
            return true;
        }
        return update_condition_.wait_for(lock, timeout, has_update);
    }

    void close() noexcept {
        closed_.store(true, std::memory_order_release);
        update_condition_.notify_all();
    }

    // reset() is called only after all users from the previous run have
    // stopped. It intentionally does not attempt to coordinate live workers.
    void reset() noexcept {
        std::lock_guard<std::mutex> lock(wait_mutex_);
        timestamp_ns_.store(-1, std::memory_order_relaxed);
        overload_y_bits_.store(0, std::memory_order_relaxed);
        overload_z_bits_.store(0, std::memory_order_relaxed);
        flags_.store(0, std::memory_order_relaxed);
        published_sequence_.store(0, std::memory_order_relaxed);
        consumed_sequence_.store(0, std::memory_order_relaxed);
        superseded_count_.store(0, std::memory_order_relaxed);
        write_version_.store(0, std::memory_order_relaxed);
        closed_.store(false, std::memory_order_relaxed);
    }

    bool closed() const noexcept {
        return closed_.load(std::memory_order_acquire);
    }

    std::uint64_t superseded_count() const noexcept {
        return superseded_count_.load(std::memory_order_relaxed);
    }

private:
    static std::uint32_t float_to_bits(float value) noexcept {
        std::uint32_t bits = 0;
        static_assert(sizeof(bits) == sizeof(value));
        std::memcpy(&bits, &value, sizeof(bits));
        return bits;
    }

    static float bits_to_float(std::uint32_t bits) noexcept {
        float value = 0.0F;
        static_assert(sizeof(bits) == sizeof(value));
        std::memcpy(&value, &bits, sizeof(value));
        return value;
    }

    std::atomic<std::uint64_t> published_sequence_{0};
    // Even means stable; odd means the producer is updating the logical slot.
    std::atomic<std::uint64_t> write_version_{0};
    std::atomic<std::uint64_t> consumed_sequence_{0};
    std::atomic<std::int64_t> timestamp_ns_{-1};
    std::atomic<std::uint32_t> overload_y_bits_{0};
    std::atomic<std::uint32_t> overload_z_bits_{0};
    std::atomic<std::uint32_t> flags_{0};
    std::atomic<std::uint64_t> superseded_count_{0};
    std::atomic<bool> closed_{false};
    std::mutex wait_mutex_;
    std::condition_variable update_condition_;
};

}  // namespace mosas::runtime

#endif  // MOSAS_CONTROL_MAILBOX_HPP
