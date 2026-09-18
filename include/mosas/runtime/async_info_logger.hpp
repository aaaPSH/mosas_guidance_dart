#ifndef MOSAS_RUNTIME_ASYNC_INFO_LOGGER_HPP
#define MOSAS_RUNTIME_ASYNC_INFO_LOGGER_HPP

#include <mosas/runtime/runtime_types.hpp>
#include <mosas/runtime/spsc_ring.hpp>

#include <atomic>
#include <array>
#include <cstddef>
#include <cstdint>
#include <iosfwd>
#include <string>
#include <thread>

namespace mosas::runtime {

enum class RuntimeLogEventKind {
    guidance_command,
    diagnostic,
    text,
};

struct RuntimeLogEvent {
    RuntimeLogEventKind kind = RuntimeLogEventKind::diagnostic;
    TimestampNs timestamp_ns = -1;
    std::uint64_t count = 0;
    double value0 = 0.0;
    double value1 = 0.0;
    double value2 = 0.0;
    std::array<char, 192> text{};
};

class AsyncInfoLogger final {
public:
    explicit AsyncInfoLogger(std::ostream& output,
                             std::size_t queue_capacity = 256);
    ~AsyncInfoLogger();

    AsyncInfoLogger(const AsyncInfoLogger&) = delete;
    AsyncInfoLogger& operator=(const AsyncInfoLogger&) = delete;

    bool start();
    bool try_submit(const RuntimeLogEvent& event) noexcept;
    bool try_submit_text(TimestampNs timestamp_ns,
                         const std::string& message) noexcept;

    bool running() const noexcept {
        return running_.load(std::memory_order_acquire);
    }

    void stop() noexcept;

    std::uint64_t dropped_count() const noexcept {
        return dropped_count_.load(std::memory_order_relaxed);
    }

private:
    void worker_loop() noexcept;

    std::ostream* output_ = nullptr;
    SpscRing<RuntimeLogEvent> queue_;
    std::atomic<bool> stop_requested_{false};
    std::atomic<bool> running_{false};
    std::atomic<std::uint64_t> dropped_count_{0};
    std::thread worker_;
};

}  // namespace mosas::runtime

#endif  // MOSAS_RUNTIME_ASYNC_INFO_LOGGER_HPP
