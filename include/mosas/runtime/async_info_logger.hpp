#ifndef MOSAS_RUNTIME_ASYNC_INFO_LOGGER_HPP
#define MOSAS_RUNTIME_ASYNC_INFO_LOGGER_HPP

#include <mosas/runtime/log_event.hpp>
#include <mosas/runtime/output_ports.hpp>
#include <mosas/runtime/runtime_types.hpp>
#include <mosas/runtime/spsc_ring.hpp>

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <iosfwd>
#include <mutex>
#include <string>
#include <thread>

namespace mosas::runtime {

class AsyncInfoLogger final : public IAsyncLogger {
public:
    explicit AsyncInfoLogger(std::ostream& output,
                             std::size_t queue_capacity = 256);
    ~AsyncInfoLogger();

    AsyncInfoLogger(const AsyncInfoLogger&) = delete;
    AsyncInfoLogger& operator=(const AsyncInfoLogger&) = delete;

    bool start() override;
    bool try_submit(const RuntimeLogEvent& event) noexcept override;
    bool try_submit_text(TimestampNs timestamp_ns,
                         const std::string& message) noexcept;
    SinkDiagnostics diagnostics() const noexcept override;

    bool running() const noexcept {
        return running_.load(std::memory_order_acquire);
    }

    void stop_and_flush() noexcept override;

private:
    void worker_loop() noexcept;
    void set_error(const char* message) noexcept;

    std::ostream* output_ = nullptr;
    // Submissions can originate from frame, IMU, and output workers.
    // Serialize enqueues so the underlying ring remains single-producer.
    std::mutex producer_mutex_;
    SpscRing<RuntimeLogEvent> queue_;
    std::atomic<bool> stop_requested_{false};
    std::atomic<bool> running_{false};
    std::atomic<std::uint64_t> accepted_count_{0};
    std::atomic<std::uint64_t> dropped_count_{0};
    std::atomic<std::uint64_t> failure_count_{0};
    std::atomic<bool> healthy_{true};
    mutable std::mutex error_mutex_;
    std::string last_error_;
    std::thread worker_;
};

}  // namespace mosas::runtime

#endif  // MOSAS_RUNTIME_ASYNC_INFO_LOGGER_HPP
