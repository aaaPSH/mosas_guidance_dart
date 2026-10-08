#ifndef MOSAS_ADAPTERS_HOST_ADAPTERS_HPP
#define MOSAS_ADAPTERS_HOST_ADAPTERS_HPP

#include <mosas/app/app_config.hpp>
#include <mosas/runtime/async_info_logger.hpp>
#include <mosas/runtime/latest_value_mailbox.hpp>
#include <serial_package/serial_port.hpp>

#include <chrono>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <iosfwd>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace mosas::app {

class SimulatedImuSource final : public runtime::ImuSource {
public:
    explicit SimulatedImuSource(double sample_rate_hz,
                                double gravity_mps2 = 9.80665,
                                bool accelerometer_specific_force = true);

    runtime::SourceResult start() override;
    bool try_pop(runtime::ImuAngularVelocitySample* sample) noexcept override;
    bool wait_for_data() noexcept override;
    void cancel() noexcept override;
    void stop() noexcept override;

private:
    std::chrono::nanoseconds period_;
    double gravity_mps2_ = 9.80665;
    bool accelerometer_specific_force_ = true;
    std::mutex mutex_;
    std::condition_variable condition_;
    std::chrono::steady_clock::time_point next_deadline_{};
    bool started_ = false;
    bool cancelled_ = false;
};

class LoggingCommandSink final : public runtime::CommandSink {
public:
    LoggingCommandSink();
    explicit LoggingCommandSink(std::ostream& output);

    bool start() override;
    bool try_submit(const runtime::GuidanceCommand& command) noexcept override;
    void stop() noexcept override;
    Diagnostics diagnostics() const noexcept override;

private:
    runtime::AsyncInfoLogger logger_;
    std::atomic<std::uint64_t> submitted_count_{0};
};

class SerialCommandSink final : public runtime::CommandSink {
public:
    explicit SerialCommandSink(
        std::shared_ptr<serial_package::SerialPort> port);
    SerialCommandSink(std::shared_ptr<serial_package::SerialPort> port,
                      std::ostream& output);
    ~SerialCommandSink() override;

    bool start() override;
    bool try_submit(const runtime::GuidanceCommand& command) noexcept override;
    void set_active_generation(std::uint64_t generation) noexcept override;
    void cancel() noexcept override;
    void stop() noexcept override;
    Diagnostics diagnostics() const noexcept override;

    std::string last_error() const;

private:
    void worker_loop() noexcept;
    bool fail(const std::string& message);

    std::shared_ptr<serial_package::SerialPort> port_;
    std::ostream* output_ = nullptr;
    runtime::LatestValueMailbox<runtime::GuidanceCommand> mailbox_;
    std::atomic<bool> stop_requested_{false};
    std::atomic<bool> cancel_requested_{false};
    std::atomic<bool> running_{false};
    std::atomic<std::uint64_t> submitted_count_{0};
    std::atomic<std::uint64_t> send_failure_count_{0};
    std::atomic<std::uint64_t> active_generation_{0};
    std::thread worker_;
    mutable std::mutex mutex_;
    std::mutex send_mutex_;
    std::string last_error_;
};

}  // mosas::app 命名空间结束

#endif  // MOSAS_ADAPTERS_HOST_ADAPTERS_HPP
