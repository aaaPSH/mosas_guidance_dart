#include <mosas/adapters/host_adapters.hpp>
#include "command_adapter.hpp"
#include <serial_package/serial_protocol.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <utility>

namespace mosas::app {

SimulatedImuSource::SimulatedImuSource(double sample_rate_hz,
                                       double gravity_mps2,
                                       bool accelerometer_specific_force)
    : period_(std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(1.0 / sample_rate_hz))),
      gravity_mps2_(std::isfinite(gravity_mps2) && gravity_mps2 > 0.0
                        ? gravity_mps2
                        : 9.80665),
      accelerometer_specific_force_(accelerometer_specific_force) {
    if (sample_rate_hz <= 0.0 || !std::isfinite(sample_rate_hz) ||
        period_.count() <= 0) {
        period_ = std::chrono::nanoseconds(1);
    }
}

runtime::SourceResult SimulatedImuSource::start() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (cancelled_) {
        return {runtime::SourceStatus::cancelled, {}};
    }
    started_ = true;
    next_deadline_ = std::chrono::steady_clock::now();
    return {runtime::SourceStatus::ok, {}};
}

bool SimulatedImuSource::try_pop(
    runtime::ImuAngularVelocitySample* sample) noexcept {
    if (sample == nullptr) {
        return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (cancelled_ || !started_) {
        return false;
    }
    const auto now = std::chrono::steady_clock::now();
    if (now < next_deadline_) {
        return false;
    }
    sample->timestamp_ns = runtime::monotonic_timestamp_ns();
    if (sample->timestamp_ns < 0) {
        return false;
    }
    sample->acceleration = {
        0.0,
        accelerometer_specific_force_ ? gravity_mps2_ : -gravity_mps2_,
        0.0};
    sample->angular_velocity = {0.0, 0.0, 0.0};
    next_deadline_ = now + period_;
    return true;
}

bool SimulatedImuSource::wait_for_data() noexcept {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!started_ || cancelled_) {
        return false;
    }
    condition_.wait_until(lock, next_deadline_, [this] {
        return cancelled_ || !started_;
    });
    return started_ && !cancelled_;
}

void SimulatedImuSource::cancel() noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    cancelled_ = true;
    condition_.notify_all();
}

void SimulatedImuSource::stop() noexcept {
    cancel();
}

LoggingCommandSink::LoggingCommandSink() : logger_(std::cout) {}

LoggingCommandSink::LoggingCommandSink(std::ostream& output)
    : logger_(output) {}

bool LoggingCommandSink::start() {
    std::lock_guard<std::mutex> lock(emission_mutex_);
    if (!command_output_.configured() || !logger_.start()) return false;
    command_output_.reset();
    return true;
}

void LoggingCommandSink::set_active_generation(std::uint64_t generation) noexcept {
    std::lock_guard<std::mutex> lock(emission_mutex_);
    active_generation_ = generation;
    command_output_.invalidate();
}

bool LoggingCommandSink::try_submit(
    const runtime::GuidanceCommand& command) noexcept {
    std::lock_guard<std::mutex> lock(emission_mutex_);
    if (command.generation != active_generation_) return false;
    auto attempt = command_output_.prepare(command, active_generation_, false);
    runtime::RuntimeLogEvent event{};
    event.kind = runtime::RuntimeLogEventKind::guidance_command;
    event.timestamp_ns = attempt.mapping.trace.emit_time_ns;
    const auto& mapped = attempt.mapping.command;
    event.value0 = mapped.overload_g.x;
    event.value1 = mapped.overload_g.y;
    event.value2 = mapped.overload_g.z;
    const bool accepted = logger_.try_submit(event);
    command_output_.complete(std::move(attempt), accepted ? domain::CommandEmissionDisposition::log_accepted
                                                        : domain::CommandEmissionDisposition::io_rejected);
    if (accepted) submitted_count_.fetch_add(1, std::memory_order_relaxed);
    return accepted;
}

void LoggingCommandSink::stop() noexcept {
    std::lock_guard<std::mutex> lock(emission_mutex_);
    logger_.stop_and_flush();
}

runtime::CommandSink::Diagnostics LoggingCommandSink::diagnostics() const noexcept {
    return {true, submitted_count_.load(std::memory_order_relaxed),
            logger_.diagnostics().dropped_count, 0};
}

SerialCommandSink::SerialCommandSink(
    std::shared_ptr<serial_package::SerialPort> port,
    const CommandAppConfig& config, std::ostream& output)
    : port_(std::move(port)), config_(config), output_(&output) {}

SerialCommandSink::~SerialCommandSink() { stop(); }

bool SerialCommandSink::start() {
    if (!command_output_.configured()) return fail("输出映射依赖未绑定");
    if (port_ == nullptr) {
        return fail("串口对象为空");
    }
    if (!port_->is_open()) {
        return fail("串口未打开");
    }
    if (config_.send_rate_hz <= 0 || config_.send_rate_hz > 1000 ||
        config_.command_timeout_ms <= 0) {
        return fail("串口周期发送参数无效");
    }
    std::unique_lock<std::mutex> lock(mutex_);
    if (worker_.joinable()) {
        return true;
    }
    last_error_.clear();
    mailbox_.reset();
    command_output_.reset();
    stop_requested_.store(false, std::memory_order_release);
    cancel_requested_.store(false, std::memory_order_release);
    running_.store(true, std::memory_order_release);
    try {
        worker_ = std::thread(&SerialCommandSink::worker_loop, this);
    } catch (...) {
        running_.store(false, std::memory_order_release);
        lock.unlock();
        return fail("启动异步串口发送线程失败");
    }
    return true;
}

bool SerialCommandSink::try_submit(
    const runtime::GuidanceCommand& command) noexcept {
    if (!running_.load(std::memory_order_acquire) ||
        cancel_requested_.load(std::memory_order_acquire) ||
        command.generation !=
            active_generation_.load(std::memory_order_acquire)) {
        return false;
    }
    if (!mailbox_.try_publish({command, std::chrono::steady_clock::now()})) {
        return false;
    }
    submitted_count_.fetch_add(1, std::memory_order_relaxed);
    return true;
}

void SerialCommandSink::set_active_generation(
    std::uint64_t generation) noexcept {
    // 与串口写入共用屏障，返回后旧周期的在途写入也已结束。
    std::lock_guard<std::mutex> lock(send_mutex_);
    active_generation_.store(generation, std::memory_order_release);
    command_output_.invalidate();
}

void SerialCommandSink::cancel() noexcept {
    {
        std::lock_guard<std::mutex> lock(wait_mutex_);
        cancel_requested_.store(true, std::memory_order_release);
    }
    wait_condition_.notify_all();
}

void SerialCommandSink::stop() noexcept {
    {
        std::lock_guard<std::mutex> lock(wait_mutex_);
        cancel_requested_.store(true, std::memory_order_release);
        stop_requested_.store(true, std::memory_order_release);
    }
    mailbox_.close();
    wait_condition_.notify_all();
    if (worker_.joinable()) {
        worker_.join();
    }
    running_.store(false, std::memory_order_release);
}

runtime::CommandSink::Diagnostics SerialCommandSink::diagnostics() const noexcept {
    return {true, submitted_count_.load(std::memory_order_relaxed),
            mailbox_.dropped_count(),
            send_failure_count_.load(std::memory_order_relaxed)};
}

void SerialCommandSink::worker_loop() noexcept {
    using Clock = std::chrono::steady_clock;
    const auto period = std::chrono::nanoseconds(1'000'000'000 / config_.send_rate_hz);
    const auto timeout = std::chrono::milliseconds(config_.command_timeout_ms);
    auto next_send_at = Clock::now();
    PendingCommand latest{};
    const auto cancelled = [this] {
        return stop_requested_.load(std::memory_order_acquire) ||
               cancel_requested_.load(std::memory_order_acquire);
    };

    while (true) {
        {
            std::unique_lock<std::mutex> lock(wait_mutex_);
            if (wait_condition_.wait_until(lock, next_send_at, cancelled)) {
                return;
            }
        }
        {
            std::lock_guard<std::mutex> send_lock(send_mutex_);
            if (cancelled()) {
                return;
            }
            const auto generation = active_generation_.load(std::memory_order_acquire);
            if (latest.command.generation != generation) {
                latest = {};
                latest.command.generation = generation;
            }
            PendingCommand pending{};
            if (mailbox_.try_consume(&pending) &&
                pending.command.generation == generation) {
                latest = pending;
            }

            const bool expired = latest.submitted_at == Clock::time_point{} ||
                                 Clock::now() - latest.submitted_at >= timeout;
            auto attempt = command_output_.prepare(latest.command, generation, expired);
            if (cancelled()) return;
            const bool written = send_command(attempt.mapping.command);
            command_output_.complete(std::move(attempt), written ? domain::CommandEmissionDisposition::serial_written
                                                               : domain::CommandEmissionDisposition::io_rejected);
        }

        next_send_at += period;
        const auto now = Clock::now();
        if (next_send_at <= now) {
            // 维持周期相位，跳过已错过的时隙，不在阻塞恢复后突发补发。
            next_send_at += period * ((now - next_send_at) / period + 1);
        }
    }
}

bool SerialCommandSink::send_command(
    const domain::MappedGuidanceCommand& command) noexcept {
    std::array<std::uint8_t, serial_package::kControlFrameSize> frame{};
    std::string error;
    const auto values = detail::make_control_frame_values(command);
    if (!serial_package::encode_control_frame(values, &frame, &error) ||
        !port_->write_all(frame.data(), frame.size())) {
        send_failure_count_.fetch_add(1, std::memory_order_relaxed);
        const std::string port_error = port_->last_error();
        fail(error.empty() ? (port_error.empty() ? "控制帧发送失败" : port_error)
                           : error);
        return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    last_error_.clear();
    return true;
}

bool SerialCommandSink::fail(const std::string& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    last_error_ = message;
    if (output_ != nullptr) {
        *output_ << "[串口发送失败] " << message << '\n';
    }
    return false;
}

std::string SerialCommandSink::last_error() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return last_error_;
}

}  // mosas::app 命名空间结束
