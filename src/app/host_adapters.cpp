#include <mosas/app/host_adapters.hpp>
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

runtime::SourceResult SimulatedImuSource::read(runtime::ImuSample* sample) {
    if (sample == nullptr) {
        return {runtime::SourceStatus::fatal, "imu sample output is null"};
    }
    const runtime::SourceResult start_result = start();
    if (start_result.status != runtime::SourceStatus::ok) {
        return start_result;
    }
    std::unique_lock<std::mutex> lock(mutex_);
    while (!cancelled_) {
        const auto deadline = next_deadline_;
        if (condition_.wait_until(lock, deadline,
                                  [this] { return cancelled_; })) {
            return {runtime::SourceStatus::cancelled, {}};
        }
        lock.unlock();
        runtime::ImuAngularVelocitySample angular_velocity{};
        if (try_pop(&angular_velocity)) {
            sample->timestamp_ns = angular_velocity.timestamp_ns;
            sample->acceleration = angular_velocity.acceleration;
            sample->angular_velocity = angular_velocity.angular_velocity;
            return {runtime::SourceStatus::ok, {}};
        }
        lock.lock();
    }
    return {runtime::SourceStatus::cancelled, {}};
}

void SimulatedImuSource::cancel() noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    cancelled_ = true;
    condition_.notify_all();
}

void SimulatedImuSource::stop() noexcept {
    cancel();
}

LoggingCommandSink::LoggingCommandSink()
    : output_(&std::cout), logger_(std::cout) {}

LoggingCommandSink::LoggingCommandSink(std::ostream& output)
    : output_(&output), logger_(output) {}

bool LoggingCommandSink::start() { return logger_.start(); }

bool LoggingCommandSink::try_submit(
    const runtime::GuidanceCommand& command) noexcept {
    runtime::RuntimeLogEvent event{};
    event.kind = runtime::RuntimeLogEventKind::guidance_command;
    event.timestamp_ns = command.timestamp_ns;
    event.value0 = command.output.valid ? command.output.body_overload.x : 0.0;
    event.value1 = command.output.valid ? command.output.body_overload.y : 0.0;
    event.value2 = command.output.valid ? command.output.body_overload.z : 0.0;
    if (!logger_.try_submit(event)) {
        return false;
    }
    submitted_count_.fetch_add(1, std::memory_order_relaxed);
    return true;
}

bool LoggingCommandSink::send(const runtime::GuidanceCommand& command) {
    if (try_submit(command)) {
        return true;
    }
    if (!start()) {
        return false;
    }
    return try_submit(command);
}

void LoggingCommandSink::stop() noexcept { logger_.stop(); }

runtime::CommandSink::Diagnostics LoggingCommandSink::diagnostics() const noexcept {
    return {true, submitted_count_.load(std::memory_order_relaxed),
            logger_.dropped_count(), 0};
}

SerialCommandSink::SerialCommandSink(
    std::shared_ptr<serial_package::SerialPort> port)
    : port_(std::move(port)), output_(&std::cerr) {}

SerialCommandSink::SerialCommandSink(
    std::shared_ptr<serial_package::SerialPort> port, std::ostream& output)
    : port_(std::move(port)), output_(&output) {}

SerialCommandSink::~SerialCommandSink() { stop(); }

bool SerialCommandSink::start() {
    if (port_ == nullptr) {
        return fail("串口对象为空");
    }
    if (!port_->is_open()) {
        return fail("串口未打开");
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (worker_.joinable()) {
        return true;
    }
    mailbox_.reset();
    stop_requested_.store(false, std::memory_order_release);
    cancel_requested_.store(false, std::memory_order_release);
    running_.store(true, std::memory_order_release);
    try {
        worker_ = std::thread(&SerialCommandSink::worker_loop, this);
    } catch (...) {
        running_.store(false, std::memory_order_release);
        return fail("启动异步串口发送线程失败");
    }
    return true;
}

bool SerialCommandSink::try_submit(
    const runtime::GuidanceCommand& command) noexcept {
    if (!running_.load(std::memory_order_acquire) ||
        cancel_requested_.load(std::memory_order_acquire) ||
        command.launch_generation !=
            active_generation_.load(std::memory_order_acquire)) {
        return false;
    }
    if (!mailbox_.try_publish(command)) {
        return false;
    }
    submitted_count_.fetch_add(1, std::memory_order_relaxed);
    return true;
}

bool SerialCommandSink::send(const runtime::GuidanceCommand& command) {
    if (!running_.load(std::memory_order_acquire) && !start()) {
        return false;
    }
    return try_submit(command);
}

void SerialCommandSink::set_active_generation(
    std::uint64_t generation) noexcept {
    // 与串口写入共用屏障，返回后旧周期的在途写入也已结束。
    std::lock_guard<std::mutex> lock(send_mutex_);
    active_generation_.store(generation, std::memory_order_release);
}

void SerialCommandSink::cancel() noexcept {
    cancel_requested_.store(true, std::memory_order_release);
}

void SerialCommandSink::stop() noexcept {
    cancel_requested_.store(true, std::memory_order_release);
    stop_requested_.store(true, std::memory_order_release);
    mailbox_.close();
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
    runtime::GuidanceCommand command{};
    while (!stop_requested_.load(std::memory_order_acquire)) {
        if (cancel_requested_.load(std::memory_order_acquire)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        if (!mailbox_.try_consume(&command)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        std::lock_guard<std::mutex> send_lock(send_mutex_);
        if (command.launch_generation !=
            active_generation_.load(std::memory_order_acquire)) {
            continue;
        }
        if (port_ == nullptr) {
            send_failure_count_.fetch_add(1, std::memory_order_relaxed);
            fail("串口对象为空");
            continue;
        }
        if (!port_->is_open()) {
            send_failure_count_.fetch_add(1, std::memory_order_relaxed);
            fail("串口未打开");
            continue;
        }

        std::array<std::uint8_t, serial_package::kControlFrameSize> frame{};
        std::string error;
        const serial_package::ControlFrameValues values{
            command.launched,
            command.target_found,
            command.output.valid ? command.output.body_overload.z : 0.0,
            command.output.valid ? command.output.body_overload.y : 0.0};
        if (!serial_package::encode_control_frame(values, &frame, &error) ||
            !port_->write_all(frame.data(), frame.size())) {
            send_failure_count_.fetch_add(1, std::memory_order_relaxed);
            const std::string port_error = port_->last_error();
            fail(error.empty() ? (port_error.empty() ? "控制帧发送失败"
                                                      : port_error)
                               : error);
            continue;
        }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            last_error_.clear();
        }
    }
}

/*
 * 保留旧的编码逻辑说明：实际编码和写串口已移到 worker_loop。
 */
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
