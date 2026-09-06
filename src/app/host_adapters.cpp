#include <mosas/app/host_adapters.hpp>
#include <serial_package/serial_protocol.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <utility>

namespace mosas::app {
namespace {

runtime::TimestampNs steady_timestamp_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

}  // 匿名命名空间结束

SimulatedImuSource::SimulatedImuSource(double sample_rate_hz)
    : period_(std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(1.0 / sample_rate_hz))) {
    if (sample_rate_hz <= 0.0 || !std::isfinite(sample_rate_hz) ||
        period_.count() <= 0) {
        period_ = std::chrono::nanoseconds(1);
    }
}

runtime::SourceResult SimulatedImuSource::read(runtime::ImuSample* sample) {
    if (sample == nullptr) {
        return {runtime::SourceStatus::fatal, "imu sample output is null"};
    }
    std::unique_lock<std::mutex> lock(mutex_);
    if (cancelled_) {
        return {runtime::SourceStatus::cancelled, {}};
    }
    const auto now = std::chrono::steady_clock::now();
    if (!started_) {
        next_deadline_ = now;
        started_ = true;
    }
    if (condition_.wait_until(lock, next_deadline_,
                              [this] { return cancelled_; })) {
        return {runtime::SourceStatus::cancelled, {}};
    }
    const auto timestamp = steady_timestamp_ns();
    next_deadline_ = std::chrono::steady_clock::now() + period_;
    sample->timestamp_ns = timestamp;
    sample->acceleration = {0.0, 0.0, 0.0};
    sample->angular_velocity = {0.0, 0.0, 0.0};
    return {runtime::SourceStatus::ok, {}};
}

void SimulatedImuSource::cancel() noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    cancelled_ = true;
    condition_.notify_all();
}

LoggingCommandSink::LoggingCommandSink() : output_(&std::cout) {}

LoggingCommandSink::LoggingCommandSink(std::ostream& output) : output_(&output) {}

bool LoggingCommandSink::send(const runtime::GuidanceCommand& command) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (output_ == nullptr) {
        return false;
    }
    *output_ << "[日志] timestamp_ns=" << command.timestamp_ns
             << " valid=" << (command.output.valid ? "true" : "false")
             << " command_overload=" << command.output.command_overload << '\n';
    return static_cast<bool>(*output_);
}

SerialCommandSink::SerialCommandSink(
    std::shared_ptr<serial_package::SerialPort> port)
    : port_(std::move(port)), output_(&std::cerr) {}

SerialCommandSink::SerialCommandSink(
    std::shared_ptr<serial_package::SerialPort> port, std::ostream& output)
    : port_(std::move(port)), output_(&output) {}

bool SerialCommandSink::send(const runtime::GuidanceCommand& command) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (port_ == nullptr) {
        return fail("串口对象为空");
    }
    if (!port_->is_open()) {
        return fail("串口未打开");
    }

    std::array<std::uint8_t, serial_package::kControlFrameSize> frame{};
    std::string error;
    const serial_package::ControlFrameValues values{
        0.0,
        command.output.body_overload.z,
        command.output.body_overload.y,
    };
    if (!serial_package::encode_control_frame(values, &frame, &error)) {
        return fail(error.empty() ? "控制帧编码失败" : error);
    }
    if (!port_->write_all(frame.data(), frame.size())) {
        const std::string port_error = port_->last_error();
        return fail(port_error.empty() ? "控制帧发送失败" : port_error);
    }

    last_error_.clear();
    return true;
}

std::string SerialCommandSink::last_error() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return last_error_;
}

bool SerialCommandSink::fail(const std::string& message) {
    last_error_ = message;
    if (output_ != nullptr) {
        *output_ << "[串口发送失败] " << message << '\n';
    }
    return false;
}

}  // mosas::app 命名空间结束
