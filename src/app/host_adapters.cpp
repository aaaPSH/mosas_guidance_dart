#include <mosas/app/host_adapters.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>

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

}  // mosas::app 命名空间结束
