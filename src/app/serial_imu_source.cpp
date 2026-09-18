#include <mosas/app/serial_imu_source.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <iostream>
#include <ostream>
#include <thread>
#include <utility>

namespace mosas::app {
namespace {

runtime::TimestampNs wire_byte_period_ns(
    const serial_package::SerialPortConfig& config) noexcept {
    if (config.baud_rate == 0) {
        return 1;
    }
    const std::int64_t parity_bits = config.parity == serial_package::Parity::none
                                         ? 0
                                         : 1;
    const std::int64_t stop_bits =
        config.stop_bits == serial_package::StopBits::two ? 2 : 1;
    const std::int64_t bits_per_byte =
        1 + static_cast<std::int64_t>(config.data_bits) + parity_bits +
        stop_bits;
    const std::int64_t numerator = bits_per_byte * 1'000'000'000LL;
    const std::int64_t baud = static_cast<std::int64_t>(config.baud_rate);
    return std::max<std::int64_t>(1, (numerator + baud - 1) / baud);
}

runtime::TimestampNs source_wire_byte_period(
    const std::shared_ptr<serial_package::SerialPort>& port) noexcept {
    return port == nullptr ? 1 : wire_byte_period_ns(port->config());
}

serial_package::ReceiveThreadConfig default_receive_config() noexcept {
    serial_package::ReceiveThreadConfig config;
    config.read_chunk_size = runtime::RawSerialChunk::kMaxChunkBytes;
    config.realtime_priority = 20;
    config.require_realtime_priority = false;
    config.cpu_affinity = -1;
    return config;
}

}  // namespace

SerialImuSource::SerialImuSource(
    std::shared_ptr<serial_package::SerialPort> port, double sample_rate_hz,
    int read_timeout_ms)
    : SerialImuSource(std::move(port), sample_rate_hz, read_timeout_ms, false,
                      std::cerr) {}

SerialImuSource::SerialImuSource(
    std::shared_ptr<serial_package::SerialPort> port, double sample_rate_hz,
    int read_timeout_ms, std::ostream& output)
    : SerialImuSource(std::move(port), sample_rate_hz, read_timeout_ms, false,
                      output, 0) {}

SerialImuSource::SerialImuSource(
    std::shared_ptr<serial_package::SerialPort> port, double sample_rate_hz,
    int read_timeout_ms, bool enable_interval_statistics)
    : SerialImuSource(std::move(port), sample_rate_hz, read_timeout_ms,
                      enable_interval_statistics, std::cerr, 0) {}

SerialImuSource::SerialImuSource(
    std::shared_ptr<serial_package::SerialPort> port, double sample_rate_hz,
    int read_timeout_ms, bool enable_interval_statistics,
    std::ostream& output)
    : SerialImuSource(std::move(port), sample_rate_hz, read_timeout_ms,
                      enable_interval_statistics, output, 0,
                      default_receive_config()) {}

SerialImuSource::SerialImuSource(
    std::shared_ptr<serial_package::SerialPort> port, double sample_rate_hz,
    int read_timeout_ms, bool enable_interval_statistics,
    std::ostream& output, runtime::TimestampNs measurement_latency_ns)
    : SerialImuSource(std::move(port), sample_rate_hz, read_timeout_ms,
                      enable_interval_statistics, output,
                      measurement_latency_ns, default_receive_config()) {}

SerialImuSource::SerialImuSource(
    std::shared_ptr<serial_package::SerialPort> port, double sample_rate_hz,
    int read_timeout_ms, bool enable_interval_statistics,
    std::ostream& output, runtime::TimestampNs measurement_latency_ns,
    const serial_package::ReceiveThreadConfig& receive_config)
    : port_(std::move(port)),
      output_(&output),
      read_timeout_ms_(read_timeout_ms),
      measurement_latency_ns_(measurement_latency_ns),
      interval_statistics_enabled_(enable_interval_statistics),
      raw_receive_buffer_(kRawQueueCapacity),
      decoder_(source_wire_byte_period(port_), measurement_latency_ns,
               kDecodedQueueCapacity),
      receive_config_(receive_config) {
    if (port_ == nullptr) {
        configuration_error_ = "串口对象为空";
        return;
    }
    if (!std::isfinite(sample_rate_hz) || sample_rate_hz <= 0.0) {
        configuration_error_ = "IMU 采样频率必须是大于 0 的有限数值";
        return;
    }
    if (read_timeout_ms_ <= 0) {
        configuration_error_ = "串口 IMU read_timeout_ms 必须大于 0";
        return;
    }
    if (measurement_latency_ns_ < 0) {
        configuration_error_ = "IMU 测量延迟不能为负数";
        return;
    }
    if (receive_config_.read_chunk_size == 0 ||
        receive_config_.realtime_priority < 0 ||
        receive_config_.realtime_priority > 99 ||
        receive_config_.cpu_affinity < -1) {
        configuration_error_ = "IMU 接收线程调度配置无效";
        return;
    }
}

runtime::SourceResult SerialImuSource::start() {
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    if (started_.load(std::memory_order_acquire)) {
        return {runtime::SourceStatus::ok, {}};
    }
    if (!configuration_error_.empty()) {
        return fatal_result(configuration_error_.c_str());
    }
    if (!port_->is_open()) {
        return fatal_result("串口未打开");
    }

    cancelled_.store(false, std::memory_order_release);
    raw_receive_buffer_.reset();
    decoder_.reset();
    interval_statistics_ = {};
    has_interval_timestamp_ = false;
    last_interval_timestamp_ns_ = -1;
    last_sample_timestamp_ns_.store(-1, std::memory_order_relaxed);
    delivered_sample_count_.store(0, std::memory_order_relaxed);
    timeout_count_.store(0, std::memory_order_relaxed);

    if (!port_->start_receive(RawImuReceiveBuffer::receive_callback,
                              &raw_receive_buffer_, receive_config_)) {
        return fatal_result("启动串口异步接收失败");
    }
    started_.store(true, std::memory_order_release);
    return {runtime::SourceStatus::ok, {}};
}

bool SerialImuSource::try_pop(
    runtime::ImuAngularVelocitySample* sample) noexcept {
    if (sample == nullptr || !started_.load(std::memory_order_acquire) ||
        cancelled_.load(std::memory_order_acquire)) {
        return false;
    }

    if (decoder_.try_pop(sample)) {
        return deliver_sample(sample);
    }

    runtime::RawSerialChunk chunk{};
    while (raw_receive_buffer_.try_pop(&chunk)) {
        decoder_.consume(chunk);
        if (decoder_.try_pop(sample)) {
            return deliver_sample(sample);
        }
    }
    return false;
}

runtime::SourceResult SerialImuSource::read(runtime::ImuSample* sample) {
    if (sample == nullptr) {
        return fatal_result("IMU sample 输出指针为空");
    }
    const runtime::SourceResult start_result = start();
    if (start_result.status != runtime::SourceStatus::ok) {
        return start_result;
    }
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(read_timeout_ms_);
    runtime::ImuAngularVelocitySample angular_velocity{};
    while (!cancelled_.load(std::memory_order_acquire) &&
           std::chrono::steady_clock::now() < deadline) {
        if (try_pop(&angular_velocity)) {
            sample->timestamp_ns = angular_velocity.timestamp_ns;
            sample->acceleration = angular_velocity.acceleration;
            sample->angular_velocity = angular_velocity.angular_velocity;
            return {runtime::SourceStatus::ok, {}};
        }
        std::this_thread::yield();
    }
    if (cancelled_.load(std::memory_order_acquire)) {
        return {runtime::SourceStatus::cancelled, {}};
    }
    return timeout_result();
}

void SerialImuSource::cancel() noexcept {
    cancelled_.store(true, std::memory_order_release);
    if (port_ != nullptr) {
        port_->stop_receive();
    }
    started_.store(false, std::memory_order_release);
}

void SerialImuSource::stop() noexcept {
    cancel();
}

std::uint16_t SerialImuSource::initialization_g_raw() const noexcept {
    return 0;
}

SerialImuIntervalStatistics
SerialImuSource::interval_statistics() const noexcept {
    return interval_statistics_;
}

runtime::ImuSourceDiagnostics SerialImuSource::diagnostics() const noexcept {
    runtime::ImuSourceDiagnostics diagnostics;
    diagnostics.available = true;
    diagnostics.latest_sample_timestamp_ns =
        last_sample_timestamp_ns_.load(std::memory_order_relaxed);
    diagnostics.buffered_bytes = raw_receive_buffer_.queued_chunk_count();
    diagnostics.delivered_sample_count =
        delivered_sample_count_.load(std::memory_order_relaxed);
    diagnostics.timeout_count = timeout_count_.load(std::memory_order_relaxed);
    diagnostics.parse_error_count = decoder_.parse_error_count();
    diagnostics.received_chunk_count =
        raw_receive_buffer_.received_chunk_count();
    diagnostics.dropped_chunk_count = raw_receive_buffer_.dropped_chunk_count();
    diagnostics.sequence_gap_count = decoder_.sequence_gap_count();
    diagnostics.timestamp_rollback_count = decoder_.timestamp_rollback_count();
    diagnostics.dropped_sample_count = decoder_.dropped_sample_count();
    diagnostics.queued_chunk_count = raw_receive_buffer_.queued_chunk_count();
    diagnostics.chunk_queue_capacity = raw_receive_buffer_.queue_capacity();
    diagnostics.receive_realtime_priority_applied =
        port_ != nullptr && port_->receive_realtime_priority_applied();
    diagnostics.receive_priority_error =
        port_ != nullptr && port_->receive_priority_error();
    return diagnostics;
}

bool SerialImuSource::deliver_sample(
    runtime::ImuAngularVelocitySample* sample) noexcept {
    if (sample == nullptr || sample->timestamp_ns < 0) {
        return false;
    }
    if (interval_statistics_enabled_ && has_interval_timestamp_) {
        const runtime::TimestampNs interval =
            sample->timestamp_ns - last_interval_timestamp_ns_;
        if (interval > 0) {
            if (interval_statistics_.interval_count == 0) {
                interval_statistics_.minimum_interval_ns = interval;
                interval_statistics_.maximum_interval_ns = interval;
            } else {
                interval_statistics_.minimum_interval_ns = std::min(
                    interval_statistics_.minimum_interval_ns, interval);
                interval_statistics_.maximum_interval_ns = std::max(
                    interval_statistics_.maximum_interval_ns, interval);
            }
            ++interval_statistics_.interval_count;
            if (static_cast<std::uint64_t>(interval) <=
                std::numeric_limits<std::uint64_t>::max() -
                    interval_statistics_.total_interval_ns) {
                interval_statistics_.total_interval_ns +=
                    static_cast<std::uint64_t>(interval);
            }
        }
    }
    has_interval_timestamp_ = true;
    last_interval_timestamp_ns_ = sample->timestamp_ns;
    last_sample_timestamp_ns_.store(sample->timestamp_ns,
                                    std::memory_order_relaxed);
    delivered_sample_count_.fetch_add(1, std::memory_order_relaxed);
    return true;
}

runtime::SourceResult SerialImuSource::fatal_result(const char* message) {
    (void)output_;
    return {runtime::SourceStatus::fatal,
            message == nullptr ? "串口 IMU 数据源错误" : message};
}

runtime::SourceResult SerialImuSource::timeout_result() noexcept {
    timeout_count_.fetch_add(1, std::memory_order_relaxed);
    return {runtime::SourceStatus::timeout, "等待 IMU 运动样本超时"};
}

}  // namespace mosas::app
