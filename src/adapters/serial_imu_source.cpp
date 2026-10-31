#include <mosas/adapters/serial_imu_source.hpp>

#include <algorithm>
#include <limits>
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

}  // namespace

SerialImuSource::SerialImuSource(
    std::shared_ptr<serial_package::SerialPort> port,
    bool enable_interval_statistics,
    runtime::TimestampNs measurement_latency_ns,
    const serial_package::ReceiveThreadConfig& receive_config, domain::SequenceTimeConfig timing)
    : port_(std::move(port)),
      measurement_latency_ns_(measurement_latency_ns),
      interval_statistics_enabled_(enable_interval_statistics),
      raw_receive_buffer_(kRawQueueCapacity),
      decoder_(source_wire_byte_period(port_), measurement_latency_ns,
               kDecodedQueueCapacity, timing),
      receive_config_(receive_config) {
    if (port_ == nullptr) {
        configuration_error_ = "串口对象为空";
        return;
    }
    if (!domain::SequenceTime::valid_config(timing)) {
        configuration_error_ = "IMU 固定序号时间配置无效";
        return;
    }
    if (measurement_latency_ns_ < 0) {
        configuration_error_ = "IMU 测量延迟不能为负数";
        return;
    }
    if (receive_config_.read_chunk_size == 0 ||
        receive_config_.realtime_priority < 0 ||
        receive_config_.realtime_priority > 99) {
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
    if (!port_->flush_input()) return fatal_result("清空串口启动积压失败");
    decoder_.reset(runtime::monotonic_timestamp_ns());
    interval_statistics_ = {};
    has_interval_timestamp_ = false;
    last_interval_timestamp_ns_ = -1;
    last_sample_timestamp_ns_.store(-1, std::memory_order_relaxed);
    delivered_sample_count_.store(0, std::memory_order_relaxed);

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
        decoder_.consume(chunk, runtime::monotonic_timestamp_ns());
        if (decoder_.try_pop(sample)) {
            return deliver_sample(sample);
        }
    }
    return false;
}

bool SerialImuSource::wait_for_data() noexcept {
    if (!started_.load(std::memory_order_acquire) ||
        cancelled_.load(std::memory_order_acquire)) {
        return false;
    }
    return raw_receive_buffer_.wait_for_data();
}

void SerialImuSource::cancel() noexcept {
    cancelled_.store(true, std::memory_order_release);
    raw_receive_buffer_.close();
    if (port_ != nullptr) {
        port_->request_stop_receive();
    }
    started_.store(false, std::memory_order_release);
}

void SerialImuSource::stop() noexcept {
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    cancel();
    if (port_ != nullptr) {
        port_->stop_receive();
    }
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
    diagnostics.parse_error_count = decoder_.parse_error_count();
    diagnostics.received_chunk_count =
        raw_receive_buffer_.received_chunk_count();
    diagnostics.dropped_chunk_count = raw_receive_buffer_.dropped_chunk_count();
    diagnostics.sequence_gap_count = decoder_.sequence_gap_count();
    diagnostics.crc_error_count = decoder_.crc_error_count();
    diagnostics.timing = decoder_.time_diagnostics();
    diagnostics.timing.baseline_compensation_ns = -measurement_latency_ns_;
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
    return {runtime::SourceStatus::fatal,
            message == nullptr ? "串口 IMU 数据源错误" : message};
}

}  // namespace mosas::app
