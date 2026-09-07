#include <mosas/app/serial_imu_source.hpp>

#include <serial_package/serial_protocol.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <utility>

namespace mosas::app {
namespace {

runtime::TimestampNs steady_timestamp_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

std::string hex_dump(const std::uint8_t* data, std::size_t size) {
    std::ostringstream stream;
    stream << std::hex << std::uppercase << std::setfill('0');
    const std::size_t dump_size = std::min(size, std::size_t{32});
    for (std::size_t index = 0; index < dump_size; ++index) {
        if (index != 0) {
            stream << ' ';
        }
        stream << std::setw(2) << static_cast<unsigned int>(data[index]);
    }
    if (dump_size < size) {
        stream << " ...";
    }
    return stream.str();
}

std::string port_error_or(const serial_package::SerialPort& port,
                          const char* fallback) {
    const std::string error = port.last_error();
    return error.empty() ? std::string(fallback) : error;
}

runtime::TimestampNs saturating_add(runtime::TimestampNs lhs,
                                    runtime::TimestampNs rhs) noexcept {
    const auto maximum = std::numeric_limits<runtime::TimestampNs>::max();
    const auto minimum = std::numeric_limits<runtime::TimestampNs>::min();
    if (rhs > 0 && lhs > maximum - rhs) {
        return maximum;
    }
    if (rhs < 0 && lhs < minimum - rhs) {
        return minimum;
    }
    return lhs + rhs;
}

runtime::TimestampNs saturating_subtract(runtime::TimestampNs lhs,
                                         runtime::TimestampNs rhs) noexcept {
    const auto maximum = std::numeric_limits<runtime::TimestampNs>::max();
    const auto minimum = std::numeric_limits<runtime::TimestampNs>::min();
    if (rhs > 0 && lhs < minimum + rhs) {
        return minimum;
    }
    if (rhs < 0 && lhs > maximum + rhs) {
        return maximum;
    }
    return lhs - rhs;
}

runtime::TimestampNs saturating_multiply(runtime::TimestampNs value,
                                         std::size_t multiplier) noexcept {
    if (value <= 0 || multiplier == 0) {
        return 0;
    }
    const auto maximum = std::numeric_limits<runtime::TimestampNs>::max();
    if (multiplier > static_cast<std::size_t>(maximum / value)) {
        return maximum;
    }
    return value * static_cast<runtime::TimestampNs>(multiplier);
}

}  // 匿名命名空间结束

SerialImuSource::SerialImuSource(
    std::shared_ptr<serial_package::SerialPort> port, double sample_rate_hz,
    int read_timeout_ms)
    : SerialImuSource(std::move(port), sample_rate_hz, read_timeout_ms, false,
                      std::cerr) {}

SerialImuSource::SerialImuSource(
    std::shared_ptr<serial_package::SerialPort> port, double sample_rate_hz,
    int read_timeout_ms, std::ostream& output)
    : SerialImuSource(std::move(port), sample_rate_hz, read_timeout_ms, false,
                      output) {}

SerialImuSource::SerialImuSource(
    std::shared_ptr<serial_package::SerialPort> port, double sample_rate_hz,
    int read_timeout_ms, bool enable_interval_statistics)
    : SerialImuSource(std::move(port), sample_rate_hz, read_timeout_ms,
                      enable_interval_statistics, std::cerr) {}

SerialImuSource::SerialImuSource(
    std::shared_ptr<serial_package::SerialPort> port, double sample_rate_hz,
    int read_timeout_ms, bool enable_interval_statistics,
    std::ostream& output)
    : port_(std::move(port)),
      output_(&output),
      interval_statistics_enabled_(enable_interval_statistics) {
    if (port_ == nullptr) {
        configuration_error_ = "串口对象为空";
        return;
    }
    if (!std::isfinite(sample_rate_hz) || sample_rate_hz <= 0.0) {
        configuration_error_ = "IMU 采样频率必须是大于 0 的有限数值";
        return;
    }
    if (read_timeout_ms <= 0) {
        configuration_error_ = "串口 IMU read_timeout_ms 必须大于 0";
        return;
    }

    const double period_ns_value = 1e9 / sample_rate_hz;
    if (!std::isfinite(period_ns_value) ||
        period_ns_value > static_cast<double>(
                              std::numeric_limits<std::int64_t>::max())) {
        configuration_error_ = "IMU 采样周期超出时间戳范围";
        return;
    }
    const auto period_count = static_cast<std::int64_t>(
        std::llround(period_ns_value));
    if (period_count <= 0) {
        configuration_error_ = "IMU 采样频率过高，无法形成正采样周期";
        return;
    }
    period_ = std::chrono::nanoseconds(period_count);

    // watchdog 是等待一帧完整数据的窗口，不能用采样周期截断。
    // 采样周期仅用于积压帧的逻辑时间戳重建。
    watchdog_timeout_ms_ = read_timeout_ms;
}

runtime::SourceResult SerialImuSource::read(runtime::ImuSample* sample) {
    if (sample == nullptr) {
        return fatal_result("IMU sample 输出指针为空");
    }
    if (cancelled_.load()) {
        return {runtime::SourceStatus::cancelled, {}};
    }
    if (!configuration_error_.empty()) {
        return fatal_result(configuration_error_);
    }
    if (!port_->is_open()) {
        return fatal_result("串口未打开");
    }

    if (!startup_reported_) {
        const ssize_t startup_bytes = port_->input_bytes_available();
        if (startup_bytes < 0) {
            invalidate_pending_bytes();
            return fatal_result("查询启动时串口输入队列失败: " +
                                port_error_or(*port_, "未知串口错误"));
        }
        remember_pending_bytes(static_cast<std::size_t>(startup_bytes));
        startup_reported_ = true;
        std::ostringstream message;
        message << "启动输入队列有 " << startup_bytes
                << " 字节（完整帧 "
                << static_cast<std::size_t>(startup_bytes) /
                       serial_package::kImuFrameSize
                << "，余 "
                << static_cast<std::size_t>(startup_bytes) %
                       serial_package::kImuFrameSize
                << "）；若发现启动积压，将清理旧数据并等待新 IMU 帧";
        log(message.str());
    }

    const auto deadline = Clock::now() +
                          std::chrono::milliseconds(watchdog_timeout_ms_);
    bool had_parse_error = false;
    while (!cancelled_.load()) {
        align_buffer();

        if (buffered_size_ == serial_package::kImuFrameSize) {
            serial_package::ImuFrameValues values;
            std::string decode_error;
            if (!serial_package::decode_imu_frame(
                    raw_buffer_.data(), serial_package::kImuFrameSize, &values,
                    &decode_error)) {
                const auto parse_error_count =
                    parse_error_count_.fetch_add(1) + 1;
                had_parse_error = true;
                if (parse_error_count == 1 ||
                    parse_error_count % 100 == 0) {
                    std::ostringstream message;
                    message << "候选帧解析失败(累计 " << parse_error_count
                            << " 次): " << decode_error << ", data="
                            << hex_dump(raw_buffer_.data(),
                                        serial_package::kImuFrameSize);
                    log(message.str());
                }
                discard_prefix(1, "非法候选帧重新同步");
                continue;
            }

            // 当前读取调用只交付这一帧，后续完整帧留给下一次读取。
            buffered_size_ = 0;
            buffered_bytes_.store(0);
            const ssize_t pending_bytes = port_->input_bytes_available();
            if (pending_bytes < 0) {
                invalidate_pending_bytes();
                return fatal_result("查询串口输入积压失败: " +
                                    port_error_or(*port_, "未知串口错误"));
            }
            const auto pending_bytes_value =
                static_cast<std::size_t>(pending_bytes);
            remember_pending_bytes(pending_bytes_value);
            const auto pending_frame_count =
                pending_bytes_value / serial_package::kImuFrameSize;
            if (!has_timestamp_ &&
                pending_frame_count != 0) {
                remember_initialization_g(values.initialization_g_raw);
                std::ostringstream message;
                message << "检测到启动阶段串口输入积压 " << pending_bytes
                        << " 字节（完整帧 " << pending_frame_count
                        << "，余 "
                        << pending_bytes_value % serial_package::kImuFrameSize
                        << "），清理旧数据并等待新的 IMU 帧";
                log(message.str());
                if (!port_->flush_input()) {
                    return fatal_result("清理启动阶段串口输入队列失败: " +
                                        port_error_or(*port_, "未知串口错误"));
                }
                // 当前候选帧已经被消费，但它属于启动旧数据。
                buffered_size_ = 0;
                buffered_bytes_.store(0);
                remember_pending_bytes(0);
                continue;
            }

            report_runtime_backlog(pending_bytes_value);
            const runtime::TimestampNs timestamp_ns = timestamp_for_frame(
                steady_timestamp_ns(), pending_frame_count);
            runtime::TimestampNs interval_ns = 0;
            if (has_timestamp_) {
                interval_ns =
                    saturating_subtract(timestamp_ns, last_timestamp_ns_);
                if (interval_ns <= 0) {
                    std::ostringstream message;
                    message << "IMU 时间戳不递增: previous="
                            << last_timestamp_ns_ << ", current="
                            << timestamp_ns;
                    return fatal_result(message.str());
                }
            }

            if (interval_statistics_enabled_ && has_timestamp_) {
                if (interval_statistics_.interval_count == 0) {
                    interval_statistics_.minimum_interval_ns = interval_ns;
                    interval_statistics_.maximum_interval_ns = interval_ns;
                } else {
                    interval_statistics_.minimum_interval_ns = std::min(
                        interval_statistics_.minimum_interval_ns, interval_ns);
                    interval_statistics_.maximum_interval_ns = std::max(
                        interval_statistics_.maximum_interval_ns, interval_ns);
                }
                ++interval_statistics_.interval_count;
                interval_statistics_.total_interval_ns +=
                    static_cast<std::uint64_t>(interval_ns);
            }

            sample->timestamp_ns = timestamp_ns;
            sample->acceleration = {values.acceleration_x_mps2,
                                    values.acceleration_y_mps2,
                                    values.acceleration_z_mps2};
            sample->angular_velocity = {values.angular_velocity_x_rad_s,
                                        values.angular_velocity_y_rad_s,
                                        values.angular_velocity_z_rad_s};
            remember_initialization_g(values.initialization_g_raw);
            last_timestamp_ns_ = timestamp_ns;
            has_timestamp_ = true;
            last_sample_timestamp_ns_.store(timestamp_ns);
            delivered_sample_count_.fetch_add(1);
            if (had_parse_error) {
                log("已从非法 IMU 帧中恢复并交付合法帧");
            }
            return {runtime::SourceStatus::ok, {}};
        }

        const int timeout_ms = remaining_timeout_ms(deadline);
        if (timeout_ms < 0) {
            break;
        }
        const std::size_t bytes_needed =
            serial_package::kImuFrameSize - buffered_size_;
        if (bytes_needed == 0 ||
            buffered_size_ + bytes_needed > raw_buffer_.size()) {
            return fatal_result("串口 IMU 接收缓存越界");
        }
        const ssize_t count = port_->read(raw_buffer_.data() + buffered_size_,
                                          bytes_needed, timeout_ms);
        if (count < 0) {
            return fatal_result("读取串口 IMU 数据失败: " +
                                port_error_or(*port_, "未知串口错误"));
        }
        if (count == 0) {
            break;
        }
        buffered_size_ += static_cast<std::size_t>(count);
        buffered_bytes_.store(buffered_size_);
    }

    if (cancelled_.load()) {
        return {runtime::SourceStatus::cancelled, {}};
    }
    return timeout_result();
}

void SerialImuSource::cancel() noexcept {
    cancelled_.store(true);
}

std::uint16_t SerialImuSource::initialization_g_raw() const noexcept {
    return initialization_g_raw_.load();
}

SerialImuIntervalStatistics
SerialImuSource::interval_statistics() const noexcept {
    return interval_statistics_;
}

runtime::ImuSourceDiagnostics SerialImuSource::diagnostics() const noexcept {
    runtime::ImuSourceDiagnostics diagnostics;
    diagnostics.available = true;
    diagnostics.latest_sample_timestamp_ns = last_sample_timestamp_ns_.load();
    diagnostics.pending_bytes_valid = pending_bytes_valid_.load();
    diagnostics.pending_bytes = pending_bytes_.load();
    diagnostics.pending_frame_count =
        diagnostics.pending_bytes / serial_package::kImuFrameSize;
    diagnostics.buffered_bytes = buffered_bytes_.load();
    diagnostics.delivered_sample_count = delivered_sample_count_.load();
    diagnostics.timeout_count = timeout_count_.load();
    diagnostics.parse_error_count = parse_error_count_.load();
    diagnostics.backlog_event_count = backlog_event_count_.load();
    return diagnostics;
}

void SerialImuSource::remember_initialization_g(std::uint16_t raw_value) {
    if (raw_value == 0) {
        return;
    }
    std::uint16_t expected_initialization_g = 0;
    if (initialization_g_raw_.compare_exchange_strong(expected_initialization_g,
                                                       raw_value)) {
        std::ostringstream message;
        message << "捕获下位机初始化 g raw=0x" << std::hex << std::uppercase
                << std::setfill('0') << std::setw(4)
                << static_cast<unsigned int>(raw_value);
        log(message.str());
    }
}

void SerialImuSource::remember_pending_bytes(
    std::size_t pending_bytes) noexcept {
    pending_bytes_.store(pending_bytes);
    pending_bytes_valid_.store(true);
}

void SerialImuSource::invalidate_pending_bytes() noexcept {
    pending_bytes_valid_.store(false);
}

void SerialImuSource::report_runtime_backlog(std::size_t pending_bytes) {
    const auto pending_frame_count =
        pending_bytes / serial_package::kImuFrameSize;
    const auto now = Clock::now();
    if (pending_frame_count == 0) {
        if (backlog_active_ && now >= next_backlog_log_) {
            log("运行阶段串口输入积压已清空");
            next_backlog_log_ = now + std::chrono::seconds(1);
        }
        if (backlog_active_) {
            backlog_active_ = false;
        }
        return;
    }

    const auto event_count = backlog_event_count_.fetch_add(1) + 1;
    if (now >= next_backlog_log_) {
        std::ostringstream message;
        message << "警告: 检测到运行阶段串口输入积压: pending_bytes="
                << pending_bytes << ", pending_frames="
                << pending_frame_count << ", remainder_bytes="
                << pending_bytes % serial_package::kImuFrameSize
                << ", expected_period_ns=" << period_.count()
                << "，可能是下位机发送频率超过接收线程处理能力（包括发送超频）、"
                   "串口波特率不足或线程调度延迟"
                << "（累计观测 " << event_count << " 次）";
        log(message.str());
        next_backlog_log_ = now + std::chrono::seconds(1);
    }
    backlog_active_ = true;
}

runtime::TimestampNs SerialImuSource::timestamp_for_frame(
    runtime::TimestampNs received_timestamp_ns,
    std::size_t pending_frame_count) const {
    runtime::TimestampNs timestamp = received_timestamp_ns;
    if (has_timestamp_ && pending_frame_count != 0) {
        const runtime::TimestampNs backlog_ns = saturating_multiply(
            period_.count(), pending_frame_count);
        timestamp = saturating_subtract(received_timestamp_ns, backlog_ns);
    }
    if (has_timestamp_) {
        timestamp = std::max(
            timestamp, saturating_add(last_timestamp_ns_, period_.count()));
    }
    return timestamp;
}

void SerialImuSource::align_buffer() {
    if (buffered_size_ == 0) {
        return;
    }
    if (buffered_size_ == 1) {
        if (raw_buffer_[0] != 0x55) {
            discard_prefix(1, "丢弃前置噪声");
        }
        return;
    }

    std::size_t header_offset = buffered_size_;
    for (std::size_t index = 0; index + 1 < buffered_size_; ++index) {
        if (raw_buffer_[index] == 0x55 && raw_buffer_[index + 1] == 0xAA) {
            header_offset = index;
            break;
        }
    }
    if (header_offset == 0) {
        return;
    }
    if (header_offset < buffered_size_) {
        discard_prefix(header_offset, "丢弃帧头前置噪声");
        return;
    }

    const std::size_t keep = raw_buffer_[buffered_size_ - 1] == 0x55 ? 1 : 0;
    discard_prefix(buffered_size_ - keep, "丢弃无法组成帧头的噪声");
}

void SerialImuSource::discard_prefix(std::size_t count,
                                      const std::string& reason) {
    if (count == 0 || count > buffered_size_) {
        return;
    }
    discarded_noise_count_ += count;
    if (discarded_noise_count_ == count || discarded_noise_count_ % 100 == 0) {
        std::ostringstream message;
        message << reason << ": 丢弃 " << count << " 字节，累计 "
                << discarded_noise_count_ << " 字节";
        log(message.str());
    }
    const std::size_t remaining = buffered_size_ - count;
    if (remaining != 0) {
        std::memmove(raw_buffer_.data(), raw_buffer_.data() + count, remaining);
    }
    buffered_size_ = remaining;
    buffered_bytes_.store(remaining);
}

void SerialImuSource::log(const std::string& message) {
    std::lock_guard<std::mutex> lock(log_mutex_);
    if (output_ != nullptr) {
        *output_ << "[串口接收] " << message << '\n';
    }
}

runtime::SourceResult SerialImuSource::fatal_result(
    const std::string& message) {
    log("错误: " + message);
    return {runtime::SourceStatus::fatal, message};
}

int SerialImuSource::remaining_timeout_ms(Clock::time_point deadline) const {
    const auto remaining = deadline - Clock::now();
    if (remaining <= Clock::duration::zero()) {
        return -1;
    }
    const auto remaining_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(remaining).count();
    const auto timeout_ms = remaining_ns / 1'000'000 +
                            (remaining_ns % 1'000'000 == 0 ? 0 : 1);
    return static_cast<int>(std::min<std::int64_t>(
        std::numeric_limits<int>::max(), std::max<std::int64_t>(0, timeout_ms)));
}

runtime::SourceResult SerialImuSource::timeout_result() {
    const ssize_t pending_bytes = port_->input_bytes_available();
    if (pending_bytes >= 0) {
        const auto pending_bytes_value =
            static_cast<std::size_t>(pending_bytes);
        remember_pending_bytes(pending_bytes_value);
        if (has_timestamp_) {
            report_runtime_backlog(pending_bytes_value);
        }
    } else {
        invalidate_pending_bytes();
    }

    const auto timeout_count = timeout_count_.fetch_add(1) + 1;
    const auto now = Clock::now();
    if (timeout_count == 1 || now >= next_timeout_log_) {
        std::ostringstream message;
        message << "警告: IMU 读取 watchdog 超时(累计 " << timeout_count
                << " 次), buffered_bytes=" << buffered_size_;
        if (pending_bytes >= 0) {
            const auto pending_bytes_value =
                static_cast<std::size_t>(pending_bytes);
            message << ", pending_bytes=" << pending_bytes_value
                    << ", pending_frames="
                    << pending_bytes_value / serial_package::kImuFrameSize;
        } else {
            message << ", pending_bytes=N/A（查询失败: "
                    << port_error_or(*port_, "未知串口错误") << ")";
        }
        log(message.str());
        next_timeout_log_ = now + std::chrono::seconds(1);
    }
    return {runtime::SourceStatus::timeout, "IMU 读取超时"};
}

}  // namespace mosas::app
