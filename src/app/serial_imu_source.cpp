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

std::int64_t maximum_interval_ns(std::int64_t period_ns) noexcept {
    const std::int64_t half_period_ns = period_ns / 2;
    const std::int64_t maximum_value =
        std::numeric_limits<std::int64_t>::max();
    if (period_ns > maximum_value - half_period_ns) {
        return maximum_value;
    }
    return period_ns + half_period_ns;
}

}  // 匿名命名空间结束

SerialImuSource::SerialImuSource(
    std::shared_ptr<serial_package::SerialPort> port, double sample_rate_hz,
    int read_timeout_ms)
    : SerialImuSource(std::move(port), sample_rate_hz, read_timeout_ms,
                      std::cerr) {}

SerialImuSource::SerialImuSource(
    std::shared_ptr<serial_package::SerialPort> port, double sample_rate_hz,
    int read_timeout_ms, std::ostream& output)
    : port_(std::move(port)), output_(&output) {
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

    const auto period_ms_floor = period_.count() / 1'000'000;
    const auto period_ms_ceil =
        period_ms_floor + (period_.count() % 1'000'000 == 0 ? 0 : 1);
    const auto bounded_period_ms = std::min<std::int64_t>(
        std::numeric_limits<int>::max(), std::max<std::int64_t>(1,
                                                                 period_ms_ceil));
    watchdog_timeout_ms_ = std::min(
        read_timeout_ms, static_cast<int>(bounded_period_ms));
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
            return fatal_result("查询启动时串口输入队列失败: " +
                                port_error_or(*port_, "未知串口错误"));
        }
        startup_reported_ = true;
        std::ostringstream message;
        message << "启动输入队列有 " << startup_bytes
                << " 字节；若发现启动积压，将清理旧数据并等待新 IMU 帧";
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
                ++parse_error_count_;
                had_parse_error = true;
                if (parse_error_count_ == 1 ||
                    parse_error_count_ % 100 == 0) {
                    std::ostringstream message;
                    message << "候选帧解析失败(累计 " << parse_error_count_
                            << " 次): " << decode_error << ", data="
                            << hex_dump(raw_buffer_.data(),
                                        serial_package::kImuFrameSize);
                    log(message.str());
                }
                discard_prefix(1, "非法候选帧重新同步");
                continue;
            }

            // 当前读取调用只允许交付这一帧，不能把完整帧留在源内部。
            buffered_size_ = 0;
            const runtime::TimestampNs timestamp_ns = steady_timestamp_ns();
            if (has_timestamp_) {
                const auto interval_ns = timestamp_ns - last_timestamp_ns_;
                const auto minimum_interval = period_.count() / 2;
                const auto maximum_interval =
                    maximum_interval_ns(period_.count());
                if (interval_ns <= 0) {
                    std::ostringstream message;
                    message << "IMU 时间戳不递增: previous="
                            << last_timestamp_ns_ << ", current="
                            << timestamp_ns;
                    return fatal_result(message.str());
                }
                if (interval_ns < minimum_interval ||
                    interval_ns > maximum_interval) {
                    std::ostringstream message;
                    message << "IMU 采样间隔超限: interval_ns=" << interval_ns
                            << ", expected=[" << minimum_interval << ','
                            << maximum_interval << "]";
                    return fatal_result(message.str());
                }
            }

            const ssize_t pending_bytes = port_->input_bytes_available();
            if (pending_bytes < 0) {
                return fatal_result("查询串口输入积压失败: " +
                                    port_error_or(*port_, "未知串口错误"));
            }
            if (pending_bytes >=
                static_cast<ssize_t>(serial_package::kImuFrameSize)) {
                if (!has_timestamp_) {
                    remember_initialization_g(values.initialization_g_raw);
                    std::ostringstream message;
                    message << "检测到启动阶段串口输入积压 " << pending_bytes
                            << " 字节，清理旧数据并等待新的 IMU 帧";
                    log(message.str());
                    if (!port_->flush_input()) {
                        return fatal_result("清理启动阶段串口输入队列失败: " +
                                            port_error_or(
                                                *port_, "未知串口错误"));
                    }
                    // 当前候选帧已经被消费，但它属于启动旧数据。
                    buffered_size_ = 0;
                    continue;
                }
                std::ostringstream message;
                message << "串口输入积压 " << pending_bytes
                        << " 字节，至少有一个完整 IMU 帧未处理，拒绝继续积分";
                return fatal_result(message.str());
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
    ++timeout_count_;
    if (timeout_count_ == 1 || timeout_count_ % 100 == 0) {
        std::ostringstream message;
        message << "IMU 读取 watchdog 超时(累计 " << timeout_count_ << " 次)";
        log(message.str());
    }
    if (has_timestamp_) {
        const runtime::TimestampNs now_ns = steady_timestamp_ns();
        const auto elapsed_ns = now_ns - last_timestamp_ns_;
        const auto maximum_interval = maximum_interval_ns(period_.count());
        if (elapsed_ns > maximum_interval) {
            std::ostringstream message;
            message << "IMU 连续超时导致采样间隔超限: elapsed_ns="
                    << elapsed_ns << ", maximum_ns=" << maximum_interval;
            return fatal_result(message.str());
        }
    }
    return {runtime::SourceStatus::timeout, "IMU 读取超时"};
}

}  // namespace mosas::app
