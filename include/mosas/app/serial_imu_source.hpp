#ifndef MOSAS_APP_SERIAL_IMU_SOURCE_HPP
#define MOSAS_APP_SERIAL_IMU_SOURCE_HPP

#include <mosas/runtime/runtime_interfaces.hpp>
#include <serial_package/serial_port.hpp>

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <iosfwd>
#include <memory>
#include <mutex>
#include <string>

namespace mosas::app {

struct SerialImuIntervalStatistics {
    std::uint64_t interval_count = 0;
    std::uint64_t total_interval_ns = 0;
    std::int64_t minimum_interval_ns = 0;
    std::int64_t maximum_interval_ns = 0;
};

class SerialImuSource final : public runtime::ImuSource {
public:
    SerialImuSource(std::shared_ptr<serial_package::SerialPort> port,
                    double sample_rate_hz, int read_timeout_ms);
    SerialImuSource(std::shared_ptr<serial_package::SerialPort> port,
                    double sample_rate_hz, int read_timeout_ms,
                    std::ostream& output);
    SerialImuSource(std::shared_ptr<serial_package::SerialPort> port,
                    double sample_rate_hz, int read_timeout_ms,
                    bool enable_interval_statistics);
    SerialImuSource(std::shared_ptr<serial_package::SerialPort> port,
                    double sample_rate_hz, int read_timeout_ms,
                    bool enable_interval_statistics, std::ostream& output);

    runtime::SourceResult read(runtime::ImuSample* sample) override;
    void cancel() noexcept override;

    // 返回首次捕获的下位机初始化 g raw 值；后续 0 不会覆盖它。
    std::uint16_t initialization_g_raw() const noexcept;

    // 返回有效 IMU 帧逻辑时间轴的间隔统计值；调用方应在读取线程停止后调用。
    SerialImuIntervalStatistics interval_statistics() const noexcept;

    // 返回串口输入队列和接收错误的线程安全快照。
    runtime::ImuSourceDiagnostics diagnostics() const noexcept override;

private:
    static constexpr std::size_t kMaxRawBufferSize = 4096;
    using Clock = std::chrono::steady_clock;

    void align_buffer();
    void discard_prefix(std::size_t count, const std::string& reason);
    void remember_initialization_g(std::uint16_t raw_value);
    void remember_pending_bytes(std::size_t pending_bytes) noexcept;
    void invalidate_pending_bytes() noexcept;
    void report_runtime_backlog(std::size_t pending_bytes);
    runtime::TimestampNs timestamp_for_frame(
        runtime::TimestampNs received_timestamp_ns,
        std::size_t pending_frame_count) const;
    void log(const std::string& message);
    runtime::SourceResult fatal_result(const std::string& message);
    int remaining_timeout_ms(Clock::time_point deadline) const;
    runtime::SourceResult timeout_result();

    std::shared_ptr<serial_package::SerialPort> port_;
    std::ostream* output_ = nullptr;
    std::chrono::nanoseconds period_{0};
    int watchdog_timeout_ms_ = 0;
    std::string configuration_error_;
    bool interval_statistics_enabled_ = false;
    SerialImuIntervalStatistics interval_statistics_{};

    std::array<std::uint8_t, kMaxRawBufferSize> raw_buffer_{};
    std::size_t buffered_size_ = 0;
    std::atomic<bool> cancelled_{false};
    std::atomic<std::uint16_t> initialization_g_raw_{0};
    std::atomic<bool> pending_bytes_valid_{false};
    std::atomic<std::size_t> pending_bytes_{0};
    std::atomic<std::size_t> buffered_bytes_{0};
    std::atomic<runtime::TimestampNs> last_sample_timestamp_ns_{-1};
    std::atomic<std::uint64_t> delivered_sample_count_{0};
    std::atomic<std::uint64_t> timeout_count_{0};
    std::atomic<std::uint64_t> parse_error_count_{0};
    std::atomic<std::uint64_t> backlog_event_count_{0};

    bool has_timestamp_ = false;
    runtime::TimestampNs last_timestamp_ns_ = 0;
    std::uint64_t discarded_noise_count_ = 0;
    bool startup_reported_ = false;
    bool backlog_active_ = false;
    Clock::time_point next_backlog_log_{};
    Clock::time_point next_timeout_log_{};
    mutable std::mutex log_mutex_;
};

}  // namespace mosas::app

#endif  // MOSAS_APP_SERIAL_IMU_SOURCE_HPP
