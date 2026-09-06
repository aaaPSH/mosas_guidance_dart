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

class SerialImuSource final : public runtime::ImuSource {
public:
    SerialImuSource(std::shared_ptr<serial_package::SerialPort> port,
                    double sample_rate_hz, int read_timeout_ms);
    SerialImuSource(std::shared_ptr<serial_package::SerialPort> port,
                    double sample_rate_hz, int read_timeout_ms,
                    std::ostream& output);

    runtime::SourceResult read(runtime::ImuSample* sample) override;
    void cancel() noexcept override;

    // 返回首次捕获的下位机初始化 g raw 值；后续 0 不会覆盖它。
    std::uint16_t initialization_g_raw() const noexcept;

private:
    static constexpr std::size_t kMaxRawBufferSize = 4096;
    using Clock = std::chrono::steady_clock;

    void align_buffer();
    void discard_prefix(std::size_t count, const std::string& reason);
    void remember_initialization_g(std::uint16_t raw_value);
    void log(const std::string& message);
    runtime::SourceResult fatal_result(const std::string& message);
    int remaining_timeout_ms(Clock::time_point deadline) const;
    runtime::SourceResult timeout_result();

    std::shared_ptr<serial_package::SerialPort> port_;
    std::ostream* output_ = nullptr;
    std::chrono::nanoseconds period_{0};
    int watchdog_timeout_ms_ = 0;
    std::string configuration_error_;

    std::array<std::uint8_t, kMaxRawBufferSize> raw_buffer_{};
    std::size_t buffered_size_ = 0;
    std::atomic<bool> cancelled_{false};
    std::atomic<std::uint16_t> initialization_g_raw_{0};

    bool has_timestamp_ = false;
    runtime::TimestampNs last_timestamp_ns_ = 0;
    std::uint64_t timeout_count_ = 0;
    std::uint64_t parse_error_count_ = 0;
    std::uint64_t discarded_noise_count_ = 0;
    bool startup_reported_ = false;
    mutable std::mutex log_mutex_;
};

}  // namespace mosas::app

#endif  // MOSAS_APP_SERIAL_IMU_SOURCE_HPP
