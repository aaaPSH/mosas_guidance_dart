#ifndef MOSAS_APP_SERIAL_IMU_SOURCE_HPP
#define MOSAS_APP_SERIAL_IMU_SOURCE_HPP

#include <mosas/app/imu_stream_decoder.hpp>
#include <mosas/app/raw_imu_receive_buffer.hpp>
#include <mosas/runtime/runtime_interfaces.hpp>
#include <serial_package/serial_port.hpp>

#include <atomic>
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
    SerialImuSource(std::shared_ptr<serial_package::SerialPort> port,
                    double sample_rate_hz, int read_timeout_ms,
                    bool enable_interval_statistics, std::ostream& output,
                    runtime::TimestampNs measurement_latency_ns);
    SerialImuSource(std::shared_ptr<serial_package::SerialPort> port,
                    double sample_rate_hz, int read_timeout_ms,
                    bool enable_interval_statistics, std::ostream& output,
                    runtime::TimestampNs measurement_latency_ns,
                    const serial_package::ReceiveThreadConfig& receive_config);

    // 启动串口原始字节接收线程；协议解析在 try_pop 所在线程完成。
    runtime::SourceResult start() override;

    // 非阻塞取出一个已经解析的三轴加速度和角速度样本。
    bool try_pop(runtime::ImuAngularVelocitySample* sample) noexcept override;

    // 兼容旧调用方：在普通调用线程等待并返回一个 IMU 运动样本。
    runtime::SourceResult read(runtime::ImuSample* sample) override;
    void cancel() noexcept override;
    void stop() noexcept override;

    // 该字段已不进入运行时状态；保留接口以兼容旧诊断调用方，始终返回 0。
    std::uint16_t initialization_g_raw() const noexcept;

    // 返回有效 IMU 样本时间戳的间隔统计值；调用方应在读取线程停止后调用。
    SerialImuIntervalStatistics interval_statistics() const noexcept;

    // 返回串口输入队列、原始块和解码错误的线程安全快照。
    runtime::ImuSourceDiagnostics diagnostics() const noexcept override;

private:
    static constexpr std::size_t kRawQueueCapacity = 256;
    static constexpr std::size_t kDecodedQueueCapacity = 512;

    bool deliver_sample(runtime::ImuAngularVelocitySample* sample) noexcept;
    runtime::SourceResult fatal_result(const char* message);
    runtime::SourceResult timeout_result() noexcept;

    std::shared_ptr<serial_package::SerialPort> port_;
    std::ostream* output_ = nullptr;
    int read_timeout_ms_ = 0;
    runtime::TimestampNs measurement_latency_ns_ = 0;
    std::string configuration_error_;
    bool interval_statistics_enabled_ = false;

    RawImuReceiveBuffer raw_receive_buffer_;
    ImuStreamDecoder decoder_;
    serial_package::ReceiveThreadConfig receive_config_{};

    mutable std::mutex lifecycle_mutex_;
    std::atomic<bool> started_{false};
    std::atomic<bool> cancelled_{false};
    SerialImuIntervalStatistics interval_statistics_{};
    bool has_interval_timestamp_ = false;
    runtime::TimestampNs last_interval_timestamp_ns_ = -1;

    std::atomic<runtime::TimestampNs> last_sample_timestamp_ns_{-1};
    std::atomic<std::uint64_t> delivered_sample_count_{0};
    std::atomic<std::uint64_t> timeout_count_{0};
};

}  // namespace mosas::app

#endif  // MOSAS_APP_SERIAL_IMU_SOURCE_HPP
