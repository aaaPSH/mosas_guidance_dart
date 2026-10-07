#ifndef MOSAS_APP_SERIAL_IMU_SOURCE_HPP
#define MOSAS_APP_SERIAL_IMU_SOURCE_HPP

#include <mosas/app/imu_stream_decoder.hpp>
#include <mosas/app/raw_imu_receive_buffer.hpp>
#include <mosas/runtime/runtime_interfaces.hpp>
#include <serial_package/serial_port.hpp>

#include <atomic>
#include <cstdint>
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
    SerialImuSource(
        std::shared_ptr<serial_package::SerialPort> port,
        bool enable_interval_statistics,
        runtime::TimestampNs measurement_latency_ns,
        const serial_package::ReceiveThreadConfig& receive_config);

    // 启动串口原始字节接收线程；协议解析在消费样本的线程完成。
    runtime::SourceResult start() override;

    // 非阻塞取出一个已经解析的三轴加速度和角速度样本。
    bool try_pop(runtime::ImuAngularVelocitySample* sample) noexcept override;
    bool wait_for_data() noexcept override;

    void cancel() noexcept override;
    void stop() noexcept override;

    // 返回有效 IMU 样本时间戳的间隔统计值；调用方应在读取线程停止后调用。
    SerialImuIntervalStatistics interval_statistics() const noexcept;

    // 返回串口输入队列、原始块和解码错误的线程安全快照。
    runtime::ImuSourceDiagnostics diagnostics() const noexcept override;

private:
    static constexpr std::size_t kRawQueueCapacity = 256;
    static constexpr std::size_t kDecodedQueueCapacity = 512;

    bool deliver_sample(runtime::ImuAngularVelocitySample* sample) noexcept;
    runtime::SourceResult fatal_result(const char* message);

    std::shared_ptr<serial_package::SerialPort> port_;
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
};

}  // namespace mosas::app

#endif  // MOSAS_APP_SERIAL_IMU_SOURCE_HPP
