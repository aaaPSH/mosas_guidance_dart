#ifndef MOSAS_ADAPTERS_SERIAL_IMU_SOURCE_HPP
#define MOSAS_ADAPTERS_SERIAL_IMU_SOURCE_HPP

#include <mosas/adapters/imu_stream_decoder.hpp>
#include <mosas/adapters/raw_imu_receive_buffer.hpp>
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
    /**
     * @brief 创建串口 IMU 适配器并配置原始块/解码样本队列。
     * @param port 已配置的串口对象；必须保持共享所有权并在启动前打开。
     * @param enable_interval_statistics 是否累计成功交付样本的时间间隔。
     * @param measurement_latency_ns IMU 测量到串口接收时间的延迟补偿（ns），不得为负。
     * @param receive_config 原始接收线程的块大小和调度策略。
     * @param timing 实际采样频率与同步窗口。
     * @note 协议帧解码发生在 IMU 消费线程；时间补偿不会使无设备时间戳的样本变为精确时间。
     */
    SerialImuSource(
        std::shared_ptr<serial_package::SerialPort> port,
        bool enable_interval_statistics,
        runtime::TimestampNs measurement_latency_ns,
        const serial_package::ReceiveThreadConfig& receive_config,
        domain::SequenceTimeConfig timing);

    bool uses_sequence_time() const noexcept override { return true; }
    /** @param sync 启动前绑定的同步屏障。 */
    void bind_acquisition_sync(std::shared_ptr<domain::AcquisitionSync> sync) override {
        decoder_.bind_sync(std::move(sync));
    }

    /**
     * @brief 启动串口异步接收线程并重置解析状态。
     * @return 数据源启动结果；串口未打开或配置无效时返回 fatal。
     * @note 原始字节接收与协议解码分线程执行，解码在 try_pop 的消费线程中推进。
     */
    runtime::SourceResult start() override;

    /**
     * @brief 非阻塞地取得一个已解码的加速度和角速度样本。
     * @param sample 非空输出指针；成功时写入样本。
     * @return 有有效样本可用时为 true，否则为 false。
     */
    bool try_pop(runtime::ImuAngularVelocitySample* sample) noexcept override;

    /**
     * @brief 等待串口原始接收队列有数据或数据源关闭。
     * @return 数据源仍可运行时为 true；取消或关闭时为 false。
     */
    bool wait_for_data() noexcept override;

    /** @brief 非阻塞请求停止接收并唤醒等待者。 */
    void cancel() noexcept override;
    /** @brief 停止接收线程并回收串口接收资源。 */
    void stop() noexcept override;

    /**
     * @brief 获取成功交付样本之间的时间间隔统计。
     * @return 间隔次数、总时长及最小/最大间隔；未启用统计时计数为零。
     * @note 调用方应确保样本读取线程已停止。
     */
    SerialImuIntervalStatistics interval_statistics() const noexcept;

    /**
     * @brief 获取串口块队列、解码和样本丢弃计数的线程安全快照。
     * @return IMU 数据源诊断；此适配器始终提供诊断。
     */
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

#endif  // MOSAS_ADAPTERS_SERIAL_IMU_SOURCE_HPP
