#ifndef MOSAS_ADAPTERS_HOST_ADAPTERS_HPP
#define MOSAS_ADAPTERS_HOST_ADAPTERS_HPP

#include <mosas/app/app_config.hpp>
#include <mosas/runtime/async_info_logger.hpp>
#include <mosas/runtime/latest_value_mailbox.hpp>
#include <serial_package/serial_port.hpp>

#include <chrono>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <iosfwd>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace mosas::app {

class SimulatedImuSource final : public runtime::ImuSource {
public:
    explicit SimulatedImuSource(double sample_rate_hz,
                                double gravity_mps2 = 9.80665,
                                bool accelerometer_specific_force = true);

    runtime::SourceResult start() override;
    bool try_pop(runtime::ImuAngularVelocitySample* sample) noexcept override;
    bool wait_for_data() noexcept override;
    void cancel() noexcept override;
    void stop() noexcept override;

private:
    std::chrono::nanoseconds period_;
    double gravity_mps2_ = 9.80665;
    bool accelerometer_specific_force_ = true;
    std::mutex mutex_;
    std::condition_variable condition_;
    std::chrono::steady_clock::time_point next_deadline_{};
    bool started_ = false;
    bool cancelled_ = false;
};

class LoggingCommandSink final : public runtime::CommandSink {
public:
    LoggingCommandSink();
    explicit LoggingCommandSink(std::ostream& output);

    bool start() override;
    bool try_submit(const runtime::GuidanceCommand& command) noexcept override;
    /** @param generation 新代际；同步本地日志映射边界，不重新发布旧请求。 */
    void set_active_generation(std::uint64_t generation) noexcept override;
    void stop() noexcept override;
    Diagnostics diagnostics() const noexcept override;

private:
    std::mutex emission_mutex_;
    std::uint64_t active_generation_ = 0;
    runtime::AsyncInfoLogger logger_;
    std::atomic<std::uint64_t> submitted_count_{0};
};

/** @brief 通过 latest-value mailbox 和专属线程周期发送指令；正常制导无新鲜请求时清零，定时调试可独立触发。 */
class SerialCommandSink final : public runtime::CommandSink {
public:
    /**
     * @brief 创建按固定频率发送的串口命令 sink。
     * @param port 已配置的串口对象；须在 start() 前打开。
     * @param config 发送频率和命令超时；不改变制导计算和发射门槛。
     * @param output 写入串口发送错误文本的流；其生命周期须覆盖 sink。
     */
    SerialCommandSink(std::shared_ptr<serial_package::SerialPort> port,
                      const CommandAppConfig& config, std::ostream& output);
    ~SerialCommandSink() override;

    /** @brief 检查参数并启动周期发送线程，初始发送零过载。 @return 成功启动时为 true。 */
    bool start() override;
    /**
     * @brief 非阻塞地发布当前起飞代际的最新命令。
     * @param command 控制命令；代际与当前 active generation 不符时拒绝。
     * @return 命令进入 mailbox 时为 true；未运行、已取消或代际失效时为 false。
     * @note 替换尚未发送的旧命令并记录提交时刻；本调用不执行串口写入。
     *       只有新提交会更新有效期，周期重发不会延长有效期。
     */
    bool try_submit(const runtime::GuidanceCommand& command) noexcept override;
    /**
     * @brief 更新允许发送的起飞代际并同步串口发送临界区。
     * @param generation 新的有效代际。
     * @note 可能等待当前串口写操作完成；返回后旧代际在途写入已结束。
     *       新代际在收到新命令前周期发送零过载和零状态标志。
     */
    void set_active_generation(std::uint64_t generation) noexcept override;
    /** @brief 拒绝新命令并唤醒发送线程退出；不会继续周期重发。 */
    void cancel() noexcept override;
    /** @brief 关闭 mailbox 并等待发送线程退出。 */
    void stop() noexcept override;
    /** @brief 获取命令提交、队列替代和串口失败计数。 @return 命令 sink 诊断快照。 */
    Diagnostics diagnostics() const noexcept override;

    /** @brief 获取最近一次串口发送错误。 @return 错误文本；无错误时为空。 */
    std::string last_error() const;

private:
    struct PendingCommand {
        runtime::GuidanceCommand command{};
        std::chrono::steady_clock::time_point submitted_at{};
    };

    /**
     * @brief 按单调时钟周期采样最新指令，检查代际、发射门槛和提交超时。
     * @note 正常制导无新指令时以最新姿态重新映射最近弹道请求；超时清零过载和识别位，但保留发射标志。
     *       两种固定模式均独立使用当前 IMU 发射状态；空 mailbox 不阻止发布上升状态。
     *       launch_delay 不要求视觉请求；target_detected 仅由未过期的发射后识别结果首次触发。
     *       调度或写入超时后跳过错过的周期，不突发补发积压指令。
     */
    void worker_loop() noexcept;
    /**
     * @brief 编码并写出一帧，保留发送失败计数和错误说明。
     * @param command 已通过代际、发射门槛及有效期检查的待发送指令。
     * @note 仅由发送线程在持有 send_mutex_ 时调用。
     */
    bool send_command(const domain::MappedGuidanceCommand& command) noexcept;
    bool fail(const std::string& message);

    std::shared_ptr<serial_package::SerialPort> port_;
    CommandAppConfig config_;
    std::ostream* output_ = nullptr;
    runtime::LatestValueMailbox<PendingCommand> mailbox_;
    std::atomic<bool> stop_requested_{false};
    std::atomic<bool> cancel_requested_{false};
    std::atomic<bool> running_{false};
    std::atomic<std::uint64_t> submitted_count_{0};
    std::atomic<std::uint64_t> send_failure_count_{0};
    std::atomic<std::uint64_t> active_generation_{0};
    std::thread worker_;
    mutable std::mutex mutex_;
    std::mutex send_mutex_;
    std::mutex wait_mutex_;
    std::condition_variable wait_condition_;
    std::string last_error_;
};

}  // mosas::app 命名空间结束

#endif  // MOSAS_ADAPTERS_HOST_ADAPTERS_HPP
