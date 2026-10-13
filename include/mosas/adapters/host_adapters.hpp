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
    void stop() noexcept override;
    Diagnostics diagnostics() const noexcept override;

private:
    runtime::AsyncInfoLogger logger_;
    std::atomic<std::uint64_t> submitted_count_{0};
};

/// 通过 latest-value mailbox 和专属发送线程将制导命令写入串口。
class SerialCommandSink final : public runtime::CommandSink {
public:
    /**
     * @brief 创建串口命令 sink，错误信息默认写入标准错误流。
     * @param port 已配置的串口对象；须在 start() 前打开。
     */
    explicit SerialCommandSink(
        std::shared_ptr<serial_package::SerialPort> port);

    /**
     * @brief 创建带指定诊断输出流的串口命令 sink。
     * @param port 已配置的串口对象；须在 start() 前打开。
     * @param output 写入串口发送错误文本的流；其生命周期须覆盖 sink。
     */
    SerialCommandSink(std::shared_ptr<serial_package::SerialPort> port,
                      std::ostream& output);
    ~SerialCommandSink() override;

    /** @brief 检查串口并启动唯一的后台命令发送线程。 @return 成功启动时为 true。 */
    bool start() override;
    /**
     * @brief 非阻塞地发布当前起飞代际的最新命令。
     * @param command 控制命令；代际与当前 active generation 不符时拒绝。
     * @return 命令进入 mailbox 时为 true；未运行、已取消或代际失效时为 false。
     * @note 替换尚未发送的旧命令；本调用不执行串口写入。
     */
    bool try_submit(const runtime::GuidanceCommand& command) noexcept override;
    /**
     * @brief 更新允许发送的起飞代际并同步串口发送临界区。
     * @param generation 新的有效代际。
     * @note 可能等待当前串口写操作完成；返回后旧代际在途写入已结束。
     */
    void set_active_generation(std::uint64_t generation) noexcept override;
    /** @brief 请求发送线程停止接受/处理命令。 */
    void cancel() noexcept override;
    /** @brief 关闭 mailbox 并等待发送线程退出。 */
    void stop() noexcept override;
    /** @brief 获取命令提交、队列替代和串口失败计数。 @return 命令 sink 诊断快照。 */
    Diagnostics diagnostics() const noexcept override;

    /** @brief 获取最近一次串口发送错误。 @return 错误文本；无错误时为空。 */
    std::string last_error() const;

private:
    /**
     * @brief 消费 mailbox、校验命令代际并串行编码/写出控制帧。
     * @note 串口写入只在该 worker 执行；失败计数和错误说明在 sink 中保留。
     */
    void worker_loop() noexcept;
    bool fail(const std::string& message);

    std::shared_ptr<serial_package::SerialPort> port_;
    std::ostream* output_ = nullptr;
    runtime::LatestValueMailbox<runtime::GuidanceCommand> mailbox_;
    std::atomic<bool> stop_requested_{false};
    std::atomic<bool> cancel_requested_{false};
    std::atomic<bool> running_{false};
    std::atomic<std::uint64_t> submitted_count_{0};
    std::atomic<std::uint64_t> send_failure_count_{0};
    std::atomic<std::uint64_t> active_generation_{0};
    std::thread worker_;
    mutable std::mutex mutex_;
    std::mutex send_mutex_;
    std::string last_error_;
};

}  // mosas::app 命名空间结束

#endif  // MOSAS_ADAPTERS_HOST_ADAPTERS_HPP
