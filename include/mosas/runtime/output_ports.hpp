#ifndef MOSAS_RUNTIME_OUTPUT_PORTS_HPP
#define MOSAS_RUNTIME_OUTPUT_PORTS_HPP

#include <mosas/domain/contracts.hpp>
#include <mosas/runtime/log_event.hpp>
#include <mosas/runtime/output_diagnostics.hpp>
#include <mosas/runtime/processed_frame.hpp>

#include <memory>
#include <string>

namespace mosas::runtime {

/// 帧输出 sink 的实现由各自 worker 调用；record/publish 可执行编码和 I/O。
/// 制导路径只与 worker 的有界非阻塞队列交互，不直接调用这些慢操作。
class IVideoRecorder {
public:
    virtual ~IVideoRecorder() = default;

    /** @brief 在输出 worker 启动前打开录像所需资源。 @return 启动成功时为 true。 */
    virtual bool start() = 0;
    /** @brief 清除上一轮运行的缓存和错误状态。 */
    virtual void reset() noexcept = 0;
    /**
     * @brief 写入一帧处理结果。
     * @param frame 带图像所有权、时间、视觉和制导快照的不可变帧。
     * @return 该帧处理成功时为 true。
     * @note 可能阻塞于编码或文件 I/O；仅由录像 worker 调用。
     */
    virtual bool record(const ProcessedFrame& frame) = 0;
    /**
     * @brief 取出起飞窗口完成后请求重新布防的通知。
     * @return 存在尚未消费的请求时为 true；非窗口式实现默认返回 false。
     */
    virtual bool take_rearm_request() noexcept { return false; }
    /** @brief 请求中止录像工作；不替代 stop() 的资源回收。 */
    virtual void cancel() noexcept = 0;
    /** @brief 关闭录像资源并完成待处理清理。 */
    virtual void stop() noexcept = 0;
    /** @brief 获取最近一次错误说明。 @return 错误文本；无错误时为空。 */
    virtual std::string last_error() const = 0;
};

/// 逐帧遥测 CSV 输出端口；调用线程和阻塞语义与 IVideoRecorder 相同。
class ITelemetryRecorder {
public:
    virtual ~ITelemetryRecorder() = default;

    /** @brief 打开遥测输出资源。 @return 启动成功时为 true。 */
    virtual bool start() = 0;
    /** @brief 清除上一轮运行的缓冲和错误状态。 */
    virtual void reset() noexcept = 0;
    /**
     * @brief 记录一帧遥测快照。
     * @param frame 与视频输出共用的不可变处理结果。
     * @return 写入成功时为 true。
     * @note 可能执行阻塞式文件 I/O；仅由遥测 worker 调用。
     */
    virtual bool record(const ProcessedFrame& frame) = 0;
    /** @brief 取出起飞窗口完成后的重新布防请求。 @return 有待处理请求时为 true。 */
    virtual bool take_rearm_request() noexcept { return false; }
    /** @brief 请求停止遥测记录。 */
    virtual void cancel() noexcept = 0;
    /** @brief 关闭文件并完成待处理清理。 */
    virtual void stop() noexcept = 0;
    /** @brief 获取最近一次错误说明。 @return 错误文本；无错误时为空。 */
    virtual std::string last_error() const = 0;
};

/// 可视化输出端口；发布工作与控制路径隔离。
class IVisualizationSink {
public:
    virtual ~IVisualizationSink() = default;

    /** @brief 初始化可视化输出。 @return 初始化成功时为 true。 */
    virtual bool start() = 0;
    /** @brief 清除上一轮可视化状态。 */
    virtual void reset() noexcept = 0;
    /**
     * @brief 发布一帧处理结果供本地显示或网络传输。
     * @param frame 带图像和制导叠加信息的不可变帧。
     * @return 发布成功时为 true。
     * @note 可能执行渲染、编码或网络 I/O；仅由可视化 worker 调用。
     */
    virtual bool publish(const ProcessedFrame& frame) = 0;
    /** @brief 请求停止发布工作。 */
    virtual void cancel() noexcept = 0;
    /** @brief 释放可视化资源。 */
    virtual void stop() noexcept = 0;
    /** @brief 获取最近一次错误说明。 @return 错误文本；无错误时为空。 */
    virtual std::string last_error() const = 0;
};

/// IMU 遥测记录端口；样本提交在 IMU 控制链上必须有界且非阻塞。
class IImuRecorder {
public:
    virtual ~IImuRecorder() = default;

    /** @brief 启动后台记录资源。 @return 启动成功时为 true。 */
    virtual bool start() = 0;
    /**
     * @brief 非阻塞地提交一条 IMU 运动样本供后台持久化。
     * @param sample 带时间戳的领域 IMU 样本。
     * @return 已接收时为 true；队列满或 sink 不可用时为 false。
     */
    virtual bool try_record(
        const domain::ImuKinematicSample& sample) noexcept = 0;
    /** @brief 返回记录队列和错误计数的快照。 @return sink 诊断信息。 */
    virtual SinkDiagnostics diagnostics() const noexcept = 0;
    /** @brief 请求停止后台记录。 */
    virtual void cancel() noexcept = 0;
    /** @brief 停止 worker 并关闭记录资源。 */
    virtual void stop() noexcept = 0;
};

/// 异步结构化日志端口；拒绝日志不得回退到控制线程同步 I/O。
class IAsyncLogger {
public:
    virtual ~IAsyncLogger() = default;

    /** @brief 启动日志 worker。 @return 启动成功时为 true。 */
    virtual bool start() = 0;
    /**
     * @brief 非阻塞地提交固定大小日志事件。
     * @param event 含单调时钟时间和结构化消息字段的日志事件。
     * @return 被队列接收时为 true；竞争、队列满或 logger 不可用时可立即拒绝并返回 false。
     */
    virtual bool try_submit(const RuntimeLogEvent& event) noexcept = 0;
    /** @brief 返回日志队列和写入错误快照。 @return logger 诊断信息。 */
    virtual SinkDiagnostics diagnostics() const noexcept = 0;
    /**
     * @brief 停止日志 worker，并在返回前尽力刷新已接收事件。
     */
    virtual void stop_and_flush() noexcept = 0;
};

struct RuntimeOutputPorts {
    std::unique_ptr<IVideoRecorder> video;
    std::unique_ptr<ITelemetryRecorder> telemetry;
    std::unique_ptr<IVisualizationSink> visualization;
    std::unique_ptr<IImuRecorder> imu_recorder;
    std::unique_ptr<IAsyncLogger> logger;
};

}  // namespace mosas::runtime

#endif  // MOSAS_RUNTIME_OUTPUT_PORTS_HPP
