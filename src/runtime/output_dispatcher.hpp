#ifndef MOSAS_RUNTIME_OUTPUT_DISPATCHER_HPP
#define MOSAS_RUNTIME_OUTPUT_DISPATCHER_HPP

#include <mosas/runtime/latest_frame_queue.hpp>
#include <mosas/runtime/output_ports.hpp>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace mosas::runtime {

enum class OutputDispatchTimingStage {
    queue_wait,
    queue_delay,
    publish,
};

struct OutputDispatcherCallbacks {
    std::function<bool()> stop_requested;
    std::function<std::uint64_t()> active_generation;
    std::function<double()> processing_fps;
    std::function<void(OutputDispatchTimingStage,
                       std::chrono::steady_clock::duration)> record_timing;
    std::function<void()> record_published_frame;
    std::function<void()> request_rearm;
    std::function<void(std::string)> report_fault;
};

class OutputFrameWorker;

/// 将同一处理帧分发至互相隔离的有界输出 worker。
class OutputDispatcher final {
public:
    /**
     * @brief 创建独立的帧输出 worker 和可选 IMU 记录端口。
     * @param video_recorder 本地视频 sink，可为空。
     * @param telemetry_recorder 逐帧遥测 sink，可为空。
     * @param visualization_sink 可视化 sink，可为空。
     * @param imu_recorder IMU 样本记录 sink，可为空。
     * @param queue_capacity 每个帧 sink 独立队列的容量。
     * @param callbacks 生命周期、时序、重新布防和故障回调。
     */
    OutputDispatcher(std::unique_ptr<IVideoRecorder> video_recorder,
                     std::unique_ptr<ITelemetryRecorder> telemetry_recorder,
                     std::unique_ptr<IVisualizationSink> visualization_sink,
                     std::unique_ptr<IImuRecorder> imu_recorder,
                     std::size_t queue_capacity,
                     OutputDispatcherCallbacks callbacks);
    ~OutputDispatcher();

    OutputDispatcher(const OutputDispatcher&) = delete;
    OutputDispatcher& operator=(const OutputDispatcher&) = delete;

    /** @brief 清空各 worker 队列和诊断，为新一轮运行复位 sink。 */
    void reset() noexcept;
    /** @brief 启动 IMU recorder 和所有已配置帧 sink worker。 */
    void start();

    /**
     * @brief 将不可变处理帧分别提交到各帧 sink 的有界队列。
     * @param snapshot 帧、时间、视觉、姿态和制导结果的快照。
     * @return 所有已启用帧队列均接受该帧时为 true；队列拒绝时为 false。
     * @note 只做有界 fan-out，不执行视频编码、文件或网络 I/O。
     */
    bool enqueue(ProcessedFrame snapshot);

    /**
     * @brief 非阻塞地向 IMU recorder 提交一条运动样本。
     * @param sample 待记录 IMU 样本。
     * @return recorder 接受样本时为 true；未配置 recorder 时视为成功。
     */
    bool try_record_imu_sample(
        const domain::ImuKinematicSample& sample) noexcept;

    /** @brief 请求所有帧 sink worker 取消当前操作。 */
    void cancel() noexcept;
    /** @brief 请求 IMU recorder 停止接收新样本。 */
    void cancel_imu_recorder() noexcept;
    /** @brief 关闭各帧队列并唤醒等待 worker。 */
    void close_queue() noexcept;
    /** @brief 等待所有帧 sink worker 退出。 */
    void join() noexcept;

    /**
     * @brief 停止所有 sink 并收集首个关闭错误。
     * @return 首个错误文本；没有错误时为空。
     * @note 可执行阻塞式 flush/文件关闭；应在生命周期管理线程调用。
     */
    std::string stop_sink();
    /** @brief 汇总各帧输出 worker 丢弃的帧数。 @return 累计丢弃数。 */
    std::uint64_t dropped_count() const;
    /** @brief 获取每个独立 sink 的健康、队列和故障诊断。 @return 输出诊断快照。 */
    RuntimeOutputDiagnostics diagnostics() const;

private:
    void report_fault(std::string message) noexcept;
    std::unique_ptr<IImuRecorder> imu_recorder_;
    std::vector<std::shared_ptr<void>> frame_output_owners_;
    std::vector<std::unique_ptr<OutputFrameWorker>> frame_workers_;
    OutputDispatcherCallbacks callbacks_;
    bool sink_stopped_ = false;
};

}  // namespace mosas::runtime

#endif  // MOSAS_RUNTIME_OUTPUT_DISPATCHER_HPP
