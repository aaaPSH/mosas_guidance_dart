#ifndef MOSAS_RUNTIME_FRAME_PIPELINE_HPP
#define MOSAS_RUNTIME_FRAME_PIPELINE_HPP

#include <mosas/runtime/frame_rate_meter.hpp>
#include <mosas/runtime/latest_frame_queue.hpp>
#include <mosas/runtime/runtime_interfaces.hpp>
#include <mosas/runtime/runtime_supervisor.hpp>
#include <mosas/runtime/timing_distribution.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace mosas::runtime {

class ImuPipeline;
class LaunchGenerationCoordinator;
class OutputDispatcher;

struct FramePipelineCallbacks {
    std::function<bool()> stop_requested;
    std::function<bool()> faulted;
    std::function<void(std::string)> report_fault;
    std::function<void(const std::string&)> log;
};

/// 相机帧流水线：采集线程入有界队列，单一处理线程串行调用有状态算法。
class FramePipeline final {
  public:
    /**
     * @brief 创建相机帧采集与制导处理流水线。
     * @param camera_source 相机源；本对象接管其所有权。
     * @param vision_detector 单线程调用的视觉识别器。
     * @param los_estimator 视觉观测到镖体系视线估计器。
     * @param los_rate_filter 有状态视线速率滤波器。
     * @param guidance_law 制导律实现。
     * @param config 帧龄限制、队列容量和相机内参/外参配置。
     * @param imu_pipeline 提供按相机时间查询的 IMU/姿态快照。
     * @param command_sink 非阻塞控制命令端口。
     * @param generation_coordinator 协调命令与起飞状态代际。
     * @param output_dispatcher 接收处理后的不可变帧快照。
     * @param callbacks 运行停止、故障和日志回调。
     */
    FramePipeline(std::unique_ptr<CameraSource> camera_source,
                  std::unique_ptr<ports::IVisionDetector> vision_detector,
                  std::unique_ptr<ports::ILosEstimator> los_estimator,
                  std::unique_ptr<ports::ILosRateFilter> los_rate_filter,
                  std::unique_ptr<ports::IGuidanceLaw> guidance_law,
                  RuntimeSupervisorConfig config, ImuPipeline& imu_pipeline,
                  CommandSink& command_sink,
                  LaunchGenerationCoordinator& generation_coordinator,
                  OutputDispatcher& output_dispatcher,
                  FramePipelineCallbacks callbacks);
    ~FramePipeline();

    FramePipeline(const FramePipeline&) = delete;
    FramePipeline& operator=(const FramePipeline&) = delete;

    /** @brief 检查源、算法依赖和队列是否完整。 @return 可运行时为 true。 */
    bool valid() const noexcept;
    /** @brief 配置底层相机源。 @return 配置结果及可选错误信息。 */
    SourceResult configure_source();
    /** @brief 清空采集队列并重置视觉和 LOS 滤波器的跨帧状态。 */
    void reset() noexcept;
    /** @brief 创建相机采集与帧处理工作线程。 */
    void start_workers();
    /** @brief 请求相机源取消并唤醒阻塞捕获。 */
    void cancel_source() noexcept;
    /** @brief 等待相机采集线程退出。 */
    void join_capture_worker() noexcept;
    /** @brief 在线程退出后释放底层相机源。 */
    void stop_source() noexcept;
    /** @brief 关闭帧队列，通知处理线程不再有新帧。 */
    void close_queue() noexcept;
    /** @brief 等待串行帧处理线程退出。 */
    void join_processing_worker() noexcept;

    /** @brief 返回本地统计的处理帧率。 @return 当前处理 FPS。 */
    double processing_fps() const;
    /** @brief 获取本流水线采集和处理帧率。 @return 当前运行统计快照；输出 FPS 由 supervisor 汇总。 */
    RuntimeStatistics statistics() const;
    /** @brief 获取本流水线采集和处理阶段时序及采集队列丢帧。 @return 当前时序快照。 */
    RuntimeTiming timing() const;

  private:
    struct QueuedCameraFrame {
        CameraFrame frame;
        std::chrono::steady_clock::time_point enqueued_at{};
    };

    enum class TimingStage {
        capture,
        capture_queue_wait,
        capture_queue_delay,
        prepare,
        vision,
        line_of_sight,
        guidance,
        command,
        processing_total,
    };

    struct TimingAccumulator {
        TimingDistribution distribution;
    };

    /**
     * @brief 从相机获取帧、验证时间戳并送入有界采集队列。
     * @note 运行于采集线程；捕获故障通过 callbacks_ 上报。
     */
    void capture_worker() noexcept;

    /**
     * @brief 串行执行帧预处理、识别、LOS/滤波/制导并提交命令和输出快照。
     * @note 有状态视觉和制导算法只在此线程调用；慢速输出由 OutputDispatcher 异步消费。
     */
    void processing_worker() noexcept;
    void record_timing(TimingStage stage,
                       std::chrono::steady_clock::duration duration) noexcept;
    void report_fault(std::string message) noexcept;
    void log(const std::string& message) const;

    std::unique_ptr<CameraSource> camera_source_;
    std::unique_ptr<ports::IVisionDetector> vision_detector_;
    std::unique_ptr<ports::ILosEstimator> los_estimator_;
    std::unique_ptr<ports::ILosRateFilter> los_rate_filter_;
    std::unique_ptr<ports::IGuidanceLaw> guidance_law_;
    RuntimeSupervisorConfig config_;
    ImuPipeline& imu_pipeline_;
    CommandSink& command_sink_;
    LaunchGenerationCoordinator& generation_coordinator_;
    OutputDispatcher& output_dispatcher_;
    FramePipelineCallbacks callbacks_;
    LatestFrameQueue<QueuedCameraFrame> capture_queue_;
    std::thread capture_thread_;
    std::thread processing_thread_;
    FrameRateMeter capture_fps_meter_;
    FrameRateMeter processing_fps_meter_;
    mutable std::mutex timing_mutex_;
    TimingAccumulator capture_timing_;
    TimingAccumulator capture_queue_wait_timing_;
    TimingAccumulator capture_queue_delay_timing_;
    TimingAccumulator prepare_timing_;
    TimingAccumulator vision_timing_;
    TimingAccumulator line_of_sight_timing_;
    TimingAccumulator guidance_timing_;
    TimingAccumulator command_timing_;
    TimingAccumulator processing_total_timing_;
};

} // namespace mosas::runtime

#endif // MOSAS_RUNTIME_FRAME_PIPELINE_HPP
