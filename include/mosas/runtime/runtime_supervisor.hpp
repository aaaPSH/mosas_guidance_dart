#ifndef MOSAS_RUNTIME_RUNTIME_SUPERVISOR_HPP
#define MOSAS_RUNTIME_RUNTIME_SUPERVISOR_HPP

#include <mosas/domain/algorithm_results.hpp>
#include <mosas/ports/clock.hpp>
#include <mosas/ports/flight_state_estimator.hpp>
#include <mosas/ports/guidance_law.hpp>
#include <mosas/ports/los_estimator.hpp>
#include <mosas/ports/los_rate_filter.hpp>
#include <mosas/ports/vision_detector.hpp>
#include <mosas/runtime/output_ports.hpp>
#include <mosas/runtime/runtime_interfaces.hpp>
#include <mosas/runtime/runtime_types.hpp>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

namespace mosas::runtime {

struct RuntimeStatistics {
    double capture_fps = 0.0;
    double processing_fps = 0.0;
    double output_fps = 0.0;
};

struct RuntimeTimingStage {
    std::uint64_t samples = 0;
    double minimum_ms = 0.0;
    double average_ms = 0.0;
    double p50_ms = 0.0;
    double p95_ms = 0.0;
    double p99_ms = 0.0;
    double maximum_ms = 0.0;
};

struct RuntimeTiming {
    RuntimeTimingStage capture;
    RuntimeTimingStage capture_queue_wait;
    RuntimeTimingStage capture_queue_delay;
    RuntimeTimingStage prepare;
    RuntimeTimingStage vision;
    RuntimeTimingStage line_of_sight;
    RuntimeTimingStage guidance;
    RuntimeTimingStage command;
    RuntimeTimingStage processing_total;
    RuntimeTimingStage output_queue_wait;
    RuntimeTimingStage output_queue_delay;
    RuntimeTimingStage output;
    std::uint64_t capture_queue_dropped = 0;
    std::uint64_t output_queue_dropped = 0;
};

struct RuntimeSupervisorConfig {
    double launch_speed_mps = 100.0;
    std::size_t history_capacity = 256;
    TimestampNs max_imu_age_ns = 50000000;
    TimestampNs max_camera_future_ns = 50000000;
    TimestampNs max_camera_age_ns = 2000000000;
    domain::CameraIntrinsics camera_intrinsics{1.0, 1.0, 0.0, 0.0};
    domain::Rotation3<domain::DartFrameTag, domain::CameraFrameTag>
        dart_from_camera{};
    CameraCaptureConfig camera_capture{};
    std::size_t capture_queue_capacity = 1;
    std::size_t output_queue_capacity = 1;
    std::size_t info_log_queue_capacity = 256;
};

struct RuntimeSupervisorDependencies {
    std::unique_ptr<ports::IVisionDetector> vision_detector;
    std::unique_ptr<ports::IFlightStateEstimator> flight_state_estimator;
    std::unique_ptr<ports::ILosEstimator> los_estimator;
    std::unique_ptr<ports::ILosRateFilter> los_rate_filter;
    std::unique_ptr<ports::IGuidanceLaw> guidance_law;
    std::unique_ptr<ports::IMonotonicClock> monotonic_clock;
};

/// 单进程制导运行时的生命周期监督器和组件组合边界。
class RuntimeSupervisor {
public:
    /**
     * @brief 创建运行时并接管各数据源、算法、命令和输出组件的所有权。
     * @param imu_source IMU 数据源。
     * @param camera_source 相机数据源。
     * @param command_sink 控制命令输出端口。
     * @param config 队列、时间有效性和相机参数配置。
     * @param dependencies 视觉、姿态/起飞、LOS、滤波、制导算法及单调时钟实现。
     * @param output_ports 各独立录像、遥测、可视化、IMU 记录和日志 sink。
     */
    RuntimeSupervisor(std::unique_ptr<ImuSource> imu_source,
                      std::unique_ptr<CameraSource> camera_source,
                      std::unique_ptr<CommandSink> command_sink,
                      const RuntimeSupervisorConfig& config,
                      RuntimeSupervisorDependencies dependencies,
                      RuntimeOutputPorts output_ports);

    /**
     * @brief 销毁运行时前执行 stop()，等待工作线程退出并回收组件。
     */
    ~RuntimeSupervisor();

    RuntimeSupervisor(const RuntimeSupervisor&) = delete;
    RuntimeSupervisor& operator=(const RuntimeSupervisor&) = delete;

    /**
     * @brief 按依赖顺序启动日志、相机/IMU、命令和各处理 worker。
     * @return 全部必需组件启动成功并进入运行态时为 true；失败原因可由 last_error() 获取。
     * @note 可阻塞；不得从实时处理 worker 调用。
     */
    bool start();

    /**
     * @brief 请求取消运行并唤醒阻塞中的源/worker，不等待线程退出。
     */
    void cancel() noexcept;

    /**
     * @brief 完成取消、线程 join、sink flush 和资源回收。
     * @note 可阻塞；可由 supervisor 所在线程调用。
     */
    void stop() noexcept;

    /** @brief 查询运行时是否处于运行状态。 @return 运行中时为 true。 */
    bool running() const noexcept;
    /** @brief 查询是否发生运行时故障。 @return 已记录故障时为 true。 */
    bool faulted() const noexcept;
    /** @brief 获取最近一次运行错误。 @return 错误文本；尚无错误时为空。 */
    std::string last_error() const;
    /** @brief 获取最近的 IMU 姿态/起飞状态快照。 @return 有样本时返回快照，否则为空。 */
    std::optional<ImuStateSnapshot> latest_state() const;
    /** @brief 获取相机采集、处理和输出帧率统计。 @return 当前统计快照。 */
    RuntimeStatistics statistics() const;
    /** @brief 获取各运行阶段延迟分布及队列丢帧计数。 @return 当前时序快照。 */
    RuntimeTiming timing() const;
    /** @brief 获取各独立输出 sink 的状态和丢弃/故障诊断。 @return 输出诊断快照。 */
    RuntimeOutputDiagnostics output_diagnostics() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace mosas::runtime

#endif  // MOSAS_RUNTIME_RUNTIME_SUPERVISOR_HPP
