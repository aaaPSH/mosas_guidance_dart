#ifndef MOSAS_RUNTIME_IMU_PIPELINE_HPP
#define MOSAS_RUNTIME_IMU_PIPELINE_HPP

#include <mosas/ports/flight_state_estimator.hpp>
#include <mosas/runtime/attitude_history.hpp>
#include <mosas/runtime/imu_angular_velocity_history.hpp>
#include <mosas/runtime/runtime_interfaces.hpp>

#include <cstddef>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

namespace mosas::runtime {

class LaunchGenerationCoordinator;

struct ImuPipelineCallbacks {
    std::function<bool()> stop_requested;
    std::function<bool(const domain::ImuKinematicSample&)> try_record_sample;
    std::function<void(std::string)> report_fault;
};

/// IMU 数据流水线：独占样本消费、飞行状态更新和时间历史写入。
class ImuPipeline final {
public:
    /**
     * @brief 创建 IMU 接收、状态估计和历史缓存流水线。
     * @param source IMU 数据源；本对象接管其所有权。
     * @param estimator 单次消费样本后同时生成姿态与起飞状态快照的估计器。
     * @param history_capacity 姿态和角速度历史容量。
     * @param generation_coordinator 运行代际协调器；切代时触发估计器 reset。
     * @param callbacks 停止检测、IMU 记录提交及故障报告回调。
     */
    ImuPipeline(std::unique_ptr<ImuSource> source,
                std::unique_ptr<ports::IFlightStateEstimator> estimator,
                std::size_t history_capacity,
                LaunchGenerationCoordinator& generation_coordinator,
                ImuPipelineCallbacks callbacks);
    ~ImuPipeline();

    ImuPipeline(const ImuPipeline&) = delete;
    ImuPipeline& operator=(const ImuPipeline&) = delete;

    /** @brief 检查数据源和状态估计器是否可用。 @return 可运行时为 true。 */
    bool valid() const noexcept;
    /** @brief 启动底层 IMU 数据源。 @return 数据源启动状态。 */
    SourceResult start_source();
    /** @brief 清空历史并重置估计器，为新的运行代际作准备。 */
    void reset() noexcept;
    /** @brief 启动唯一的 IMU 样本消费/状态估计线程。 */
    void start_worker();
    /** @brief 请求停止 IMU 接收并唤醒样本等待。 */
    void cancel_source() noexcept;
    /** @brief 等待 IMU 状态估计线程退出。 */
    void join_worker() noexcept;
    /** @brief 在线程退出后停止并释放底层 IMU 源。 */
    void stop_source() noexcept;

    /**
     * @brief 获取最近一次 IMU 更新形成的完整飞行状态。
     * @return 至少处理过一个样本时返回快照，否则为空。
     */
    std::optional<ImuStateSnapshot> latest_state() const;

    /**
     * @brief 按单调时间查询角速度历史并在有效样本间插值。
     * @param timestamp_ns 目标相机帧的单调时间戳（ns）。
     * @return 可匹配时返回角速度及时间质量信息，否则为空。
     */
    std::optional<ImuAngularVelocityLookup> angular_velocity_at(
        TimestampNs timestamp_ns) const noexcept;
    /**
     * @brief 按单调时间查询姿态历史。
     * @param timestamp_ns 目标相机帧的单调时间戳（ns）。
     * @return 可匹配时返回插值姿态和起飞代际，否则为空。
     */
    std::optional<AttitudeLookup> attitude_at(
        TimestampNs timestamp_ns) const noexcept;
    /** @brief 返回当前角速度历史中可查询的样本数。 @return 历史样本数。 */
    std::size_t angular_velocity_history_size() const noexcept;
    /** @brief 返回底层 IMU 源的诊断快照。 @return 接收和解码诊断。 */
    ImuSourceDiagnostics diagnostics() const noexcept;

private:
    /**
     * @brief 消费样本、更新状态估计并写入供相机帧查询的有界历史。
     * @note 仅运行在 IMU worker；无效样本、非递增时间戳和 IMU 记录失败会触发运行时故障。
     */
    void worker_loop() noexcept;
    void report_fault(std::string message) noexcept;

    std::unique_ptr<ImuSource> source_;
    std::unique_ptr<ports::IFlightStateEstimator> estimator_;
    LaunchGenerationCoordinator& generation_coordinator_;
    ImuPipelineCallbacks callbacks_;
    ImuAngularVelocityHistory angular_velocity_history_;
    AttitudeHistory attitude_history_;
    mutable std::mutex state_mutex_;
    std::optional<ImuStateSnapshot> latest_state_;
    std::thread worker_;
};

}  // namespace mosas::runtime

#endif  // MOSAS_RUNTIME_IMU_PIPELINE_HPP
