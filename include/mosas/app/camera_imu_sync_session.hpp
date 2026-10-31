#ifndef MOSAS_APP_CAMERA_IMU_SYNC_SESSION_HPP
#define MOSAS_APP_CAMERA_IMU_SYNC_SESSION_HPP

#include <mosas/app/app_config.hpp>
#include <mosas/runtime/timing_distribution.hpp>

#include <cstdint>
#include <memory>

namespace mosas::app {

/** @brief 采集结束原因；只有时长正常届满才可能生成完整数据。 */
enum class CameraImuSyncEnd { duration_elapsed, interrupted, runtime_fault, startup_failure };

/** @brief 固定大小的记录错误码，可由传感器消费线程安全地报告。 */
enum class CameraImuSyncError {
    none, invalid_camera, invalid_imu, image_queue_full, imu_queue_full,
    budget_exceeded, write_failure, allocation_failure, insufficient_data,
};

/** @brief 停机后收集的数据源和运行时诊断，不持有设备对象。 */
struct CameraImuSyncSummary {
    runtime::RuntimeTiming timing;
    runtime::RuntimeStatistics fps;
    runtime::ImuSourceDiagnostics imu;
    runtime::TimingDistributionSummary camera_callback_wait;
    runtime::TimingDistributionSummary camera_queue_delay;
    runtime::TimingDistributionSummary camera_decode;
    std::uint64_t camera_queue_dropped = 0;
    std::uint64_t camera_timestamp_dropped = 0;
    std::uint32_t sdk_width = 0;
    std::uint32_t sdk_height = 0;
    double sdk_fps = 0;
    std::uint32_t exposure_us = 0;
    std::uint32_t gain = 0;
};

/**
 * @brief 在打开设备之前校验采集设置及预计存储空间。
 * @param options 新输出目录、时长、抽样率和记录上限。
 * @param config 实际运行配置，保留其中的时间补偿。
 * @throws std::invalid_argument 模式不兼容、范围无效或目录已经存在时抛出。
 * @throws std::filesystem::filesystem_error 无法检查目录状态时抛出。
 */
void validate_camera_imu_sync_options(const CameraImuSyncOptions& options,
                                     const AppConfig& config);

/** @brief 在采集链之外转换错误文本。 @param error 首次错误。 @return 静态诊断名称。 */
const char* camera_imu_sync_error_name(CameraImuSyncError error) noexcept;

/**
 * @brief 在正常运行时旁路记录有界的原始图像及 IMU 数据。
 * @note 每种提交方法仅允许一个生产者；finish() 必须在两个生产者均退出后调用。
 *       被选中的图像在 prepare() 修改像素之前独立深拷贝。
 */
class CameraImuSyncSession final {
public:
    /** @brief 创建并校验记录器。 @param options 记录上限。 @param config 未改写的实际运行设置。 */
    CameraImuSyncSession(CameraImuSyncOptions options, const AppConfig& config);
    /** @brief 排空并关闭未结束的会话，将其标记为不完整；不抛出异常。 */
    ~CameraImuSyncSession();
    CameraImuSyncSession(const CameraImuSyncSession&) = delete;
    CameraImuSyncSession& operator=(const CameraImuSyncSession&) = delete;

    /** @brief 新建目录、不完整清单及记录线程。 @throws std::exception 启动失败时抛出。 */
    void start();
    /**
     * @brief 抽样并独立保留尚未预处理的 SDK 图像，不执行磁盘 I/O。
     * @param frame 原始 BGR8 像素及应用补偿后的运行时间戳。
     * @param observed_ns capture() 返回时的单调时间，单位 ns，同时用于抽样。
     * @return 已接收或按抽样规则跳过时为 true；记录失败或未处于接收状态时为 false。
     */
    bool record_camera(const runtime::CameraFrame& frame, runtime::TimestampNs observed_ns) noexcept;
    /**
     * @brief 保留全速原始解码 IMU 样本，不改变正常数据流。
     * @param sample 估计器修正前的角速度及加速度，单位分别为 rad/s 和 m/s²。
     * @param observed_ns 消费线程观测到样本时的单调时间，单位 ns；不是传感器采样时刻。
     * @return 成功入队时为 true；数据无效、队列溢出或记录已停止时为 false。
     */
    bool record_imu(const runtime::ImuAngularVelocitySample& sample,
                    runtime::TimestampNs observed_ns) noexcept;
    /** @brief 线程安全地查询健康状态。 @return 首次记录错误，无错误时为 none。 */
    CameraImuSyncError error() const noexcept;
    /**
     * @brief 在运行时停止后排空并关闭数据文件，原子替换最终清单。
     * @param end 停止原因；中断或故障均不能将采集标记为完整。
     * @param summary 所有生产者退出后收集的最终数据源及运行时诊断。
     * @return 仅在正常完成且数据及诊断均健康时返回 true。
     * @note 重复调用返回首次完成结果；清单写入失败时保留初始的不完整标记。
     */
    bool finish(CameraImuSyncEnd end, const CameraImuSyncSummary& summary) noexcept;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace mosas::app
#endif
