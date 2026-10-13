#ifndef MOSAS_RUNTIME_INTERFACES_HPP
#define MOSAS_RUNTIME_INTERFACES_HPP

#include <mosas/domain/algorithm_results.hpp>
#include <mosas/domain/contracts.hpp>
#include <mosas/runtime/runtime_types.hpp>
#include <mosas/runtime/imu_angular_velocity.hpp>

#include <cstddef>
#include <cstdint>
#include <string>

namespace mosas::runtime {

enum class SourceStatus {
    ok,
    timeout,
    fatal,
    cancelled,
};

struct SourceResult {
    SourceStatus status = SourceStatus::fatal;
    std::string message;
};

// IMU 数据源的可选接收诊断快照；不可提供该信息的数据源返回 available=false。
struct ImuSourceDiagnostics {
    bool available = false;
    TimestampNs latest_sample_timestamp_ns = -1;
    bool pending_bytes_valid = false;
    std::size_t pending_bytes = 0;
    std::size_t pending_frame_count = 0;
    std::size_t buffered_bytes = 0;
    std::uint64_t delivered_sample_count = 0;
    std::uint64_t parse_error_count = 0;
    std::uint64_t backlog_event_count = 0;
    std::uint64_t received_chunk_count = 0;
    std::uint64_t dropped_chunk_count = 0;
    std::uint64_t sequence_gap_count = 0;
    std::uint64_t timestamp_rollback_count = 0;
    std::uint64_t dropped_sample_count = 0;
    std::size_t queued_chunk_count = 0;
    std::size_t chunk_queue_capacity = 0;
    bool receive_realtime_priority_applied = false;
    bool receive_priority_error = false;
};

inline const char* source_status_name(SourceStatus status) noexcept {
    switch (status) {
        case SourceStatus::ok:
            return "ok";
        case SourceStatus::timeout:
            return "timeout";
        case SourceStatus::fatal:
            return "fatal";
        case SourceStatus::cancelled:
            return "cancelled";
    }
    return "unknown";
}

/// IMU 输入端口；try_pop 与 wait_for_data 由唯一的 IMU 消费线程调用。
class ImuSource {
public:
    virtual ~ImuSource() = default;

    /**
     * @brief 启动 IMU 数据源及其接收资源。
     * @return 启动状态和可选错误说明；成功时 status 为 ok。
     */
    virtual SourceResult start() = 0;

    /**
     * @brief 非阻塞地取出一个已解析 IMU 样本。
     * @param sample 非空输出指针；成功时写入样本。
     * @return 取得样本时为 true，否则为 false。
     */
    virtual bool try_pop(ImuAngularVelocitySample* sample) noexcept = 0;

    /**
     * @brief 在当前没有样本时等待数据源就绪或取消。
     * @return 数据源仍可消费时为 true；停止或取消时为 false。
     */
    virtual bool wait_for_data() noexcept = 0;

    /**
     * @brief 停止并回收数据源资源；允许等待内部接收线程退出。
     */
    virtual void stop() noexcept = 0;

    /**
     * @brief 非阻塞地请求停止，并唤醒可能阻塞在 wait_for_data 的线程。
     */
    virtual void cancel() noexcept = 0;

    /**
     * @brief 获取线程安全的接收诊断快照。
     * @return 可用时返回计数和队列状态；不支持诊断的数据源返回 available=false。
     */
    virtual ImuSourceDiagnostics diagnostics() const noexcept { return {}; }
};

enum class CameraCaptureMode {
    free_run,
    hardware_trigger,
};

struct CameraCaptureConfig {
    CameraCaptureMode mode = CameraCaptureMode::free_run;
};

/// 相机输入端口；捕获和预处理分开，以便在处理线程中执行帧转换。
class CameraSource {
public:
    virtual ~CameraSource() = default;

    /**
     * @brief 根据采集模式初始化相机源。
     * @param config 采集模式配置；设备分辨率和格式由具体适配器配置。
     * @return 配置状态和可选错误说明。
     */
    virtual SourceResult configure(const CameraCaptureConfig& config) = 0;

    /**
     * @brief 获取一帧拥有型图像及其采集/接收时间信息。
     * @param frame 非空输出指针；成功时填入帧数据。
     * @return 捕获状态；无数据、取消和致命故障由 status 区分。
     * @note 本操作可能等待下一帧，调用方必须提供可取消的生命周期。
     */
    virtual SourceResult capture(CameraFrame* frame) = 0;

    /**
     * @brief 返回相机源实测的有效帧到达速率。
     * @return 帧率；返回 0 表示适配器未提供统计，运行时改用本地测量。
     */
    virtual double capture_fps() const noexcept { return 0.0; }

    /**
     * @brief 在处理线程执行可选的颜色转换、缩放或安装方向修正。
     * @param frame 输入/输出帧；图像必须有效。
     * @return 处理状态；基础实现只验证帧有效性。
     */
    virtual SourceResult prepare(CameraFrame* frame) {
        if (frame == nullptr || !frame->image.is_valid()) {
            return {SourceStatus::fatal, "camera frame is invalid"};
        }
        return {SourceStatus::ok, {}};
    }

    /**
     * @brief 停止并最终释放相机资源；具体适配器可覆写。
     */
    virtual void stop() noexcept {}

    /**
     * @brief 非阻塞地请求取消并唤醒阻塞中的捕获；实际清理由 stop() 完成。
     */
    virtual void cancel() noexcept = 0;
};

/// 制导命令输出端口；实时路径只提交命令，实际 I/O 由 sink 自己执行。
class CommandSink {
public:
    virtual ~CommandSink() = default;

    /**
     * @brief 启动命令输出资源。
     * @return 成功启动时为 true。
     */
    virtual bool start() = 0;

    /**
     * @brief 非阻塞地提交最新制导命令。
     * @param command 带有效性、来源帧、时间戳和起飞代际的命令。
     * @return 命令已接受时为 true；队列/邮箱不可用或提交失败时为 false。
     * @note 不得在调用线程执行串口或文件写入；队列容量及替代策略由实现保证。
     */
    virtual bool try_submit(const GuidanceCommand& command) noexcept = 0;

    /**
     * @brief 通知异步 sink 当前有效的起飞代际。
     * @param generation 新代际编号；旧代际尚未发送的命令应被丢弃。
     */
    virtual void set_active_generation(std::uint64_t generation) noexcept {
        (void)generation;
    }

    /**
     * @brief 非阻塞地请求取消正在进行的发送工作。
     */
    virtual void cancel() noexcept {}

    /**
     * @brief 停止输出并回收发送资源。
     */
    virtual void stop() noexcept {}

    struct Diagnostics {
        bool available = false;
        std::uint64_t submitted_count = 0;
        std::uint64_t dropped_count = 0;
        std::uint64_t send_failure_count = 0;
    };

    /**
     * @brief 返回命令输出计数器快照。
     * @return 可用时包含提交、丢弃和发送失败计数。
     */
    virtual Diagnostics diagnostics() const noexcept { return {}; }
};

}  // namespace mosas::runtime

#endif  // MOSAS_RUNTIME_INTERFACES_HPP
