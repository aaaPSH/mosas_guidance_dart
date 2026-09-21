#ifndef MOSAS_RUNTIME_INTERFACES_HPP
#define MOSAS_RUNTIME_INTERFACES_HPP

#include <mosas/runtime/runtime_types.hpp>
#include <mosas/runtime/imu_angular_velocity.hpp>
#include <mosas/vision/vision_types.hpp>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <thread>

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
    std::uint64_t timeout_count = 0;
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

class ImuSource {
public:
    virtual ~ImuSource() = default;

    // try_pop 用于非阻塞取样；没有样本时，调用方可等待数据源就绪。
    virtual SourceResult start() { return {SourceStatus::ok, {}}; }
    virtual bool try_pop(ImuAngularVelocitySample* sample) noexcept {
        (void)sample;
        return false;
    }
    // 无数据时等待数据源通知；未实现通知的旧数据源使用短暂退避。
    virtual bool wait_for_data() noexcept {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        return true;
    }
    virtual void stop() noexcept {}

    // 保留旧接口以兼容现有外部数据源；运行时主路径使用上面的异步接口。
    virtual SourceResult read(ImuSample* sample) = 0;
    virtual void cancel() noexcept = 0;

    // 返回线程安全的接收诊断；通用或模拟数据源默认不提供该信息。
    virtual ImuSourceDiagnostics diagnostics() const noexcept { return {}; }
};

enum class CameraCaptureMode {
    free_run,
    hardware_trigger,
};

struct CameraCaptureConfig {
    CameraCaptureMode mode = CameraCaptureMode::free_run;
};

class CameraSource {
public:
    virtual ~CameraSource() = default;

    virtual SourceResult configure(const CameraCaptureConfig& config) = 0;
    virtual SourceResult capture(CameraFrame* frame) = 0;

    // 返回相机源实测的有效帧到达速率；返回 0 时运行时使用本地统计回退。
    virtual double capture_fps() const noexcept { return 0.0; }

    // 在处理线程完成颜色转换、缩放和相机安装方向修正等帧预处理。
    virtual SourceResult prepare(CameraFrame* frame) {
        if (frame == nullptr || frame->image.empty() ||
            frame->image.type() != CV_8UC3) {
            return {SourceStatus::fatal, "camera frame is invalid"};
        }
        return {SourceStatus::ok, {}};
    }

    // 仅请求取消并唤醒阻塞操作；source 资源由 stop() 最终释放。
    virtual void stop() noexcept {}

    virtual void cancel() noexcept = 0;
};

class CommandSink {
public:
    virtual ~CommandSink() = default;

    virtual bool start() { return true; }

    // 只提交最新控制量；实现不得在调用线程执行串口/文件写入。
    virtual bool try_submit(const GuidanceCommand& command) noexcept {
        return send(command);
    }

    // 兼容旧的同步数据源；新的运行时路径不直接调用该接口。
    virtual bool send(const GuidanceCommand& command) {
        (void)command;
        return false;
    }

    // 起飞周期切换后，异步 sink 应丢弃尚未发送的旧周期命令。
    virtual void set_active_generation(std::uint64_t generation) noexcept {
        (void)generation;
    }

    virtual void cancel() noexcept {}
    virtual void stop() noexcept {}

    struct Diagnostics {
        bool available = false;
        std::uint64_t submitted_count = 0;
        std::uint64_t dropped_count = 0;
        std::uint64_t send_failure_count = 0;
    };

    virtual Diagnostics diagnostics() const noexcept { return {}; }
};

class FrameSink {
public:
    virtual ~FrameSink() = default;

    // 请求中断可能阻塞的发布操作；实现必须是线程安全且非阻塞的。
    virtual void cancel() noexcept {}

    // 新一轮运行开始前清除上一次停止状态。
    virtual void reset() noexcept {}

    // 输出线程退出时释放网络、录像等资源。
    virtual void stop() noexcept {}

    // 输出模块完成一段起飞窗口落盘后，请求运行时切换起飞周期。
    virtual bool take_rearm_request() noexcept { return false; }

    // 停止和刷新输出时报告最后一次录像/推流错误。
    virtual std::string last_error() const { return {}; }

    virtual bool publish(const CameraFrame& frame,
                         const VisionResult& vision_result,
                         const ImuStateSnapshot& imu_state,
                         const PngGuidanceOutput& guidance) = 0;

    // 携带视线角和视线角速度，供可视化接收；旧接口保持兼容。
    virtual bool publish(const CameraFrame& frame,
                         const VisionResult& vision_result,
                         const ImuStateSnapshot& imu_state,
                         const PngGuidanceOutput& guidance,
                         const VisionOverlayData& overlay) {
        (void)overlay;
        return publish(frame, vision_result, imu_state, guidance);
    }
};

}  // namespace mosas::runtime

#endif  // MOSAS_RUNTIME_INTERFACES_HPP
