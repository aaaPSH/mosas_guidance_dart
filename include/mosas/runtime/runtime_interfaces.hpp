#ifndef MOSAS_RUNTIME_INTERFACES_HPP
#define MOSAS_RUNTIME_INTERFACES_HPP

#include <mosas/runtime/control_command.hpp>
#include <mosas/runtime/runtime_types.hpp>
#include <mosas/vision/vision_types.hpp>

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
    std::uint64_t timeout_count = 0;
    std::uint64_t parse_error_count = 0;
    std::uint64_t backlog_event_count = 0;
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

    // 返回相机源报告的采集帧率；没有源报告时返回 0，运行时使用本地统计回退。
    virtual double capture_fps() const noexcept { return 0.0; }

    // 返回相机源内部缓存丢弃的帧数；通用或模拟数据源默认不提供丢帧。
    virtual std::uint64_t dropped_frame_count() const noexcept { return 0; }

    // 在处理线程完成颜色转换、缩放和去畸变等帧预处理。
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

    // This interface is for continuous control setpoints only. Reliable
    // one-shot events need a separate bounded event channel.
    virtual bool send(const ControlCommand& command) = 0;

    // The runtime uses this only to distinguish an unavailable device from a
    // generic send failure. The default is appropriate for non-device sinks.
    virtual bool device_connected() const noexcept { return true; }

    virtual std::string last_error() const { return {}; }

    // The lower-level protocol must define safe behavior. The current runtime
    // intentionally does not assume that a zero command is safe or send one.
    enum class SafeCommandPolicy {
        not_defined,
        sink_defined,
    };

    virtual SafeCommandPolicy safe_command_policy() const noexcept {
        return SafeCommandPolicy::not_defined;
    }

    virtual bool send_safe_command() { return false; }
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
