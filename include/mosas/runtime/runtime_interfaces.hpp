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

class ImuSource {
public:
    virtual ~ImuSource() = default;

    // try_pop 用于非阻塞取样；没有样本时，调用方等待数据源就绪。
    virtual SourceResult start() = 0;
    virtual bool try_pop(ImuAngularVelocitySample* sample) noexcept = 0;
    virtual bool wait_for_data() noexcept = 0;
    // stop() 完成资源和线程清理；cancel() 仅非阻塞地发出取消并唤醒等待。
    virtual void stop() noexcept = 0;
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
        if (frame == nullptr || !frame->image.is_valid()) {
            return {SourceStatus::fatal, "camera frame is invalid"};
        }
        return {SourceStatus::ok, {}};
    }

    // 非阻塞地请求取消并唤醒阻塞操作；资源由 stop() 最终释放。
    virtual void stop() noexcept {}
    virtual void cancel() noexcept = 0;
};

class CommandSink {
public:
    virtual ~CommandSink() = default;

    virtual bool start() = 0;

    // 只提交最新控制量；实现不得在调用线程执行串口/文件写入。
    virtual bool try_submit(const GuidanceCommand& command) noexcept = 0;

    // 起飞周期切换后，异步 sink 应丢弃尚未发送的旧周期命令。
    virtual void set_active_generation(std::uint64_t generation) noexcept {
        (void)generation;
    }

    // 非阻塞地请求取消，串口/文件等清理由 stop() 完成。
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

}  // namespace mosas::runtime

#endif  // MOSAS_RUNTIME_INTERFACES_HPP
