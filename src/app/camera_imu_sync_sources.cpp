#include <mosas/app/camera_imu_sync_sources.hpp>
#include <mosas/runtime/clock.hpp>

#include <atomic>
#include <stdexcept>
#include <utility>

namespace mosas::app {
namespace {

/** @brief 保留实际相机路径，在 prepare() 原地修改像素前复制被选中的图像。 */
class RecordingCamera final : public runtime::CameraSource {
public:
    RecordingCamera(std::unique_ptr<runtime::CameraSource> source,
                    std::shared_ptr<CameraImuSyncSession> session)
        : source_(std::move(source)), session_(std::move(session)) {}
    runtime::SourceResult configure(const runtime::CameraCaptureConfig& config) override {
        return source_->configure(config);
    }
    runtime::SourceResult capture(runtime::CameraFrame* frame) override {
        auto result = source_->capture(frame);
        if (result.status == runtime::SourceStatus::ok && frame != nullptr)
            session_->record_camera(*frame, runtime::monotonic_timestamp_ns());
        return result;
    }
    runtime::SourceResult prepare(runtime::CameraFrame* frame) override { return source_->prepare(frame); }
    double capture_fps() const noexcept override { return source_->capture_fps(); }
    void cancel() noexcept override { source_->cancel(); }
    void stop() noexcept override { source_->stop(); }
private:
    std::unique_ptr<runtime::CameraSource> source_;
    std::shared_ptr<CameraImuSyncSession> session_;
};

/** @brief 在原 IMU 消费线程观察原始解码样本，不增加第二个串口读取者。 */
class RecordingImu final : public runtime::ImuSource {
public:
    RecordingImu(std::unique_ptr<runtime::ImuSource> source,
                 std::shared_ptr<CameraImuSyncSession> session)
        : source_(std::move(source)), session_(std::move(session)) {}
    runtime::SourceResult start() override { return source_->start(); }
    bool try_pop(runtime::ImuAngularVelocitySample* sample) noexcept override {
        const bool available = source_->try_pop(sample);
        if (available && sample != nullptr) session_->record_imu(*sample, runtime::monotonic_timestamp_ns());
        return available;
    }
    bool wait_for_data() noexcept override { return source_->wait_for_data(); }
    void cancel() noexcept override { source_->cancel(); }
    void stop() noexcept override { source_->stop(); }
    runtime::ImuSourceDiagnostics diagnostics() const noexcept override { return source_->diagnostics(); }
private:
    std::unique_ptr<runtime::ImuSource> source_;
    std::shared_ptr<CameraImuSyncSession> session_;
};

/** @brief 只在内存中接收运行时命令，不持有任何设备或发送资源。 */
class ReceiveOnlyCommandSink final : public runtime::CommandSink {
public:
    bool start() override {
        submitted_.store(0);
        dropped_.store(0);
        accepting_.store(true);
        return true;
    }
    bool try_submit(const runtime::GuidanceCommand&) noexcept override {
        if (!accepting_.load()) { dropped_.fetch_add(1); return false; }
        submitted_.fetch_add(1);
        return true;
    }
    void cancel() noexcept override { accepting_.store(false); }
    void stop() noexcept override { cancel(); }
    Diagnostics diagnostics() const noexcept override {
        Diagnostics result;
        result.available = true;
        result.submitted_count = submitted_.load();
        result.dropped_count = dropped_.load();
        return result;
    }
private:
    std::atomic<bool> accepting_{false};
    std::atomic<std::uint64_t> submitted_{0}, dropped_{0};
};
}  // namespace

CameraImuSyncPorts make_camera_imu_sync_ports(
    std::unique_ptr<runtime::CameraSource> camera,
    std::unique_ptr<runtime::ImuSource> imu,
    std::shared_ptr<CameraImuSyncSession> session) {
    if (!camera || !imu || !session) throw std::invalid_argument("标定输入源和会话不能为空");
    CameraImuSyncPorts ports;
    ports.camera = std::make_unique<RecordingCamera>(std::move(camera), session);
    ports.imu = std::make_unique<RecordingImu>(std::move(imu), std::move(session));
    ports.command = std::make_unique<ReceiveOnlyCommandSink>();
    return ports;
}

}  // namespace mosas::app
