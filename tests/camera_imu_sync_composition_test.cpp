#include <mosas/app/camera_imu_sync_sources.hpp>
#include <mosas/runtime/clock.hpp>
#include <mosas/vision/opencv_image_frame.hpp>
#include "test_support.hpp"

#include <opencv2/imgproc.hpp>

#include <chrono>
#include <filesystem>
#include <memory>

namespace {

struct Calls { int start = 0, read = 0, prepare = 0, wait = 0, stop = 0, cancel = 0; };

class Camera final : public mosas::runtime::CameraSource {
public:
    Calls calls;
    mosas::runtime::SourceResult configure(const mosas::runtime::CameraCaptureConfig&) override {
        ++calls.start; return {mosas::runtime::SourceStatus::ok, {}};
    }
    mosas::runtime::SourceResult capture(mosas::runtime::CameraFrame* frame) override {
        ++calls.read;
        frame->timestamp_ns = mosas::runtime::monotonic_timestamp_ns() - 10'000'000 + calls.read;
        frame->received_timestamp_ns = frame->timestamp_ns + 1'000'000;
        frame->timestamp_source = mosas::runtime::CameraTimestampSource::sdk_monotonic;
        cv::Mat image(2, 3, CV_8UC3, cv::Scalar(42, 42, 42));
        mosas::vision::assign_bgr8_mat(&frame->image, image);
        return {mosas::runtime::SourceStatus::ok, {}};
    }
    mosas::runtime::SourceResult prepare(mosas::runtime::CameraFrame* frame) override {
        ++calls.prepare;
        auto view = mosas::vision::make_bgr8_mat_view(frame->image);
        cv::flip(view, view, -1);
        return {mosas::runtime::SourceStatus::ok, {}};
    }
    double capture_fps() const noexcept override { return 77; }
    void stop() noexcept override { ++calls.stop; }
    void cancel() noexcept override { ++calls.cancel; }
};

class Imu final : public mosas::runtime::ImuSource {
public:
    Calls calls;
    mosas::runtime::SourceResult start() override { ++calls.start; return {mosas::runtime::SourceStatus::ok, {}}; }
    bool try_pop(mosas::runtime::ImuAngularVelocitySample* sample) noexcept override {
        ++calls.read;
        sample->timestamp_ns = mosas::runtime::monotonic_timestamp_ns() - 20'000'000 + calls.read;
        sample->angular_velocity = {0.1, -0.2, 0.3};
        return true;
    }
    bool wait_for_data() noexcept override { ++calls.wait; return true; }
    void stop() noexcept override { ++calls.stop; }
    void cancel() noexcept override { ++calls.cancel; }
    mosas::runtime::ImuSourceDiagnostics diagnostics() const noexcept override {
        mosas::runtime::ImuSourceDiagnostics result;
        result.available = true;
        result.received_chunk_count = 123;
        return result;
    }
};
}  // namespace

/** @brief 验证单一数据源转发及不依赖串口的命令路径。 @return 通过时返回 0。 */
int main() {
    using namespace mosas;
    const auto directory = std::filesystem::temp_directory_path() /
        ("mosas-sync-ports-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    struct Cleanup {
        std::filesystem::path path;
        ~Cleanup() { std::error_code error; std::filesystem::remove_all(path, error); }
    } cleanup{directory};
    app::AppConfig config;
    config.camera.width = 3; config.camera.height = 2; config.camera.fps = 120;
    config.camera.auto_exposure = false;
    config.camera.frame_timestamp_domain = runtime::CameraTimestampDomain::monotonic;
    config.imu.mode = app::ImuMode::serial;
    app::CameraImuSyncOptions options;
    options.directory = directory;
    auto session = std::make_shared<app::CameraImuSyncSession>(options, config);
    session->start();
    auto camera = std::make_unique<Camera>();
    auto imu = std::make_unique<Imu>();
    auto* camera_status = camera.get();
    auto* imu_status = imu.get();
    auto ports = app::make_camera_imu_sync_ports(std::move(camera), std::move(imu), session);
    CHECK(ports.camera->configure({}).status == runtime::SourceStatus::ok);
    CHECK(ports.imu->start().status == runtime::SourceStatus::ok);
    runtime::CameraFrame frame;
    CHECK(ports.camera->capture(&frame).status == runtime::SourceStatus::ok);
    CHECK(frame.image.width == 3 && frame.image.height == 2);
    CHECK(ports.camera->prepare(&frame).status == runtime::SourceStatus::ok);
    CHECK(ports.camera->capture_fps() == 77);
    runtime::ImuAngularVelocitySample sample;
    CHECK(ports.imu->try_pop(&sample));
    CHECK_NEAR(sample.angular_velocity.y, -0.2, 1e-15);
    CHECK(ports.imu->wait_for_data());
    CHECK(ports.imu->diagnostics().received_chunk_count == 123);
    CHECK(!ports.command->try_submit({}));
    CHECK(ports.command->start());
    ports.command->set_active_generation(5);
    CHECK(ports.command->try_submit({}));
    CHECK(ports.command->diagnostics().submitted_count == 1);
    // 此接收端在不依赖 SDK 的 app_core 中构建，不持有串口或发送器。
    ports.command->cancel();
    CHECK(!ports.command->try_submit({}));
    ports.command->stop();
    ports.camera->cancel(); ports.imu->cancel();
    ports.camera->stop(); ports.imu->stop();
    CHECK(camera_status->calls.start == 1 && camera_status->calls.read == 1);
    CHECK(camera_status->calls.prepare == 1 && camera_status->calls.cancel == 1 && camera_status->calls.stop == 1);
    CHECK(imu_status->calls.start == 1 && imu_status->calls.read == 1 && imu_status->calls.wait == 1);
    CHECK(imu_status->calls.cancel == 1 && imu_status->calls.stop == 1);
    CHECK(session->error() == app::CameraImuSyncError::none);
    CHECK(!session->finish(app::CameraImuSyncEnd::interrupted, {}));
    return 0;
}
