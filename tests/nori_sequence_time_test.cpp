#include <mosas/adapters/nori_camera_source.hpp>
#include <mosas/app/camera_imu_sync_session.hpp>
#include "test_support.hpp"
#include <array>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <linux/v4l2-controls.h>
#include <limits>
#include <thread>

namespace {
/** @brief Deterministic SDK fake; never calls a device API. */
class CameraSdk final : public mosas::app::NoriSdkApi {
public:
    PCall_Back_Frame callback = nullptr;
    void* user = nullptr;
    uint32_t init(uint32_t, uint32_t* count) override { *count = 1; return NORI_OK; }
    uint32_t uninit() override { return NORI_OK; }
    uint32_t get_device_info(uint32_t, DEVICE_INFO*) override { return NORI_OK; }
    uint32_t get_device_video_info_size(uint32_t, uint32_t* size) override { *size = 1; return NORI_OK; }
    uint32_t get_device_video_info(uint32_t, uint32_t, VIDEO_INFO* info) override {
        info->u_Format = VIDEO_MEDIA_TYPE_YUYV; info->u_Width = 2; info->u_Height = 2; info->f_Fps = 100;
        return NORI_OK;
    }
    uint32_t device_video_init(uint32_t, VIDEO_INFO) override { return NORI_OK; }
    uint32_t device_video_uninit(uint32_t) override { return NORI_OK; }
    uint32_t video_callback(uint32_t, PCall_Back_Frame cb, void* context) override {
        callback = cb; user = context; return NORI_OK;
    }
    uint32_t get_trigger_mode(uint32_t, E_TRIGGER_MODE* mode) override { *mode = NON_TRIIGER_MODE; return NORI_OK; }
    uint32_t set_trigger_mode(uint32_t, E_TRIGGER_MODE) override { return NORI_OK; }
    uint32_t video_start(uint32_t) override { return NORI_OK; }
    uint32_t video_stop(uint32_t) override { return NORI_OK; }
    uint32_t get_processing_unit_control(uint32_t, int32_t, int32_t* current, int32_t*, int32_t*,
                                          int32_t*, int32_t*, int32_t*) override {
        *current = V4L2_EXPOSURE_AUTO; return NORI_OK;
    }
    uint32_t set_processing_unit_control(uint32_t, int32_t, int32_t) override { return NORI_OK; }
    uint32_t set_sensor_shutter(uint32_t, uint32_t) override { return NORI_OK; }
    uint32_t get_sensor_shutter(uint32_t, uint32_t* value) override { *value = 1000; return NORI_OK; }
    uint32_t set_sensor_gain(uint32_t, uint32_t) override { return NORI_OK; }
    uint32_t get_sensor_gain(uint32_t, uint32_t* current, uint32_t*, uint32_t*, uint32_t*) override {
        *current = 1; return NORI_OK;
    }
    void yuyv_to_bgr24(uint8_t*, uint8_t* out, int width, int height) override {
        std::memset(out, 0, width * height * 3);
    }
    /** @param sequence Driver acquisition sequence, not buffer slot index.
     * @param age_ns Age of the independently supplied SDK capture time.
     * @return Exact uncompensated SDK time after timeval microsecond quantization.
     */
    std::int64_t emit(std::uint32_t sequence, std::int64_t age_ns = 0) {
        FRAME_BUFFER_DATA frame{};
        std::array<std::uint8_t, 8> pixels{};
        get_device_video_info(0, 0, &frame.PixFormat);
        frame.pBufAddr = pixels.data(); frame.buff_Length = frame.buffer.length = pixels.size();
        frame.buffer.sequence = sequence; frame.index = 0;
        const auto time = mosas::runtime::monotonic_timestamp_ns() - age_ns;
        frame.Frame_Time.tv_sec = time / 1'000'000'000;
        frame.Frame_Time.tv_usec = time % 1'000'000'000 / 1000;
        callback(nullptr, &frame, user);
        return frame.Frame_Time.tv_sec * 1'000'000'000LL + frame.Frame_Time.tv_usec * 1000;
    }
};

/** @brief Remove calibration output even when a regression assertion fails. */
struct CalibrationDirectory {
    std::filesystem::path path = std::filesystem::temp_directory_path() /
        ("mosas-warmup-calibration-" + std::to_string(mosas::runtime::monotonic_timestamp_ns()));
    ~CalibrationDirectory() { std::error_code error; std::filesystem::remove_all(path, error); }
};

/** @return Zero if healthy warmup is excluded from errors and calibration can complete. */
int warmup_calibration() {
    using namespace mosas;
    app::AppConfig config;
    config.camera.width = config.camera.height = 2;
    config.camera.fps = 100;
    config.camera.pixel_format = "YUYV";
    config.camera.auto_exposure = false;
    config.camera.camera_timestamp_bias_ns = -80'000'000;
    config.camera.frame_timestamp_domain = runtime::CameraTimestampDomain::monotonic;
    config.camera.timing = {100, 20'000'000, 20'000'000, 50'000'000, 1'000'000'000};
    config.imu.mode = app::ImuMode::serial;
    CalibrationDirectory directory;
    app::CameraImuSyncOptions options;
    options.directory = directory.path;
    options.fps = 60;
    app::CameraImuSyncSession session(options, config);
    session.start();
    auto sdk = std::make_shared<CameraSdk>();
    app::NoriSdkCameraSource camera(config.camera, sdk);
    CHECK(camera.configure({}).status == runtime::SourceStatus::ok);
    sdk->emit(0);
    CHECK(camera.time_diagnostics().warmup_discarded == 1);
    CHECK(camera.camera_timing().timestamp_dropped == 0);

    unsigned recorded = 0;
    std::int64_t previous_recorded = -1;
    auto next_frame = std::chrono::steady_clock::now();
    for (unsigned sequence = 1; sequence <= 50 && recorded < 2; ++sequence) {
        next_frame += std::chrono::milliseconds(10);
        std::this_thread::sleep_until(next_frame);
        const auto published = camera.time_diagnostics().published;
        sdk->emit(sequence);
        if (camera.time_diagnostics().published == published) continue;
        runtime::CameraFrame frame;
        CHECK(camera.capture(&frame).status == runtime::SourceStatus::ok);
        if (previous_recorded >= 0 && frame.timestamp_ns - previous_recorded < 20'000'000) continue;
        const auto observed = runtime::monotonic_timestamp_ns();
        CHECK(session.record_camera(frame, observed));
        runtime::ImuAngularVelocitySample sample;
        sample.timestamp_ns = frame.timestamp_ns;
        CHECK(session.record_imu(sample, observed));
        previous_recorded = frame.timestamp_ns;
        ++recorded;
    }
    camera.stop();
    CHECK(recorded == 2);
    CHECK(camera.time_diagnostics().warmup_discarded > 0);
    CHECK(camera.camera_timing().timestamp_dropped == 0);
    app::CameraImuSyncSummary summary;
    summary.imu.available = true;
    summary.camera_timestamp_dropped = camera.camera_timing().timestamp_dropped;
    CHECK(session.finish(app::CameraImuSyncEnd::duration_elapsed, summary));
    return 0;
}

/** @return Zero if compensation preserves synchronization, raw evidence and the launch policy. */
int compensated_sequence_clock(std::int64_t bias) {
    using namespace mosas;
    app::CameraAppConfig config;
    config.width = config.height = 2;
    config.fps = 100;
    config.pixel_format = "YUYV";
    config.frame_timestamp_domain = runtime::CameraTimestampDomain::monotonic;
    config.timing = {100, 0, 0, 50'000'000, 1'000'000'000};
    config.camera_timestamp_bias_ns = bias;
    auto sdk = std::make_shared<CameraSdk>();
    auto sync = std::make_shared<domain::AcquisitionSync>(1);
    app::NoriSdkCameraSource camera(config, sdk);
    camera.bind_acquisition_sync(sync);
    CHECK(camera.configure({}).status == runtime::SourceStatus::ok);
    sdk->emit(0, 100'000'000); // Compensation must not conceal a stale startup frame.
    CHECK(camera.time_diagnostics().published == 0);
    CHECK(camera.camera_timing().timestamp_dropped == 1);
    const auto before = runtime::monotonic_timestamp_ns();
    const auto raw_sdk = sdk->emit(1);
    const auto after = runtime::monotonic_timestamp_ns();
    CHECK(camera.time_diagnostics().published == 1);
    CHECK(sync->state().ready && sync->state().epoch == 0);
    runtime::CameraFrame first;
    CHECK(camera.capture(&first).status == runtime::SourceStatus::ok);
    CHECK(first.timestamp_ns == raw_sdk + bias);
    CHECK(first.sensor_time->source_time_ns == raw_sdk);
    CHECK(first.received_timestamp_ns >= before && first.received_timestamp_ns <= after);
    CHECK(camera.time_diagnostics().last_receive_ns == first.received_timestamp_ns + bias);
    CHECK(camera.time_diagnostics().last_residual_ns == first.received_timestamp_ns - raw_sdk);
    CHECK(camera.time_diagnostics().baseline_ns == first.timestamp_ns);
    CHECK(camera.time_diagnostics().baseline_compensation_ns == bias);
    const bool within_capture_limits = runtime::validate_camera_timestamp(first.timestamp_ns,
        first.received_timestamp_ns, -1, 50'000'000, 2'000'000'000);
    CHECK(within_capture_limits == (bias <= 0)); // Positive bias must not bypass the future-time guard.

    sdk->emit(1000); // Real prelaunch sequence/time disagreement must still cause resynchronization.
    CHECK(!sync->state().ready && sync->state().epoch == 1);
    CHECK(camera.time_diagnostics().residual_exceeded == 1);
    sdk->emit(1001);
    CHECK(camera.time_diagnostics().published == 2 && sync->state().ready);
    CHECK(camera.capture(&first).status == runtime::SourceStatus::ok);
    CHECK(sync->latch_launch(1));
    sdk->emit(2000, runtime::monotonic_timestamp_ns() + 1'000'000'000);
    CHECK(camera.time_diagnostics().published == 3); // Bad raw SDK time is diagnostic only after launch.
    runtime::CameraFrame later;
    CHECK(camera.capture(&later).status == runtime::SourceStatus::ok);
    CHECK(later.timestamp_ns - first.timestamp_ns == 999 * 10'000'000LL);
    CHECK(later.sensor_time->source_time_ns == -1);
    CHECK(camera.time_diagnostics().invalid_capture_times == 1);
    CHECK(camera.time_diagnostics().post_launch_anomalies > 0);
    CHECK(camera.time_diagnostics().baselines == 2 && sync->state().epoch == 1);
    camera.stop();
    return 0;
}

/** @return Zero if the real source starts warmup in the same compensated domain as its observations. */
int compensated_warmup(std::int64_t bias) {
    using namespace mosas;
    app::CameraAppConfig config;
    config.width = config.height = 2;
    config.fps = 100;
    config.pixel_format = "YUYV";
    config.frame_timestamp_domain = runtime::CameraTimestampDomain::monotonic;
    config.timing = {100, 40'000'000, 20'000'000, 50'000'000, 1'000'000'000};
    config.camera_timestamp_bias_ns = bias;
    auto sdk = std::make_shared<CameraSdk>();
    app::NoriSdkCameraSource camera(config, sdk);
    const auto began = runtime::monotonic_timestamp_ns();
    CHECK(camera.configure({}).status == runtime::SourceStatus::ok);
    auto next_frame = std::chrono::steady_clock::now();
    for (unsigned sequence = 0; sequence < 50 && !camera.time_diagnostics().published; ++sequence) {
        std::this_thread::sleep_until(next_frame);
        sdk->emit(sequence);
        next_frame += std::chrono::milliseconds(10);
    }
    const auto timing = camera.time_diagnostics();
    CHECK(timing.published == 1);
    CHECK(timing.last_receive_ns - bias - began >= config.timing.warmup_ns + config.timing.stable_window_ns);
    CHECK(timing.residual_exceeded == 0 && timing.baselines == 1);
    CHECK(camera.camera_timing().timestamp_dropped == 0);
    camera.stop();
    return 0;
}

/** @return Zero if impossible compensated host clocks fail before synchronization or publication. */
int invalid_compensation() {
    using namespace mosas;
    for (auto bias : {std::numeric_limits<std::int64_t>::min(), std::numeric_limits<std::int64_t>::max()}) {
        app::CameraAppConfig config;
        config.width = config.height = 2;
        config.fps = 100;
        config.pixel_format = "YUYV";
        config.camera_timestamp_bias_ns = bias;
        auto sync = std::make_shared<domain::AcquisitionSync>(1);
        auto sdk = std::make_shared<CameraSdk>();
        app::NoriSdkCameraSource camera(config, sdk);
        camera.bind_acquisition_sync(sync);
        CHECK(camera.configure({}).status == runtime::SourceStatus::fatal);
        CHECK(!sync->state().ready);
        CHECK(camera.time_diagnostics().published == 0);
    }
    return 0;
}
}
int main() {
    using namespace mosas;
    app::CameraAppConfig config;
    config.width = config.height = 2; config.fps = 100; config.pixel_format = "YUYV";
    config.frame_timestamp_domain = runtime::CameraTimestampDomain::monotonic;
    config.timing = {100, 0, 0, 50'000'000, 1'000'000'000};
    config.callback_queue_capacity = 1;
    auto sdk = std::make_shared<CameraSdk>();
    app::NoriSdkCameraSource camera(config, sdk);
    auto sync = std::make_shared<domain::AcquisitionSync>(1);
    camera.bind_acquisition_sync(sync);
    CHECK(camera.configure({}).status == runtime::SourceStatus::ok);
    sdk->emit(0xfffffffdu, 1'000'000'000); // Startup backlog must not set the baseline.
    CHECK(camera.time_diagnostics().baselines == 0);
    CHECK(camera.camera_timing().timestamp_dropped == 1);
    sdk->emit(0xfffffffeu);
    runtime::CameraFrame first;
    CHECK(camera.capture(&first).status == runtime::SourceStatus::ok);
    CHECK(sync->latch_launch(0));
    sdk->emit(0xffffffffu);
    sdk->emit(1u, 1'000'000'000); // Wrap plus a known missing slot; stale SDK time is diagnostic only.
    runtime::CameraFrame later;
    CHECK(camera.capture(&later).status == runtime::SourceStatus::ok);
    CHECK(later.timestamp_ns - first.timestamp_ns == 30'000'000);
    CHECK(later.acquisition_sequence - first.acquisition_sequence == 3);
    CHECK(camera.time_diagnostics().known_missing == 1);
    CHECK(camera.camera_timing().callback_queue_dropped == 1);
    CHECK(later.sensor_time->source == domain::TimestampSource::sequence_fixed_period);
    sdk->emit(1u); sdk->emit(0u);
    CHECK(camera.time_diagnostics().sequence_errors == 2);
    CHECK(camera.time_diagnostics().baselines == 1);
    camera.stop();
    CHECK(camera.time_diagnostics().baselines == 1);
    sync->restart();
    CHECK(camera.configure({}).status == runtime::SourceStatus::ok);
    sdk->emit(0u);
    CHECK(camera.capture(&later).status == runtime::SourceStatus::ok);
    CHECK(later.acquisition_sequence == 0);
    sdk->emit(1000u); // A genuine prelaunch clock residual failure must still be counted.
    CHECK(camera.time_diagnostics().residual_exceeded == 1);
    CHECK(camera.camera_timing().timestamp_dropped == 1);
    camera.stop();
    CHECK(warmup_calibration() == 0);
    for (auto bias : {0LL, -80'000'000LL, 80'000'000LL}) {
        CHECK(compensated_sequence_clock(bias) == 0);
        CHECK(compensated_warmup(bias) == 0);
    }
    CHECK(invalid_compensation() == 0);
    return 0;
}
