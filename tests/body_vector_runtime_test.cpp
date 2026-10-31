#include <mosas/guidance/ballistic_png_guidance.hpp>
#include <mosas/guidance/inertial_angle_los_filter.hpp>
#include <mosas/runtime/legacy_flight_state_adapter.hpp>
#include <mosas/runtime/runtime_supervisor.hpp>

#include "fakes/fake_algorithm_ports.hpp"
#include "fakes/fake_camera_source.hpp"
#include "fakes/fake_clock.hpp"
#include "fakes/fake_command_sink.hpp"
#include "fakes/fake_imu_source.hpp"
#include "test_support.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace {

/** @brief 捕获录像端收到的逐帧叠加快照，验证距离与当前观测同步且不会残留。 */
class CapturingVideoRecorder final : public mosas::runtime::IVideoRecorder {
public:
    bool start() override { return true; }
    void reset() noexcept override {
        std::lock_guard<std::mutex> lock(mutex_);
        overlays_.clear();
    }
    bool record(const mosas::runtime::ProcessedFrame& frame) override {
        std::lock_guard<std::mutex> lock(mutex_);
        overlays_.push_back(frame.overlay);
        return true;
    }
    void cancel() noexcept override {}
    void stop() noexcept override {}
    std::string last_error() const override { return {}; }

    /** @return 按输出顺序复制已收到的叠加快照；可与录像 worker 并发调用。 */
    std::vector<mosas::domain::GuidanceOverlay> overlays() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return overlays_;
    }

private:
    mutable std::mutex mutex_;
    std::vector<mosas::domain::GuidanceOverlay> overlays_;
};

/** @brief 创建有效的拥有型测试帧，不依赖 OpenCV 或相机 SDK。 */
mosas::runtime::CameraFrame make_frame(std::int64_t timestamp) {
    auto pixels = std::make_shared<std::vector<std::uint8_t>>(3, 0);
    mosas::runtime::CameraFrame frame;
    frame.timestamp_ns = timestamp;
    frame.image.owner = pixels;
    frame.image.data = pixels->data();
    frame.image.data_size_bytes = pixels->size();
    frame.image.width = frame.image.height = 1;
    frame.image.stride_bytes = 3;
    frame.image.format = mosas::domain::PixelFormat::bgr8;
    return frame;
}

/**
 * @brief 验证唯一弹道 PNG 的实际速度积分与曝光时刻插值。
 * @param mode 发射触发判据；验证输出时机和后续速度插值随模式选择。
 * @return 全部断言通过时为 0。
 */
int verify_dynamic_speed(mosas::runtime::LaunchDetectionMode mode) {
    using namespace mosas;
    const bool two_stage = mode == runtime::LaunchDetectionMode::two_stage;
    const std::vector<runtime::TimestampNs> command_times = two_stage
        ? std::vector<runtime::TimestampNs>{60'000'000, 70'000'000, 90'000'000}
        : std::vector<runtime::TimestampNs>{40'000'000, 45'000'000, 60'000'000,
                                            70'000'000, 90'000'000};
    const std::vector<double> guidance_speeds = two_stage
        ? std::vector<double>{15.99, 15.97}
        : std::vector<double>{16.015, 16.04, 16.01, 15.99};
    constexpr double gravity = 9.80665;
    auto imu = std::make_unique<test::FakeImuSource>(
        std::vector<runtime::ImuAngularVelocitySample>{
            {0, {0.0, gravity, 0.0}, {}, 0.0, 0.0},
            {10'000'000, {30.0, gravity, 0.0}, {}, 0.0, 0.0},
            {30'000'000, {30.0, gravity, 0.0}, {}, 0.0, 0.0},
            {40'000'000, {1.0, 0.0, 0.0}, {}, 0.0, 0.0},
            {50'000'000, {1.0, 0.0, 0.0}, {}, 0.0, 0.0},
            {60'000'000, {2.0, 0.0, 0.0}, {}, 0.0, 0.0},
            {80'000'000, {-3.0, 0.0, 0.0}, {}, 0.0, 0.0},
            {100'000'000, {1.0, 0.0, 0.0}, {}, 0.0, 0.0}});
    auto* imu_control = imu.get();
    auto camera = std::make_unique<test::FakeCameraSource>(
        std::vector<runtime::CameraFrame>{make_frame(20'000'000),
                                        make_frame(40'000'000),
                                        // two_stage 下区间跨越 50 ms 确认点；acceleration 下已触发。
                                        make_frame(45'000'000),
                                        make_frame(60'000'000),
                                        make_frame(70'000'000),
                                        make_frame(90'000'000)});
    auto* camera_control = camera.get();
    auto commands = std::make_unique<test::FakeCommandSink>();
    auto* command_control = commands.get();
    auto vision = std::make_unique<test::FakeVisionDetector>();
    auto* vision_control = vision.get();
    auto los = std::make_unique<test::FakeLosEstimator>();
    auto* los_control = los.get();
    auto rate = std::make_unique<test::FakeLosRateFilter>();
    auto* rate_control = rate.get();
    auto dependencies = test::mapped_dependencies();
    runtime::AttitudeEstimatorConfig estimator_config;
    estimator_config.launch_detection_mode = mode;
    estimator_config.launch_speed_mps = 16.0;
    dependencies.flight_state_estimator =
        std::make_unique<runtime::LegacyFlightStateEstimatorAdapter>(estimator_config);
    dependencies.vision_detector = std::move(vision);
    dependencies.los_estimator = std::move(los);
    dependencies.los_rate_filter = std::move(rate);
    dependencies.guidance_law = std::make_unique<guidance::BallisticPngGuidance>();
    dependencies.monotonic_clock = std::make_unique<test::FakeClock>(100'000'000);
    runtime::RuntimeSupervisorConfig config;
    config.capture_queue_capacity = 6;
    runtime::RuntimeSupervisor runtime(std::move(imu), std::move(camera),
                                       std::move(commands), config, std::move(dependencies), {});
    CHECK(runtime.start());
    for (int i = 0; i < 5; ++i) {
        domain::VisionDetection detection;
        detection.observation.valid = true;
        vision_control->push(detection);
    }
    los_control->set_result({true, {1.0, 0.0, 0.0}});
    rate_control->set_result({true, {true, {0.2}, {0.0}},
                             {true, {1.0, 0.0, 0.0}, {0.0, 0.2, 0.0}}});
    imu_control->release();
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (std::chrono::steady_clock::now() < deadline) {
        const auto latest = runtime.latest_state();
        if (latest && latest->time.monotonic_ns == 100'000'000)
            break;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    CHECK(runtime.latest_state().has_value());
    CHECK(runtime.latest_state()->time.monotonic_ns == 100'000'000);
    CHECK_NEAR(runtime.latest_state()->speed_mps.value,
               two_stage ? 15.98 : 16.0, 1e-12);
    camera_control->release();
    while (std::chrono::steady_clock::now() < deadline &&
           command_control->commands().size() < command_times.size()) {
        CHECK(!runtime.faulted());
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    runtime.stop();
    CHECK(!runtime.faulted());
    const auto output = command_control->commands();
    CHECK(output.size() == command_times.size());
    // 触发前的图像不能建立飞行速率基准或下发；触发后的首帧仅发送零过载。
    CHECK(!output[0].valid);
    CHECK(rate_control->call_count() == command_times.size());
    const auto& rate_inputs = rate_control->inputs();
    CHECK(rate_inputs.size() == command_times.size());
    for (std::size_t i = 0; i < rate_inputs.size(); ++i) {
        const double speed = i == 0 ? (two_stage ? 16.02 : 16.01) : guidance_speeds[i - 1];
        CHECK_NEAR(rate_inputs[i].speed_mps.value, speed, 1e-12);
        CHECK_NEAR(rate_inputs[i].dt_seconds,
                   i == 0 ? 1.0 : (command_times[i] - command_times[i - 1]) * 1e-9, 1e-12);
    }
    CHECK(runtime.timing().vision.samples == 5);
    CHECK(runtime.timing().guidance.samples == guidance_speeds.size());
    for (std::size_t i = 0; i < output.size(); ++i) {
        CHECK(output[i].source_time.monotonic_ns == command_times[i]);
        CHECK(output[i].launched);
        CHECK(output[i].target_found);
    }
    // 两种模式均使用曝光时刻速度，而非标定初速或最新 IMU 速度。
    for (std::size_t i = 1; i < output.size(); ++i) {
        const double speed = guidance_speeds[i - 1];
        CHECK(output[i].valid);
        const double expected_y =
            3.0 * speed * (0.2 + gravity / (2.0 * speed)) / gravity;
        CHECK_NEAR(output[i].overload_g.y, expected_y, 1e-12);
        CHECK_NEAR(output[i].overload_g.x, 0.0, 1e-12);
        CHECK_NEAR(output[i].overload_g.z, 0.0, 1e-12);
    }
    return 0;
}

} // namespace

int main() {
    using namespace mosas;
    std::vector<runtime::ImuAngularVelocitySample> samples;
    for (int i = 0; i <= 10; ++i)
        samples.push_back(
            {1'000'000'000 + i * 10'000'000, {0.0, 9.80665, 0.0}, {0.0, 0.0, 0.0}, 0.0, 0.0});
    auto imu = std::make_unique<test::FakeImuSource>(samples);
    auto *imu_control = imu.get();
    // 缺目标和无法匹配 IMU 后，恢复首帧必须重新建立速率基准。
    const std::vector<std::int64_t> timestamps{1'010'000'000, 1'020'000'000, 1'030'000'000,
                                               1'040'000'000, 1'050'000'000, 1'200'000'000,
                                               1'060'000'000, 1'070'000'000};
    std::vector<runtime::CameraFrame> frames;
    for (auto timestamp : timestamps)
        frames.push_back(make_frame(timestamp));
    auto camera = std::make_unique<test::FakeCameraSource>(frames);
    auto *camera_control = camera.get();
    auto commands = std::make_unique<test::FakeCommandSink>();
    auto *command_control = commands.get();
    auto flight = std::make_unique<test::FakeFlightStateEstimator>();
    auto *flight_control = flight.get();
    auto vision = std::make_unique<test::FakeVisionDetector>();
    auto *vision_control = vision.get();
    auto los = std::make_unique<test::FakeLosEstimator>();
    auto *los_control = los.get();
    auto dependencies = test::mapped_dependencies();
    dependencies.flight_state_estimator = std::move(flight);
    dependencies.vision_detector = std::move(vision);
    dependencies.los_estimator = std::move(los);
    guidance::InertialAngleLosFilterConfig filter_config;
    filter_config.camera_lever_arm_x = 0.18;
    filter_config.estimated_target_distance = 1.1;
    dependencies.los_rate_filter = std::make_unique<guidance::InertialAngleLosFilter>(filter_config);
    dependencies.guidance_law = std::make_unique<guidance::BallisticPngGuidance>();
    dependencies.monotonic_clock = std::make_unique<test::FakeClock>(1'100'000'000);
    runtime::RuntimeSupervisorConfig config;
    config.capture_queue_capacity = 16;
    config.output_queue_capacity = 16;
    config.history_capacity = 32;
    auto video = std::make_unique<CapturingVideoRecorder>();
    auto* video_control = video.get();
    runtime::RuntimeOutputPorts outputs;
    outputs.video = std::move(video);
    runtime::RuntimeSupervisor runtime(std::move(imu), std::move(camera), std::move(commands),
                                       config, std::move(dependencies), std::move(outputs));
    CHECK(runtime.start());
    domain::FlightStateSnapshot state;
    state.speed_mps.value = 13.0;
    state.attitude.valid = true;
    state.launch.phase = domain::LaunchPhase::launched;
    state.launch.transition_time.monotonic_ns = 900'000'000;
    flight_control->set_result(state);
    los_control->set_result({true, {1.0, 0.0, 0.0}});
    for (int i = 0; i < 7; ++i) {
        domain::VisionDetection detection;
        detection.observation.valid = i != 2;
        vision_control->push(detection);
    }
    imu_control->release();
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (std::chrono::steady_clock::now() < deadline) {
        const auto latest = runtime.latest_state();
        if (latest && latest->time.monotonic_ns == 1'100'000'000)
            break;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    CHECK(runtime.latest_state().has_value());
    CHECK(runtime.latest_state()->time.monotonic_ns == 1'100'000'000);
    camera_control->release();
    while (std::chrono::steady_clock::now() < deadline &&
           (command_control->commands().size() < 7 || video_control->overlays().size() < timestamps.size())) {
        CHECK(!runtime.faulted());
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    runtime.stop();
    CHECK(!runtime.faulted());
    const auto output = command_control->commands();
    CHECK(output.size() == 7);
    const auto overlays = video_control->overlays();
    CHECK(overlays.size() == timestamps.size());
    for (std::size_t i = 0; i < overlays.size(); ++i) {
        const bool distance_valid = i != 2 && i != 5;
        CHECK(overlays[i].lever_arm_distance_valid == distance_valid);
        const double distance = distance_valid ? (i == 1 || i == 4 || i == 7 ? 1.0 : 1.1) : 0.0;
        CHECK_NEAR(overlays[i].lever_arm_distance_m, distance, 1e-12);
    }
    const std::array<std::size_t, 7> submitted_frames{0,1,3,4,5,6,7};
    for (std::size_t i = 0; i < output.size(); ++i) {
        const auto frame_index = submitted_frames[i];
        CHECK(output[i].source_time.monotonic_ns == timestamps[frame_index]);
        const bool should_be_valid = frame_index == 1 || frame_index == 4 || frame_index == 7;
        CHECK(output[i].valid == should_be_valid);
        if (should_be_valid) {
            CHECK(output[i].source_frame_id == frame_index);
            CHECK_NEAR(output[i].overload_g.y, 1.5, 1e-12);
        } else {
            CHECK_NEAR(output[i].overload_g.y, 0.0, 1e-12);
        }
        CHECK_NEAR(output[i].overload_g.x, 0.0, 1e-12);
        CHECK_NEAR(output[i].overload_g.z, 0.0, 1e-12);
    }
    for (const auto mode : {runtime::LaunchDetectionMode::acceleration,
                            runtime::LaunchDetectionMode::two_stage}) {
        CHECK(verify_dynamic_speed(mode) == 0);
    }
    return 0;
}
