#include <mosas/guidance/body_vector_los_filter.hpp>
#include <mosas/guidance/body_vector_pn_guidance.hpp>
#include <mosas/runtime/runtime_supervisor.hpp>

#include "fakes/fake_algorithm_ports.hpp"
#include "fakes/fake_camera_source.hpp"
#include "fakes/fake_clock.hpp"
#include "fakes/fake_command_sink.hpp"
#include "fakes/fake_imu_source.hpp"
#include "test_support.hpp"

#include <chrono>
#include <cmath>
#include <memory>
#include <thread>
#include <vector>

namespace {

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
    runtime::RuntimeSupervisorDependencies dependencies;
    dependencies.flight_state_estimator = std::move(flight);
    dependencies.vision_detector = std::move(vision);
    dependencies.los_estimator = std::move(los);
    dependencies.los_rate_filter = std::make_unique<guidance::BodyVectorLosFilter>();
    dependencies.guidance_law = std::make_unique<guidance::BodyVectorPnGuidance>();
    dependencies.monotonic_clock = std::make_unique<test::FakeClock>(1'100'000'000);
    runtime::RuntimeSupervisorConfig config;
    config.launch_speed_mps = 13.0;
    config.capture_queue_capacity = 16;
    config.history_capacity = 32;
    runtime::RuntimeSupervisor runtime(std::move(imu), std::move(camera), std::move(commands),
                                       config, std::move(dependencies), {});
    CHECK(runtime.start());
    domain::FlightStateSnapshot state;
    state.attitude.valid = true;
    state.launch.phase = domain::LaunchPhase::launched;
    state.launch.transition_time.monotonic_ns = 900'000'000;
    flight_control->set_result(state);
    // 极点处两个角速率诊断无效，但矢量估计仍有效，应正常调用新 PN。
    los_control->set_result({true, {std::acos(-1.0) / 2.0}, {0.0}});
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
    while (std::chrono::steady_clock::now() < deadline && command_control->commands().size() < 8) {
        CHECK(!runtime.faulted());
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    runtime.stop();
    CHECK(!runtime.faulted());
    const auto output = command_control->commands();
    CHECK(output.size() == 8);
    for (std::size_t i = 0; i < output.size(); ++i) {
        CHECK(output[i].source_time.monotonic_ns == timestamps[i]);
        const bool should_be_valid = i == 1 || i == 4 || i == 7;
        CHECK(output[i].valid == should_be_valid);
        if (should_be_valid) {
            CHECK(output[i].source_frame_id == i);
            CHECK_NEAR(output[i].overload_g.y, 1.0, 1e-12);
        } else {
            CHECK_NEAR(output[i].overload_g.y, 0.0, 1e-12);
        }
        CHECK_NEAR(output[i].overload_g.x, 0.0, 1e-12);
        CHECK_NEAR(output[i].overload_g.z, 0.0, 1e-12);
    }
    return 0;
}
