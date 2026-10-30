#include <mosas/guidance/ballistic_png_guidance.hpp>
#include <mosas/guidance/inertial_angle_los_filter.hpp>
#include <mosas/runtime/runtime_supervisor.hpp>
#include "fakes/fake_algorithm_ports.hpp"
#include "fakes/fake_camera_source.hpp"
#include "fakes/fake_clock.hpp"
#include "fakes/fake_command_sink.hpp"
#include "fakes/fake_imu_source.hpp"
#include "test_support.hpp"
#include <chrono>
#include <thread>

/** @brief 脚本化视觉 LOS，数学滤波和导引仍使用生产实现。 */
class ScriptedLos final : public mosas::ports::ILosEstimator {
public:
    mosas::domain::DartLineOfSight estimate(const mosas::domain::VisionDetection&,
        const mosas::domain::CameraIntrinsics&,
        const mosas::domain::Rotation3<mosas::domain::DartFrameTag,mosas::domain::CameraFrameTag>&) override {
        const auto index=next_++;
        if(index>=8)return {};
        return {true, index >= 6 ? mosas::domain::DirectionDartUnit{0, 1, 0}
                                  : mosas::domain::DirectionDartUnit{1, 0, 0}};
    }
private:
    int next_=0;
};
/** @brief 真实滤波/导引流水线验证未检出、重捕获、超时、极点和非法 LOS 的提交边界。 */
int main() {
    using namespace mosas;
    std::vector<runtime::ImuAngularVelocitySample> samples;
    for (int i = 0; i <= 22; ++i) samples.push_back({i * 10'000'000LL, {0,9.80665,0}, {},0,0});
    auto imu = std::make_unique<test::FakeImuSource>(samples);
    auto* imu_control = imu.get();
    std::vector<runtime::CameraFrame> frames;
    for (int ms : {10,20,30,40,50,150,160,170,180,190,200}) {
        auto pixels = std::make_shared<std::vector<std::uint8_t>>(3,0);
        runtime::CameraFrame frame;
        frame.timestamp_ns = ms * 1'000'000LL;
        frame.image.owner = pixels; frame.image.data = pixels->data();
        frame.image.data_size_bytes = 3; frame.image.width = frame.image.height = 1;
        frame.image.stride_bytes = 3; frame.image.format = domain::PixelFormat::bgr8;
        frames.push_back(frame);
    }
    auto camera = std::make_unique<test::FakeCameraSource>(frames);
    auto* camera_control = camera.get();
    auto sink = std::make_unique<test::FakeCommandSink>();
    auto* sink_control = sink.get();
    auto flight = std::make_unique<test::FakeFlightStateEstimator>();
    auto* flight_control = flight.get();
    auto vision = std::make_unique<test::FakeVisionDetector>();
    auto* vision_control = vision.get();
    auto los = std::make_unique<ScriptedLos>();
    auto deps = test::mapped_dependencies();
    deps.flight_state_estimator = std::move(flight);
    deps.vision_detector = std::move(vision);
    deps.los_estimator = std::move(los);
    deps.los_rate_filter = std::make_unique<guidance::InertialAngleLosFilter>();
    deps.guidance_law = std::make_unique<guidance::BallisticPngGuidance>();
    deps.monotonic_clock = std::make_unique<test::FakeClock>(230'000'000);
    runtime::RuntimeSupervisorConfig config;
    config.capture_queue_capacity = 16;
    runtime::RuntimeSupervisor supervisor(std::move(imu), std::move(camera), std::move(sink), config, std::move(deps), {});
    CHECK(supervisor.start());
    domain::FlightStateSnapshot state;
    state.attitude.valid = true; state.speed_mps.value = 50;
    state.launch.phase = domain::LaunchPhase::launched;
    state.launch.transition_time.monotonic_ns = 0;
    flight_control->set_result(state);
    for (int i = 0; i < 11; ++i) {
        domain::VisionDetection detected;
        detected.observation.valid = i != 2 && i != 7;
        detected.observation.reason = detected.observation.valid ? domain::ObservationReason::found : domain::ObservationReason::not_found;
        vision_control->push(detected);
    }
    imu_control->release();
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (std::chrono::steady_clock::now() < deadline &&
           (!supervisor.latest_state() || supervisor.latest_state()->time.monotonic_ns != 220'000'000))
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    CHECK(supervisor.latest_state() && supervisor.latest_state()->time.monotonic_ns == 220'000'000);
    camera_control->release();
    while (std::chrono::steady_clock::now() < deadline &&
           supervisor.timing().vision.samples < 11) std::this_thread::sleep_for(std::chrono::milliseconds(1));
    supervisor.stop();
    CHECK(!supervisor.faulted());
    const auto requests = sink_control->commands();
    CHECK(requests.size() == 9); // 第 30/170ms 正常未检出必须没有提交。
    const std::array<int,9> times{10,20,40,50,150,160,180,190,200};
    for (std::size_t i = 0; i < requests.size(); ++i) {
        CHECK(requests[i].source_time.monotonic_ns == times[i] * 1'000'000LL);
        CHECK(requests[i].target_found);
        const bool valid = i == 1 || i == 3 || i == 5;
        CHECK(requests[i].valid == valid);
        if (!valid) CHECK(requests[i].overload_g.y == 0 && requests[i].overload_g.z == 0);
        else CHECK(requests[i].overload_g.y > 0);
    }
    CHECK(requests[7].invalid_reason == domain::GuidanceInvalidReason::invalid_rate);
    CHECK(requests[8].invalid_reason == domain::GuidanceInvalidReason::invalid_los);
    return 0;
}
