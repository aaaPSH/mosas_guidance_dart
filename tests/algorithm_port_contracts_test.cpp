#include <mosas/ports/attitude_estimator.hpp>
#include <mosas/ports/clock.hpp>
#include <mosas/ports/flight_state_estimator.hpp>
#include <mosas/ports/guidance_law.hpp>
#include <mosas/ports/launch_detector.hpp>
#include <mosas/ports/los_estimator.hpp>
#include <mosas/ports/los_rate_filter.hpp>
#include <mosas/ports/vision_detector.hpp>

#include "fakes/fake_algorithm_ports.hpp"
#include "fakes/fake_clock.hpp"
#include "test_support.hpp"

#include <type_traits>
#include <vector>

static_assert(std::is_abstract_v<mosas::ports::IVisionDetector>);
static_assert(std::is_abstract_v<mosas::ports::IAttitudeEstimator>);
static_assert(std::is_abstract_v<mosas::ports::ILaunchDetector>);
static_assert(std::is_abstract_v<mosas::ports::IFlightStateEstimator>);
static_assert(std::is_abstract_v<mosas::ports::ILosEstimator>);
static_assert(std::is_abstract_v<mosas::ports::ILosRateFilter>);
static_assert(std::is_abstract_v<mosas::ports::IGuidanceLaw>);

int main() {
    using namespace mosas;

    test::FakeClock clock(10);
    CHECK(clock.now_ns() == 10);
    clock.advance(7);
    CHECK(clock.now_ns() == 17);
    clock.set(100);
    CHECK(clock.now_ns() == 100);

    std::vector<std::uint8_t> pixels(6, 0);
    domain::ImageView image{pixels.data(), pixels.size(), 2, 1, 6,
                            domain::PixelFormat::bgr8};
    domain::SensorTime time{};
    time.monotonic_ns = 123;
    time.source = domain::TimestampSource::camera_sdk_capture;
    time.quality = domain::TimestampQuality::calibrated;

    test::FakeVisionDetector vision;
    domain::VisionDetection detection{};
    detection.observation.valid = true;
    detection.observation.reason = domain::ObservationReason::found;
    detection.observation.center = {2.5, 0.5};
    vision.push(detection);
    const auto detected = vision.detect(image, 42, time);
    CHECK(detected.observation.valid);
    CHECK(detected.observation.source_frame_id == 42);
    CHECK(detected.observation.time.monotonic_ns == 123);
    CHECK(vision.call_count() == 1);
    vision.reset();
    CHECK(vision.call_count() == 0);
    CHECK(!vision.detect(image, 43, time).observation.valid);

    test::FakeAttitudeEstimator attitude;
    test::FakeLaunchDetector launch;
    test::FakeFlightStateEstimator flight_state;
    domain::ImuKinematicSample imu{};
    imu.time.monotonic_ns = 456;
    const auto attitude_result = attitude.update(imu);
    CHECK(attitude_result.time.monotonic_ns == 456);
    CHECK(attitude.call_count() == 1);
    CHECK(launch.update(imu).transition_time.monotonic_ns == 456);
    CHECK(launch.call_count() == 1);
    flight_state.update(imu);
    CHECK(flight_state.call_count() == 1);
    CHECK(flight_state.last_timestamp_ns() == 456);
    flight_state.reset();
    CHECK(flight_state.call_count() == 0);
    CHECK(flight_state.last_timestamp_ns() == -1);

    test::FakeLosEstimator los;
    los.set_result({true, {0.1}, {-0.2}});
    const domain::VisionDetection los_input{};
    const domain::CameraIntrinsics intrinsics{100.0, 100.0, 50.0, 50.0};
    const domain::Rotation3<domain::DartFrameTag,
                            domain::CameraFrameTag> camera_to_dart;
    const auto los_result = los.estimate(los_input, intrinsics, camera_to_dart);
    CHECK(los_result.valid);
    CHECK_NEAR(los_result.q_y.value, 0.1, 1e-12);
    CHECK_NEAR(los_result.q_z.value, -0.2, 1e-12);
    CHECK(los.call_count() == 1);

    test::FakeLosRateFilter rate_filter;
    rate_filter.set_result({true, {1.0}, {2.0}});
    const auto rate = rate_filter.update(
        los_result, {}, 0.01);
    CHECK(rate.valid);
    CHECK_NEAR(rate.q_y.value, 1.0, 1e-12);
    CHECK(rate_filter.sample().line_of_sight.valid);
    rate_filter.reset();
    CHECK(rate_filter.call_count() == 0);
    CHECK(!rate_filter.sample().line_of_sight.valid);

    test::FakeGuidanceLaw guidance;
    domain::GuidanceOutput output{};
    output.valid = true;
    output.command.valid = true;
    output.command.overload_g = {0.0, 0.0, 0.0};
    guidance.set_result(output);
    const auto guidance_result = guidance.calculate({});
    CHECK(guidance_result.valid);
    CHECK(guidance_result.command.valid);
    CHECK(guidance_result.command.overload_g.x == 0.0);
    CHECK(guidance.call_count() == 1);
    return 0;
}
