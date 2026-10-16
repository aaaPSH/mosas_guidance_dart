#include <mosas/runtime/attitude_estimator.hpp>
#include <mosas/runtime/attitude_history.hpp>
#include <mosas/runtime/legacy_flight_state_adapter.hpp>

#include "test_support.hpp"

#include <limits>

int main() {
    mosas::runtime::AttitudeEstimatorConfig config;
    config.launch_speed_mps = 16.0;
    mosas::runtime::AttitudeEstimator estimator(config);

    const auto stationary = estimator.update({
        0,
        {0.0, 0.0, 9.80665},
        {0.0, 0.0, 0.0},
        0.0,
        0.0,
    });
    CHECK(stationary.valid);
    CHECK(!stationary.launched);
    CHECK(stationary.launch_timestamp_ns == -1);
    CHECK_NEAR(stationary.speed_mps, 16.0, 1e-12);
    CHECK_NEAR(stationary.pitch_rad, 0.0, 1e-12);
    CHECK_NEAR(stationary.roll_rad, -1.5707963267948966, 1e-12);

    const auto candidate = estimator.update({
        10'000'000,
        {15.0, 0.0, 9.80665},
        {0.0, 0.0, 0.0},
        0.0,
        0.0,
    });
    CHECK(candidate.valid);
    CHECK(!candidate.launched);
    CHECK(candidate.launch_timestamp_ns == -1);
    CHECK_NEAR(candidate.speed_mps, 16.0, 1e-12);

    const auto interrupted_candidate = estimator.update({
        20'000'000,
        {0.0, 0.0, 9.80665},
        {0.0, 0.0, 0.0},
        0.0,
        0.0,
    });
    CHECK(!interrupted_candidate.launched);

    const auto new_candidate = estimator.update({
        30'000'000,
        {15.0, 0.0, 9.80665},
        {0.0, 0.0, 0.0},
        0.0,
        0.0,
    });
    CHECK(!new_candidate.launched);

    const auto launched = estimator.update({
        50'000'000,
        {15.0, 0.0, 9.80665},
        {0.0, 0.0, 0.0},
        0.0,
        0.0,
    });
    CHECK(launched.valid);
    CHECK(launched.launched);
    CHECK(launched.launch_timestamp_ns == 50'000'000);
    CHECK_NEAR(launched.speed_mps, 16.0, 1e-12);
    CHECK_NEAR(launched.pitch_rad, 0.0, 1e-12);
    CHECK_NEAR(launched.roll_rad, -1.5707963267948966, 1e-12);
    CHECK_NEAR(launched.corrected_angular_velocity.x, 0.0, 1e-12);
    CHECK_NEAR(launched.corrected_angular_velocity.y, 0.0, 1e-12);
    CHECK_NEAR(launched.corrected_angular_velocity.z, 0.0, 1e-12);

    const auto duplicate_timestamp = estimator.update({
        50'000'000,
        {100.0, 0.0, 9.80665},
        {1.0, 0.0, 0.0},
        0.0,
        0.0,
    });
    CHECK(duplicate_timestamp.launched);
    CHECK(duplicate_timestamp.launch_timestamp_ns == 50'000'000);
    CHECK_NEAR(duplicate_timestamp.speed_mps, 16.0, 1e-12);

    const auto accelerating = estimator.update(
        {60'000'000, {2.0, 0.0, 0.0}, {}, 0.0, 0.0});
    CHECK_NEAR(accelerating.speed_mps, 16.02, 1e-12);
    const auto decelerating = estimator.update(
        {80'000'000, {-3.0, 0.0, 0.0}, {}, 0.0, 0.0});
    CHECK_NEAR(decelerating.speed_mps, 15.96, 1e-12);
    const auto reversed = estimator.update(
        {70'000'000, {100.0, 0.0, 0.0}, {}, 0.0, 0.0});
    CHECK(reversed.timestamp_ns == 80'000'000);
    CHECK_NEAR(reversed.speed_mps, 15.96, 1e-12);
    const auto invalid = estimator.update(
        {90'000'000, {std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0},
         {}, 0.0, 0.0});
    CHECK(invalid.timestamp_ns == 80'000'000);
    CHECK_NEAR(invalid.speed_mps, 15.96, 1e-12);
    const auto gap = estimator.update(
        {3'000'000'000, {100.0, 0.0, 0.0}, {}, 0.0, 0.0});
    CHECK_NEAR(gap.speed_mps, 15.96, 1e-12);
    const auto resumed = estimator.update(
        {3'010'000'000, {1.0, 0.0, 0.0}, {}, 0.0, 0.0});
    CHECK_NEAR(resumed.speed_mps, 15.97, 1e-12);
    estimator.reset();
    CHECK_NEAR(estimator.update({0, {0.0, 9.80665, 0.0}, {}, 0.0, 0.0})
                   .speed_mps, 16.0, 1e-12);

    // 非零俯仰和角速度确保速度积分使用更新后的当前帧俯仰，而非上一帧。
    mosas::runtime::AttitudeEstimator pitched_estimator(config);
    const double initial_pitch = 0.4;
    const double gravity = config.gravity_mps2;
    pitched_estimator.update(
        {0, {gravity * std::sin(initial_pitch),
             gravity * std::cos(initial_pitch), 0.0}, {}, 0.0, 0.0});
    pitched_estimator.update(
        {10'000'000, {30.0, 0.0, 0.0}, {}, 0.0, 0.0});
    const auto pitched_launch = pitched_estimator.update(
        {30'000'000, {30.0, 0.0, 0.0}, {}, 0.0, 0.0});
    CHECK(pitched_launch.launched);
    CHECK_NEAR(pitched_launch.speed_mps, 16.0, 1e-12);
    const auto pitched = pitched_estimator.update(
        {40'000'000, {2.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, 0.0, 0.0});
    CHECK_NEAR(pitched.pitch_rad, initial_pitch + 0.01, 1e-12);
    CHECK_NEAR(pitched.speed_mps,
               16.0 + (2.0 - gravity * std::sin(pitched.pitch_rad)) * 0.01,
               1e-12);
    const auto balanced = pitched_estimator.update(
        {50'000'000, {gravity * std::sin(pitched.pitch_rad), 0.0, 0.0},
         {}, 0.0, 0.0});
    CHECK_NEAR(balanced.speed_mps, pitched.speed_mps, 1e-12);

    mosas::runtime::LegacyFlightStateEstimatorAdapter adapter(config);
    const auto adapt_sample = [](std::int64_t timestamp_ns, double acceleration_x) {
        mosas::domain::ImuKinematicSample sample{};
        sample.time.monotonic_ns = timestamp_ns;
        sample.time.source_domain = mosas::domain::ClockDomain::monotonic;
        sample.time.source =
            mosas::domain::TimestampSource::imu_uart_receive_estimate;
        sample.time.quality = mosas::domain::TimestampQuality::estimated;
        sample.acceleration = {acceleration_x, 0.0, 9.80665};
        return sample;
    };
    adapter.update(adapt_sample(0, 0.0));
    adapter.update(adapt_sample(10'000'000, 15.0));
    adapter.update(adapt_sample(20'000'000, 0.0));
    adapter.update(adapt_sample(30'000'000, 15.0));
    const auto adapted_flight_state =
        adapter.update(adapt_sample(50'000'000, 15.0));
    CHECK(adapted_flight_state.attitude.valid);
    CHECK(adapted_flight_state.launch.phase ==
          mosas::domain::LaunchPhase::launched);
    CHECK(adapted_flight_state.launch.transition_time.monotonic_ns ==
          50'000'000);
    CHECK(adapted_flight_state.launch.reason ==
          mosas::domain::LaunchReason::threshold_confirmed);
    CHECK_NEAR(adapted_flight_state.attitude.pitch_rad.value, 0.0, 1e-12);
    CHECK_NEAR(adapted_flight_state.attitude.roll_rad.value,
               -1.5707963267948966, 1e-12);
    CHECK_NEAR(adapted_flight_state.attitude.corrected_angular_velocity.z,
               0.0, 1e-12);
    CHECK_NEAR(adapted_flight_state.speed_mps.value, 16.0, 1e-12);
    const auto adapted_accelerating = adapter.update(adapt_sample(60'000'000, 2.0));
    CHECK_NEAR(adapted_accelerating.speed_mps.value, 16.02, 1e-12);
    adapter.reset();
    CHECK_NEAR(adapter.update(adapt_sample(0, 0.0)).speed_mps.value, 16.0, 1e-12);

    mosas::runtime::AttitudeHistory history(4);
    mosas::runtime::AttitudeSample history_sample;
    history_sample.timestamp_ns = 10'000'000;
    history_sample.launched = true;
    history_sample.launch_timestamp_ns = 0;
    history_sample.speed_mps = 16.0;
    CHECK(history.push(history_sample));
    history_sample.timestamp_ns = 30'000'000;
    history_sample.speed_mps = 14.0;
    CHECK(history.push(history_sample));
    const auto exact = history.find_interpolated(10'000'000);
    CHECK(exact && !exact->interpolated);
    CHECK_NEAR(exact->speed_mps, 16.0, 1e-12);
    const auto middle = history.find_interpolated(15'000'000);
    CHECK(middle && middle->interpolated);
    CHECK_NEAR(middle->speed_mps, 15.5, 1e-12);
    CHECK(!history.find_interpolated(40'000'000));
    history_sample.timestamp_ns = 40'000'000;
    history_sample.launch_generation = 1;
    CHECK(history.push(history_sample));
    CHECK(!history.find_interpolated(35'000'000));
    history_sample.timestamp_ns = 50'000'000;
    history_sample.speed_mps = std::numeric_limits<double>::infinity();
    CHECK(!history.push(history_sample));
    history.reset();
    CHECK(!history.find_interpolated(10'000'000));
    return 0;
}
