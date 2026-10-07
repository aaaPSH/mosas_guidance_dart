#include <mosas/runtime/attitude_estimator.hpp>
#include <mosas/runtime/legacy_flight_state_adapter.hpp>

#include "test_support.hpp"

int main() {
    mosas::runtime::AttitudeEstimator estimator;

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

    mosas::runtime::LegacyFlightStateEstimatorAdapter adapter;
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
    adapter.reset();
    return 0;
}
