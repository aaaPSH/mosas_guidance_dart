#include <mosas/runtime/attitude_estimator.hpp>

#include <cassert>
#include <cmath>

namespace {

constexpr double kGravity = 9.80665;
constexpr mosas::runtime::TimestampNs kMillisecondNs = 1'000'000;

mosas::runtime::ImuAngularVelocitySample sample(
    mosas::runtime::TimestampNs timestamp,
    const Vector3& acceleration,
    const Vector3& angular_velocity = {0.0, 0.0, 0.0}) {
    return {timestamp, acceleration, angular_velocity};
}

mosas::runtime::AttitudeEstimatorConfig test_config() {
    mosas::runtime::AttitudeEstimatorConfig config;
    config.gravity_mps2 = kGravity;
    config.launch_ax_threshold_mps2 = 5.0;
    config.launch_confirm_ns = 20 * kMillisecondNs;
    config.accelerometer_specific_force = true;
    return config;
}

void test_level_specific_force_estimates_zero_pitch() {
    mosas::runtime::AttitudeEstimator estimator(test_config());
    const auto estimate = estimator.update(
        sample(1, {0.0, kGravity, 0.0}));

    assert(estimate.valid);
    assert(!estimate.launched);
    assert(std::abs(estimate.pitch_rad) < 1e-12);
}

void test_gravity_estimates_fixed_pitch() {
    mosas::runtime::AttitudeEstimator estimator(test_config());
    const double pitch = 0.2;
    const Vector3 up_force{
        kGravity * std::sin(pitch),
        kGravity * std::cos(pitch),
        0.0};
    const auto estimate = estimator.update(sample(1, up_force));

    assert(estimate.valid);
    assert(std::abs(estimate.pitch_rad - pitch) < 1e-12);
}

void test_stationary_tolerance_accepts_gravity_attitude() {
    auto config = test_config();
    config.stationary_acceleration_tolerance_mps2 = 0.3;
    mosas::runtime::AttitudeEstimator estimator(config);
    estimator.update(sample(1, {0.0, kGravity, 0.0}));

    const auto estimate = estimator.update(
        sample(2, {0.0, kGravity - 0.1, 0.0}));

    assert(estimate.valid);
    assert(!estimate.launched);
}

void test_prelaunch_nonstationary_integrates_angular_velocity() {
    mosas::runtime::AttitudeEstimator estimator(test_config());
    estimator.update(sample(1, {0.0, kGravity, 0.0}));
    const auto estimate = estimator.update(
        sample(1 + 100 * kMillisecondNs, {0.0, kGravity, 0.0},
               {0.0, 0.0, 1.0}));

    assert(estimate.valid);
    assert(!estimate.launched);
    assert(estimate.pitch_rad > 0.05);
}

void test_x_axis_acceleration_requires_confirmation_duration() {
    mosas::runtime::AttitudeEstimator estimator(test_config());
    estimator.update(sample(0, {0.0, kGravity, 0.0}));
    const auto short_pulse = estimator.update(
        sample(10 * kMillisecondNs, {6.0, kGravity, 0.0}));
    assert(!short_pulse.launched);

    const auto confirmed = estimator.update(
        sample(35 * kMillisecondNs, {6.0, kGravity, 0.0}));
    assert(confirmed.launched);
}

void test_launch_thrust_is_not_used_as_gravity_pitch() {
    auto config = test_config();
    config.launch_ax_threshold_mps2 = 10.0;
    mosas::runtime::AttitudeEstimator estimator(config);
    estimator.update(sample(0, {0.0, kGravity, 0.0}));

    const auto accelerating = estimator.update(
        sample(10 * kMillisecondNs, {6.0, kGravity, 0.0}));
    assert(accelerating.valid);
    assert(std::abs(accelerating.pitch_rad) < 1e-12);

    estimator.update(sample(35 * kMillisecondNs, {12.0, kGravity, 0.0}));
    const auto launched = estimator.update(
        sample(60 * kMillisecondNs, {12.0, kGravity, 0.0}));
    assert(launched.launched);
    assert(std::abs(launched.pitch_rad) < 1e-12);
}

void test_constant_gyro_bias_is_removed_after_launch() {
    mosas::runtime::AttitudeEstimator estimator(test_config());
    constexpr double kGyroBias = 0.1;
    estimator.update(sample(0, {0.0, kGravity, 0.0},
                            {0.0, 0.0, kGyroBias}));
    estimator.update(sample(20 * kMillisecondNs, {6.0, kGravity, 0.0},
                            {0.0, 0.0, kGyroBias}));
    const auto launched = estimator.update(
        sample(40 * kMillisecondNs, {6.0, kGravity, 0.0},
               {0.0, 0.0, kGyroBias}));
    assert(launched.launched);
    assert(std::abs(launched.corrected_angular_velocity.z) < 1e-12);

    const auto after_bias_only = estimator.update(
        sample(1'040 * kMillisecondNs, {6.0, kGravity, 0.0},
               {0.0, 0.0, kGyroBias}));
    assert(std::abs(after_bias_only.corrected_angular_velocity.z) < 1e-12);
    assert(std::abs(after_bias_only.pitch_rad) < 1e-3);
}

void test_launch_latches_and_integrates_body_rates_after_confirmation() {
    mosas::runtime::AttitudeEstimator estimator(test_config());
    estimator.update(sample(0, {0.0, kGravity, 0.0}));
    estimator.update(sample(20 * kMillisecondNs, {6.0, kGravity, 0.0}));
    const auto launched = estimator.update(
        sample(40 * kMillisecondNs, {6.0, kGravity, 0.0},
               {0.0, 0.0, 0.0}));

    assert(launched.launched);
    assert(std::abs(launched.pitch_rad) < 1e-12);

    const auto integrated = estimator.update(
        sample(50 * kMillisecondNs, {6.0, kGravity, 0.0},
               {0.0, 0.0, 1.0}));
    assert(integrated.pitch_rad > 0.005);
}

void prepare_launched_estimator(
    mosas::runtime::AttitudeEstimator* estimator) {
    assert(estimator != nullptr);
    estimator->update(sample(0, {0.0, kGravity, 0.0}));
    estimator->update(sample(20 * kMillisecondNs, {6.0, kGravity, 0.0}));
    const auto launched = estimator->update(
        sample(40 * kMillisecondNs, {6.0, kGravity, 0.0}));
    assert(launched.launched);
}

void test_launch_integrates_all_body_axes() {
    {
        mosas::runtime::AttitudeEstimator estimator(test_config());
        prepare_launched_estimator(&estimator);
        const auto estimate = estimator.update(
            sample(50 * kMillisecondNs, {6.0, kGravity, 0.0},
                   {1.0, 0.0, 0.0}));
        assert(std::abs(estimate.body_to_navigation.m21) > 0.005);
    }
    {
        mosas::runtime::AttitudeEstimator estimator(test_config());
        prepare_launched_estimator(&estimator);
        const auto estimate = estimator.update(
            sample(50 * kMillisecondNs, {6.0, kGravity, 0.0},
                   {0.0, 1.0, 0.0}));
        assert(std::abs(estimate.body_to_navigation.m20) > 0.005);
    }
    {
        mosas::runtime::AttitudeEstimator estimator(test_config());
        prepare_launched_estimator(&estimator);
        const auto estimate = estimator.update(
            sample(50 * kMillisecondNs, {6.0, kGravity, 0.0},
                   {0.0, 0.0, 1.0}));
        assert(estimate.pitch_rad > 0.005);
    }
}

void test_near_zero_acceleration_is_not_a_valid_prelaunch_attitude() {
    mosas::runtime::AttitudeEstimator estimator(test_config());
    estimator.update(sample(0, {0.0, kGravity, 0.0}));
    const auto estimate = estimator.update(
        sample(10 * kMillisecondNs, {0.0, 0.0, 0.0}));

    assert(!estimate.valid);
}

void test_invalid_dt_does_not_create_nan_attitude() {
    mosas::runtime::AttitudeEstimator estimator(test_config());
    estimator.update(sample(10, {0.0, kGravity, 0.0}));
    const auto estimate = estimator.update(
        sample(5, {0.0, kGravity, 0.0}, {0.0, 0.0, 1.0}));

    assert(estimate.valid);
    assert(std::isfinite(estimate.pitch_rad));
}

void test_launch_confirmation_does_not_cross_sample_gap() {
    mosas::runtime::AttitudeEstimator estimator(test_config());
    estimator.update(sample(0, {0.0, kGravity, 0.0}));
    estimator.update(sample(10 * kMillisecondNs, {6.0, kGravity, 0.0}));
    const auto after_gap = estimator.update(
        sample(3'000 * kMillisecondNs, {6.0, kGravity, 0.0}));

    assert(!after_gap.launched);
}

}  // namespace

int main() {
    test_level_specific_force_estimates_zero_pitch();
    test_gravity_estimates_fixed_pitch();
    test_stationary_tolerance_accepts_gravity_attitude();
    test_prelaunch_nonstationary_integrates_angular_velocity();
    test_x_axis_acceleration_requires_confirmation_duration();
    test_launch_thrust_is_not_used_as_gravity_pitch();
    test_constant_gyro_bias_is_removed_after_launch();
    test_launch_latches_and_integrates_body_rates_after_confirmation();
    test_launch_integrates_all_body_axes();
    test_near_zero_acceleration_is_not_a_valid_prelaunch_attitude();
    test_invalid_dt_does_not_create_nan_attitude();
    test_launch_confirmation_does_not_cross_sample_gap();
    return 0;
}
