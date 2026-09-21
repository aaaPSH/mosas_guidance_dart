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
    config.acceleration_filter_alpha = 1.0;
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

void test_prelaunch_does_not_integrate_angular_velocity() {
    mosas::runtime::AttitudeEstimator estimator(test_config());
    estimator.update(sample(1, {0.0, kGravity, 0.0}, {0.0, 0.0, 1.0}));
    const auto estimate = estimator.update(
        sample(1 + 100 * kMillisecondNs, {0.0, kGravity, 0.0},
               {0.0, 0.0, 1.0}));

    assert(estimate.valid);
    assert(!estimate.launched);
    assert(std::abs(estimate.pitch_rad) < 1e-12);
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
    test_prelaunch_does_not_integrate_angular_velocity();
    test_x_axis_acceleration_requires_confirmation_duration();
    test_launch_latches_and_integrates_body_rates_after_confirmation();
    test_launch_integrates_all_body_axes();
    test_near_zero_acceleration_is_not_a_valid_prelaunch_attitude();
    test_invalid_dt_does_not_create_nan_attitude();
    test_launch_confirmation_does_not_cross_sample_gap();
    return 0;
}
