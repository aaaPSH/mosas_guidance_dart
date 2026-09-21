#include <mosas/runtime/attitude_estimator.hpp>
#include <mosas/runtime/attitude_history.hpp>

#include <cassert>

namespace {

using mosas::runtime::AttitudeEstimator;
using mosas::runtime::AttitudeEstimatorConfig;
using mosas::runtime::AttitudeHistory;
using mosas::runtime::AttitudeSample;
using mosas::runtime::ImuAngularVelocitySample;

ImuAngularVelocitySample imu_sample(std::int64_t timestamp_ns,
                                    double acceleration_x) {
    ImuAngularVelocitySample sample{};
    sample.timestamp_ns = timestamp_ns;
    sample.acceleration = {acceleration_x, 0.0, 9.80665};
    return sample;
}

void test_launch_timestamp_is_confirmation_sample_time() {
    AttitudeEstimatorConfig config;
    config.launch_ax_threshold_mps2 = 5.0;
    config.launch_confirm_ns = 20;
    AttitudeEstimator estimator(config);

    assert(!estimator.update(imu_sample(0, 0.0)).launched);
    assert(!estimator.update(imu_sample(10, 6.0)).launched);
    const auto launched = estimator.update(imu_sample(30, 6.0));

    assert(launched.launched);
    assert(launched.launch_timestamp_ns == 30);
}

AttitudeSample history_sample(std::int64_t timestamp_ns,
                              std::uint64_t launch_generation,
                              std::int64_t launch_timestamp_ns) {
    AttitudeSample sample{};
    sample.timestamp_ns = timestamp_ns;
    sample.body_to_navigation = {1.0, 0.0, 0.0,
                                 0.0, 1.0, 0.0,
                                 0.0, 0.0, 1.0};
    sample.launch_generation = launch_generation;
    sample.launch_timestamp_ns = launch_timestamp_ns;
    return sample;
}

void test_history_never_interpolates_across_launch_generations() {
    AttitudeHistory history(8);
    assert(history.push(history_sample(10, 0, -1)));
    assert(history.push(history_sample(20, 1, -1)));
    assert(!history.find_interpolated(15).has_value());

    assert(history.push(history_sample(30, 1, -1)));
    const auto lookup = history.find_interpolated(25);
    assert(lookup.has_value());
    assert(lookup->launch_generation == 1);
    assert(lookup->launch_timestamp_ns == -1);
}

void test_exact_history_lookup_preserves_launch_metadata() {
    AttitudeHistory history(4);
    auto launched = history_sample(100, 3, 90);
    launched.launched = true;
    assert(history.push(launched));

    const auto lookup = history.find_interpolated(100);
    assert(lookup.has_value());
    assert(lookup->launched);
    assert(lookup->launch_generation == 3);
    assert(lookup->launch_timestamp_ns == 90);
}

}  // namespace

int main() {
    test_launch_timestamp_is_confirmation_sample_time();
    test_history_never_interpolates_across_launch_generations();
    test_exact_history_lookup_preserves_launch_metadata();
    return 0;
}
