#include <mosas/runtime/attitude_estimator.hpp>

#include "test_support.hpp"

#include <cmath>
#include <limits>

namespace {

using mosas::runtime::AttitudeEstimate;
using mosas::runtime::AttitudeEstimator;
using mosas::runtime::AttitudeEstimatorConfig;
using mosas::runtime::LaunchDetectionMode;

constexpr double kGravity = 9.80665;
constexpr std::int64_t kMs = 1'000'000;

/** @brief Select the optional two-stage policy explicitly. @return Two-stage defaults. */
AttitudeEstimatorConfig two_stage_config() {
    AttitudeEstimatorConfig config;
    config.launch_detection_mode = LaunchDetectionMode::two_stage;
    return config;
}

/**
 * @brief Feed one IMU sample at a millisecond timestamp.
 * @param estimator Estimator under test. @param ms Sample time in milliseconds.
 * @param acceleration Specific force in m/s². @param gyro Body rate in rad/s.
 * @return Updated attitude and trigger state.
 */
AttitudeEstimate update_at(AttitudeEstimator& estimator, std::int64_t ms,
                          Vector3 acceleration, Vector3 gyro = {}) {
    return estimator.update({ms * kMs, acceleration, gyro, 0.0, 0.0});
}

/**
 * @brief Supply the complete stationary window before testing trigger evidence.
 * @param estimator Estimator under test. @param start_ms First stationary timestamp.
 * @param acceleration Static specific force. @param gyro Static gyro bias.
 * @return Last calibration timestamp in milliseconds.
 */
std::int64_t calibrate(AttitudeEstimator& estimator, std::int64_t start_ms = 0,
                       Vector3 acceleration = {0.0, kGravity, 0.0}, Vector3 gyro = {}) {
    for (auto ms = start_ms; ms <= start_ms + 200; ms += 5)
        update_at(estimator, ms, acceleration, gyro);
    return start_ms + 200;
}

/**
 * @brief Feed a continuous 20 ms acceleration confirmation window.
 * @param estimator Initialized estimator. @param start_ms First pulse timestamp.
 */
void accelerate(AttitudeEstimator& estimator, std::int64_t start_ms) {
    for (auto ms = start_ms; ms <= start_ms + 20; ms += 5)
        update_at(estimator, ms, {58.85, 12.0, 3.0}, {1.0, 0.5, 0.8});
}

/** @brief Verify selectable trigger times and configurable single-stage duration. @return Zero on success. */
int verify_modes_and_default() {
    AttitudeEstimatorConfig config;
    CHECK(config.launch_detection_mode == LaunchDetectionMode::acceleration);
    config.launch_speed_mps = 16.0;
    AttitudeEstimator acceleration(config);
    auto release_config = config;
    release_config.launch_detection_mode = LaunchDetectionMode::two_stage;
    AttitudeEstimator two_stage(release_config);
    constexpr std::int64_t t = 200;
    for (auto* estimator : {&acceleration, &two_stage}) {
        CHECK(calibrate(*estimator) == t);
        for (const auto ms : {5, 10, 15, 20})
            CHECK(!update_at(*estimator, t + ms, {30.0, 0.0, 0.0}).launched);
    }
    const auto triggered = update_at(acceleration, t + 25, {30.0, 0.0, 0.0});
    CHECK(triggered.launched && triggered.launch_timestamp_ns == (t + 25) * kMs);
    CHECK_NEAR(triggered.speed_mps, 16.0, 1e-12);
    CHECK(!update_at(two_stage, t + 25, {30.0, 0.0, 0.0}).launched);
    CHECK(!update_at(two_stage, t + 30, {1.0, 0.0, 0.0}).launched);
    CHECK(update_at(two_stage, t + 40, {1.0, 0.0, 0.0}).launched);
    CHECK_NEAR(update_at(acceleration, t + 30, {2.0, 0.0, 0.0}).speed_mps, 16.01, 1e-12);
    acceleration.reset();
    accelerate(acceleration, 0);
    CHECK(!update_at(acceleration, 25, {1.0, 0.0, 0.0}).launched);
    const auto rearmed = calibrate(acceleration, 30);
    accelerate(acceleration, rearmed + 5);
    CHECK(update_at(acceleration, rearmed + 30, {}).launch_timestamp_ns == (rearmed + 25) * kMs);

    config.launch_confirm_ns = 90 * kMs;
    AttitudeEstimator longer_confirmation(config);
    calibrate(longer_confirmation);
    for (auto ms = t + 5; ms < t + 95; ms += 5)
        CHECK(!update_at(longer_confirmation, ms, {30.0, 0.0, 0.0}).launched);
    CHECK(update_at(longer_confirmation, t + 95, {30.0, 0.0, 0.0}).launched);
    config.launch_detection_mode = static_cast<LaunchDetectionMode>(99);
    CHECK(!AttitudeEstimator::valid_config(config));
    return 0;
}

/** @brief Verify frozen rail attitude and the first post-departure integral. @return Zero on success. */
int verify_departure_and_integration() {
    auto config = two_stage_config(); config.launch_speed_mps = 16.0;
    AttitudeEstimator estimator(config);
    constexpr double pitch = 0.4;
    const Vector3 bias{0.01, -0.02, 0.03};
    const auto t = calibrate(estimator, 0,
        {kGravity * std::sin(pitch), kGravity * std::cos(pitch), 0.0}, bias);
    accelerate(estimator, t + 5);
    for (const auto ms : {30, 35}) {
        const auto rail = update_at(estimator, t + ms, {58.85, 0.0, 0.0}, {2.0, 3.0, 4.0});
        CHECK(!rail.launched && rail.launch_timestamp_ns == -1);
        CHECK_NEAR(rail.speed_mps, 16.0, 1e-12);
        CHECK_NEAR(rail.pitch_rad, pitch, 1e-12);
        CHECK_NEAR(rail.roll_rad, 0.0, 1e-12);
    }
    CHECK(!update_at(estimator, t + 40, {1.0, 0.0, 0.0}).launched);
    CHECK(!update_at(estimator, t + 45, {1.0, 0.0, 0.0}).launched);
    CHECK(!update_at(estimator, t + 50, {0.0, kGravity, 0.0}).launched);
    CHECK(!update_at(estimator, t + 55, {1.0, 0.0, 0.0}).launched);
    CHECK(!update_at(estimator, t + 60, {1.0, 0.0, 0.0}).launched);
    const auto released = update_at(estimator, t + 65, {1.0, 0.0, 0.0}, {1.0, 2.0, 3.0});
    CHECK(released.launched && released.launch_timestamp_ns == (t + 65) * kMs);
    CHECK_NEAR(released.speed_mps, 16.0, 1e-12);
    CHECK_NEAR(released.pitch_rad, pitch, 1e-12);
    CHECK_NEAR(released.corrected_angular_velocity.z, 3.0 - bias.z, 1e-12);
    const auto flight = update_at(estimator, t + 70, {2.0, 0.0, 0.0}, {bias.x, bias.y, bias.z + 1.0});
    CHECK_NEAR(flight.pitch_rad, pitch + 0.005, 1e-12);
    CHECK_NEAR(flight.speed_mps, 16.0 + (2.0 - kGravity * std::sin(flight.pitch_rad)) * 0.005, 1e-12);
    update_at(estimator, t + 75, {std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0});
    for (const auto ms : {80, 170}) {
        const auto resumed = update_at(estimator, t + ms, {58.85, 0.0, 0.0}, {5.0, 5.0, 5.0});
        CHECK(resumed.launched);
        CHECK_NEAR(resumed.speed_mps, flight.speed_mps, 1e-12);
        CHECK_NEAR(resumed.pitch_rad, flight.pitch_rad, 1e-12);
    }
    return 0;
}

/** @brief Both modes reject isolated impacts, unsupported free fall and uncalibrated starts. @return Zero on success. */
int verify_false_triggers() {
    for (const auto mode : {LaunchDetectionMode::acceleration, LaunchDetectionMode::two_stage}) {
        AttitudeEstimatorConfig config; config.launch_detection_mode = mode;
        for (bool impact : {false, true}) {
            AttitudeEstimator estimator(config);
            const auto t = calibrate(estimator);
            if (impact) CHECK(!update_at(estimator, t + 5, {41.83, 7.0, 1.03}).launched);
            for (auto ms = t + 10; ms <= t + 100; ms += 5)
                CHECK(!update_at(estimator, ms, {}).launched);
        }
        AttitudeEstimator uncalibrated(config);
        accelerate(uncalibrated, 0);
        for (auto ms = 25; ms <= 60; ms += 5) {
            const auto state = update_at(uncalibrated, ms, {1.0, 0.0, 0.0});
            CHECK(!state.launched && !state.valid);
        }
    }
    return 0;
}

/** @brief Broken release evidence and timed-out acceleration cannot authorize a later fall. @return Zero on success. */
int verify_discontinuities_and_timeout() {
    for (bool invalid : {false, true}) {
        AttitudeEstimator estimator(two_stage_config());
        const auto t = calibrate(estimator);
        accelerate(estimator, t + 5);
        CHECK(!update_at(estimator, t + 30, {1.0, 0.0, 0.0}).launched);
        if (invalid) update_at(estimator, t + 35, {std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0});
        const auto start = t + (invalid ? 40 : 80);
        for (auto ms = start; ms <= start + 20; ms += 5)
            CHECK(!update_at(estimator, ms, {1.0, 0.0, 0.0}).launched);
    }
    AttitudeEstimator estimator(two_stage_config());
    const auto t = calibrate(estimator);
    accelerate(estimator, t + 5);
    for (auto ms = t + 30; ms <= t + 600; ms += 5)
        CHECK(!update_at(estimator, ms, {58.85, 0.0, 0.0}).launched);
    for (auto ms = t + 605; ms <= t + 625; ms += 5)
        CHECK(!update_at(estimator, ms, {1.0, 0.0, 0.0}).launched);
    const auto rearmed = calibrate(estimator, t + 630);
    accelerate(estimator, rearmed + 5);
    CHECK(!update_at(estimator, rearmed + 30, {1.0, 0.0, 0.0}).launched);
    CHECK(!update_at(estimator, rearmed + 35, {1.0, 0.0, 0.0}).launched);
    CHECK(update_at(estimator, rearmed + 40, {1.0, 0.0, 0.0}).launched);
    return 0;
}

/** @brief Verify low-force boundaries and ignored non-increasing samples. @return Zero on success. */
int verify_confirmation_boundaries() {
    auto config = two_stage_config();
    config.launch_release_acceleration_max_mps2 = 2.0;
    config.launch_release_confirm_ns = 15 * kMs;
    config.launch_release_timeout_ns = 40 * kMs;
    config.maximum_sample_gap_ns = 6 * kMs;
    CHECK(AttitudeEstimator::valid_config(config));
    AttitudeEstimator estimator(config);
    const auto t = calibrate(estimator);
    accelerate(estimator, t + 5);
    for (const auto ms : {30, 35})
        CHECK(!update_at(estimator, t + ms, {1.5, 1.5, 0.0}).launched);
    for (const auto ms : {40, 45, 50})
        CHECK(!update_at(estimator, t + ms, {2.0, 0.0, 0.0}).launched);
    CHECK(update_at(estimator, t + 50, {58.85, 0.0, 0.0}).timestamp_ns == (t + 50) * kMs);
    CHECK(update_at(estimator, t + 45, {58.85, 0.0, 0.0}).timestamp_ns == (t + 50) * kMs);
    const auto released = update_at(estimator, t + 55, {2.0, 0.0, 0.0});
    CHECK(released.launched && released.launch_timestamp_ns == (t + 55) * kMs);
    estimator.reset();
    CHECK(!update_at(estimator, 0, {}).launched);
    CHECK(!update_at(estimator, 5, {}).valid);
    return 0;
}

/** @brief Replayed malformed samples cannot cancel current evidence or flight integration. @return Zero on success. */
int verify_replayed_invalid_samples() {
    auto config = two_stage_config(); config.launch_speed_mps = 16.0;
    AttitudeEstimator estimator(config);
    const auto t = calibrate(estimator);
    accelerate(estimator, t + 5);
    update_at(estimator, t + 30, {1.0, 0.0, 0.0});
    const auto invalid = std::numeric_limits<double>::quiet_NaN();
    for (const auto ms : {25, 30}) {
        const auto replay = update_at(estimator, t + ms, {invalid, 0.0, 0.0});
        CHECK(replay.timestamp_ns == (t + 30) * kMs && !replay.launched);
    }
    const auto released = update_at(estimator, t + 40, {1.0, 0.0, 0.0});
    CHECK(released.launched && released.launch_timestamp_ns == (t + 40) * kMs);
    update_at(estimator, t + 35, {}, {invalid, 0.0, 0.0});
    CHECK_NEAR(update_at(estimator, t + 45, {2.0, 0.0, 0.0}).speed_mps, 16.01, 1e-12);
    return 0;
}

/** @brief Both policies require a complete fresh calibration after invalid/gapped evidence. @return Zero on success. */
int verify_acceleration_interruption_and_rearm() {
    for (const auto mode : {LaunchDetectionMode::acceleration, LaunchDetectionMode::two_stage}) {
        for (bool invalid : {false, true}) {
            AttitudeEstimatorConfig config; config.launch_detection_mode = mode;
            AttitudeEstimator estimator(config);
            const auto t = calibrate(estimator);
            update_at(estimator, t + 5, {58.85, 0.0, 0.0});
            if (invalid) update_at(estimator, t + 10, {}, {std::numeric_limits<double>::infinity(), 0.0, 0.0});
            const auto start = t + (invalid ? 15 : 30);
            for (auto ms = start; ms <= start + 20; ms += 5)
                CHECK(!update_at(estimator, ms, {58.85, 0.0, 0.0}).launched);
            for (auto ms = t + 55; ms <= t + 70; ms += 5)
                CHECK(!update_at(estimator, ms, {1.0, 0.0, 0.0}).launched);
            // One isolated static sample must not rearm the trigger.
            update_at(estimator, t + 75, {0.0, kGravity, 0.0});
            accelerate(estimator, t + 80);
            CHECK(!update_at(estimator, t + 105, {1.0, 0.0, 0.0}).launched);
            const auto rearmed = calibrate(estimator, t + 110);
            accelerate(estimator, rearmed + 5);
            CHECK(update_at(estimator, rearmed + 30, {1.0, 0.0, 0.0}).launched ==
                  (mode == LaunchDetectionMode::acceleration));
            const auto triggered = update_at(estimator, rearmed + 40, {1.0, 0.0, 0.0});
            CHECK(triggered.launched);
            CHECK(triggered.launch_timestamp_ns ==
                  (rearmed + (mode == LaunchDetectionMode::acceleration ? 25 : 40)) * kMs);
            estimator.reset();
            accelerate(estimator, 0);
            CHECK(!update_at(estimator, 25, {1.0, 0.0, 0.0}).launched);
        }
    }
    return 0;
}

/** @brief Confirmation may finish at the timeout, but not one nanosecond later. @return Zero on success. */
int verify_timeout_boundary() {
    for (const auto delay_ns : {0, 1}) {
        auto config = two_stage_config(); config.launch_release_timeout_ns = 40 * kMs;
        AttitudeEstimator estimator(config);
        const auto t = calibrate(estimator);
        accelerate(estimator, t + 5);
        for (auto ms = t + 30; ms <= t + 50; ms += 5)
            update_at(estimator, ms, {58.85, 0.0, 0.0});
        update_at(estimator, t + 55, {1.0, 0.0, 0.0});
        update_at(estimator, t + 60, {1.0, 0.0, 0.0});
        const auto result = estimator.update({(t + 65) * kMs + delay_ns, {1.0, 0.0, 0.0}, {}, 0.0, 0.0});
        CHECK(result.launched == (delay_ns == 0));
        CHECK(result.launch_timestamp_ns == (delay_ns == 0 ? (t + 65) * kMs : -1));
    }
    return 0;
}

}  // namespace

int main() {
    CHECK(verify_modes_and_default() == 0);
    CHECK(verify_departure_and_integration() == 0);
    CHECK(verify_false_triggers() == 0);
    CHECK(verify_discontinuities_and_timeout() == 0);
    CHECK(verify_confirmation_boundaries() == 0);
    CHECK(verify_replayed_invalid_samples() == 0);
    CHECK(verify_acceleration_interruption_and_rearm() == 0);
    CHECK(verify_timeout_boundary() == 0);
    return 0;
}
