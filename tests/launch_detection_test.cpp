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
 * @brief Feed a continuous 20 ms acceleration confirmation window.
 * @param estimator Initialized estimator. @param start_ms First pulse timestamp.
 */
void accelerate(AttitudeEstimator& estimator, std::int64_t start_ms = 5) {
    for (auto ms = start_ms; ms <= start_ms + 20; ms += 5) {
        update_at(estimator, ms, {58.85, 12.0, 3.0}, {1.0, 0.5, 0.8});
    }
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
    for (auto* estimator : {&acceleration, &two_stage}) {
        update_at(*estimator, 0, {0.0, kGravity, 0.0});
        for (const auto ms : {5, 10, 15, 20}) {
            CHECK(!update_at(*estimator, ms, {30.0, 0.0, 0.0}).launched);
        }
    }
    const auto triggered = update_at(acceleration, 25, {30.0, 0.0, 0.0});
    CHECK(triggered.launched && triggered.launch_timestamp_ns == 25 * kMs);
    CHECK_NEAR(triggered.speed_mps, 16.0, 1e-12);
    CHECK(!update_at(two_stage, 25, {30.0, 0.0, 0.0}).launched);
    CHECK(!update_at(two_stage, 30, {1.0, 0.0, 0.0}).launched);
    CHECK(update_at(two_stage, 40, {1.0, 0.0, 0.0}).launched);
    const auto next = update_at(acceleration, 30, {2.0, 0.0, 0.0});
    CHECK(next.launched && next.launch_timestamp_ns == 25 * kMs);
    CHECK_NEAR(next.speed_mps, 16.01, 1e-12);
    acceleration.reset();
    accelerate(acceleration, 35); // reset requires a fresh stationary baseline.
    CHECK(!update_at(acceleration, 60, {1.0, 0.0, 0.0}).launched);
    update_at(acceleration, 65, {0.0, kGravity, 0.0});
    accelerate(acceleration, 70);
    CHECK(update_at(acceleration, 95, {1.0, 0.0, 0.0}).launch_timestamp_ns == 90 * kMs);

    config.launch_confirm_ns = 90 * kMs;
    AttitudeEstimator longer_confirmation(config);
    update_at(longer_confirmation, 0, {0.0, kGravity, 0.0});
    for (auto ms = 5; ms < 95; ms += 5) {
        CHECK(!update_at(longer_confirmation, ms, {30.0, 0.0, 0.0}).launched);
    }
    CHECK(update_at(longer_confirmation, 95, {30.0, 0.0, 0.0}).launched);
    config.launch_detection_mode = static_cast<LaunchDetectionMode>(99);
    CHECK(!AttitudeEstimator::valid_config(config));
    return 0;
}

/** @brief Verify frozen rail attitude and the first post-departure integral. @return Zero on success. */
int verify_departure_and_integration() {
    auto config = two_stage_config();
    config.launch_speed_mps = 16.0;
    AttitudeEstimator estimator(config);
    constexpr double pitch = 0.4;
    const Vector3 bias{0.01, -0.02, 0.03};
    const auto stationary = update_at(estimator, 0,
        {kGravity * std::sin(pitch), kGravity * std::cos(pitch), 0.0}, bias);
    CHECK(stationary.valid);
    accelerate(estimator);
    for (const auto ms : {30, 35}) {
        const auto rail = update_at(estimator, ms, {58.85, 0.0, 0.0}, {2.0, 3.0, 4.0});
        CHECK(!rail.launched && rail.launch_timestamp_ns == -1);
        CHECK_NEAR(rail.speed_mps, 16.0, 1e-12);
        CHECK_NEAR(rail.pitch_rad, stationary.pitch_rad, 1e-12);
        CHECK_NEAR(rail.roll_rad, stationary.roll_rad, 1e-12);
    }
    CHECK(!update_at(estimator, 40, {1.0, 0.0, 0.0}).launched);
    CHECK(!update_at(estimator, 45, {1.0, 0.0, 0.0}).launched);
    // Waiting for release must not recalibrate even if force briefly resembles gravity.
    CHECK(!update_at(estimator, 50, {0.0, kGravity, 0.0}).launched);
    CHECK(!update_at(estimator, 55, {1.0, 0.0, 0.0}).launched);
    CHECK(!update_at(estimator, 60, {1.0, 0.0, 0.0}).launched);
    const auto released = update_at(estimator, 65, {1.0, 0.0, 0.0}, {1.0, 2.0, 3.0});
    CHECK(released.launched && released.launch_timestamp_ns == 65 * kMs);
    CHECK_NEAR(released.speed_mps, 16.0, 1e-12);
    CHECK_NEAR(released.pitch_rad, pitch, 1e-12);
    CHECK_NEAR(released.roll_rad, 0.0, 1e-12);
    CHECK_NEAR(released.corrected_angular_velocity.z, 3.0 - bias.z, 1e-12);
    const auto flight = update_at(estimator, 70, {2.0, 0.0, 0.0}, {bias.x, bias.y, bias.z + 1.0});
    CHECK_NEAR(flight.pitch_rad, pitch + 0.005, 1e-12);
    CHECK_NEAR(flight.speed_mps,
        16.0 + (2.0 - kGravity * std::sin(flight.pitch_rad)) * 0.005, 1e-12);
    update_at(estimator, 75, {std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0});
    const auto resumed = update_at(estimator, 80, {58.85, 0.0, 0.0}, {5.0, 5.0, 5.0});
    CHECK_NEAR(resumed.speed_mps, flight.speed_mps, 1e-12);
    CHECK_NEAR(resumed.pitch_rad, flight.pitch_rad, 1e-12);
    const auto gap = update_at(estimator, 170, {58.85, 0.0, 0.0}, {5.0, 5.0, 5.0});
    CHECK(gap.launched);
    CHECK_NEAR(gap.speed_mps, flight.speed_mps, 1e-12);
    CHECK_NEAR(gap.pitch_rad, flight.pitch_rad, 1e-12);
    return 0;
}

/** @brief Both modes reject isolated impacts, unsupported free fall and uncalibrated starts. @return Zero on success. */
int verify_false_triggers() {
    for (const auto mode : {LaunchDetectionMode::acceleration, LaunchDetectionMode::two_stage}) {
        AttitudeEstimatorConfig config;
        config.launch_detection_mode = mode;
        for (bool impact : {false, true}) {
            AttitudeEstimator estimator(config);
            update_at(estimator, 0, {0.0, kGravity, 0.0});
            if (impact) CHECK(!update_at(estimator, 5, {41.83, 7.0, 1.03}).launched);
            for (auto ms = 10; ms <= 100; ms += 5) {
                CHECK(!update_at(estimator, ms, {}).launched);
            }
        }
        AttitudeEstimator uncalibrated(config);
        accelerate(uncalibrated);
        for (auto ms = 30; ms <= 60; ms += 5) {
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
        update_at(estimator, 0, {0.0, kGravity, 0.0});
        accelerate(estimator);
        CHECK(!update_at(estimator, 30, {1.0, 0.0, 0.0}).launched);
        if (invalid) update_at(estimator, 35, {std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0});
        const auto start = invalid ? 40 : 80;
        for (auto ms = start; ms <= start + 20; ms += 5) {
            CHECK(!update_at(estimator, ms, {1.0, 0.0, 0.0}).launched);
        }
    }
    AttitudeEstimator estimator(two_stage_config());
    update_at(estimator, 0, {0.0, kGravity, 0.0});
    accelerate(estimator);
    for (auto ms = 30; ms <= 600; ms += 5) {
        CHECK(!update_at(estimator, ms, {58.85, 0.0, 0.0}).launched);
    }
    for (auto ms = 605; ms <= 625; ms += 5) {
        CHECK(!update_at(estimator, ms, {1.0, 0.0, 0.0}).launched);
    }
    update_at(estimator, 630, {0.0, kGravity, 0.0});
    accelerate(estimator, 635);
    CHECK(!update_at(estimator, 660, {1.0, 0.0, 0.0}).launched);
    CHECK(!update_at(estimator, 665, {1.0, 0.0, 0.0}).launched);
    CHECK(update_at(estimator, 670, {1.0, 0.0, 0.0}).launched);
    return 0;
}

/** @brief Verify low-force norm/duration boundaries and ignored non-increasing samples. @return Zero on success. */
int verify_confirmation_boundaries() {
    auto config = two_stage_config();
    config.launch_release_acceleration_max_mps2 = 2.0;
    config.launch_release_confirm_ns = 15 * kMs;
    config.launch_release_timeout_ns = 40 * kMs;
    config.maximum_sample_gap_ns = 6 * kMs;
    CHECK(AttitudeEstimator::valid_config(config));
    AttitudeEstimator estimator(config);
    update_at(estimator, 0, {0.0, kGravity, 0.0});
    accelerate(estimator);
    for (const auto ms : {30, 35}) {
        CHECK(!update_at(estimator, ms, {1.5, 1.5, 0.0}).launched);
    }
    for (const auto ms : {40, 45, 50}) {
        CHECK(!update_at(estimator, ms, {2.0, 0.0, 0.0}).launched);
    }
    CHECK(update_at(estimator, 50, {58.85, 0.0, 0.0}).timestamp_ns == 50 * kMs);
    CHECK(update_at(estimator, 45, {58.85, 0.0, 0.0}).timestamp_ns == 50 * kMs);
    const auto released = update_at(estimator, 55, {2.0, 0.0, 0.0});
    CHECK(released.launched && released.launch_timestamp_ns == 55 * kMs);
    estimator.reset();
    CHECK(!update_at(estimator, 60, {}).launched);
    CHECK(!update_at(estimator, 65, {}).valid);
    return 0;
}

/** @brief Replayed malformed samples cannot cancel current evidence or flight integration. @return Zero on success. */
int verify_replayed_invalid_samples() {
    auto config = two_stage_config();
    config.launch_speed_mps = 16.0;
    AttitudeEstimator estimator(config);
    update_at(estimator, 0, {0.0, kGravity, 0.0});
    accelerate(estimator);
    update_at(estimator, 30, {1.0, 0.0, 0.0});
    const auto invalid = std::numeric_limits<double>::quiet_NaN();
    for (const auto ms : {25, 30}) {
        const auto replay = update_at(estimator, ms, {invalid, 0.0, 0.0});
        CHECK(replay.timestamp_ns == 30 * kMs && !replay.launched);
    }
    const auto released = update_at(estimator, 40, {1.0, 0.0, 0.0});
    CHECK(released.launched && released.launch_timestamp_ns == 40 * kMs);
    update_at(estimator, 35, {}, {invalid, 0.0, 0.0});
    CHECK_NEAR(update_at(estimator, 45, {2.0, 0.0, 0.0}).speed_mps, 16.01, 1e-12);
    return 0;
}

/** @brief Both policies require fresh calibration after invalid/gapped acceleration evidence. @return Zero on success. */
int verify_acceleration_interruption_and_rearm() {
    for (const auto mode : {LaunchDetectionMode::acceleration, LaunchDetectionMode::two_stage}) {
        for (bool invalid : {false, true}) {
            AttitudeEstimatorConfig config;
            config.launch_detection_mode = mode;
            AttitudeEstimator estimator(config);
            update_at(estimator, 0, {0.0, kGravity, 0.0});
            update_at(estimator, 5, {58.85, 0.0, 0.0});
            if (invalid) update_at(estimator, 10, {}, {std::numeric_limits<double>::infinity(), 0.0, 0.0});
            const auto start = invalid ? 15 : 30;
            for (auto ms = start; ms <= start + 20; ms += 5) {
                CHECK(!update_at(estimator, ms, {58.85, 0.0, 0.0}).launched);
            }
            for (auto ms = 55; ms <= 70; ms += 5) {
                CHECK(!update_at(estimator, ms, {1.0, 0.0, 0.0}).launched);
            }
            update_at(estimator, 75, {0.0, kGravity, 0.0});
            accelerate(estimator, 80);
            CHECK(update_at(estimator, 105, {1.0, 0.0, 0.0}).launched ==
                  (mode == LaunchDetectionMode::acceleration));
            const auto triggered = update_at(estimator, 115, {1.0, 0.0, 0.0});
            CHECK(triggered.launched);
            CHECK(triggered.launch_timestamp_ns ==
                  (mode == LaunchDetectionMode::acceleration ? 100 : 115) * kMs);
            estimator.reset();
            accelerate(estimator, 120);
            CHECK(!update_at(estimator, 150, {1.0, 0.0, 0.0}).launched);
            update_at(estimator, 155, {0.0, kGravity, 0.0});
            accelerate(estimator, 160);
            update_at(estimator, 185, {1.0, 0.0, 0.0});
            const auto rearmed = update_at(estimator, 195, {1.0, 0.0, 0.0});
            CHECK(rearmed.launched);
            CHECK(rearmed.launch_timestamp_ns ==
                  (mode == LaunchDetectionMode::acceleration ? 180 : 195) * kMs);
        }
    }
    return 0;
}

/** @brief Confirmation may finish at the timeout, but not one nanosecond later. @return Zero on success. */
int verify_timeout_boundary() {
    for (const auto delay_ns : {0, 1}) {
        auto config = two_stage_config();
        config.launch_release_timeout_ns = 40 * kMs;
        AttitudeEstimator estimator(config);
        update_at(estimator, 0, {0.0, kGravity, 0.0});
        accelerate(estimator);
        for (auto ms = 30; ms <= 50; ms += 5) update_at(estimator, ms, {58.85, 0.0, 0.0});
        update_at(estimator, 55, {1.0, 0.0, 0.0});
        update_at(estimator, 60, {1.0, 0.0, 0.0});
        const auto result = estimator.update({65 * kMs + delay_ns, {1.0, 0.0, 0.0}, {}, 0.0, 0.0});
        CHECK(result.launched == (delay_ns == 0));
        CHECK(result.launch_timestamp_ns == (delay_ns == 0 ? 65 * kMs : -1));
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
