#include <mosas/runtime/attitude_estimator.hpp>
#include <mosas/runtime/attitude_history.hpp>
#include <mosas/runtime/legacy_flight_state_adapter.hpp>

#include "test_support.hpp"

#include <limits>

namespace {

using namespace mosas::runtime;
constexpr TimestampNs kMs = 1'000'000;
constexpr double kGravity = 9.80665;

/** @param pitch 俯仰角，单位 rad。 @return 无滚转时的静止比力，单位 m/s²。 */
Vector3 gravity_at(double pitch) {
    return {kGravity * std::sin(pitch), kGravity * std::cos(pitch), 0.0};
}

/** @brief 验证两种模式的首次姿态、零偏和基线必须等待完整静止窗口。 @return 通过时为 0。 */
int verify_stationary_persistence() {
    for (const auto mode : {LaunchDetectionMode::acceleration, LaunchDetectionMode::two_stage}) {
        AttitudeEstimatorConfig config;
        config.launch_detection_mode = mode;
        AttitudeEstimator estimator(config);
        for (TimestampNs ms = 0; ms < 200; ms += 5) {
            const auto state = estimator.update({ms * kMs, gravity_at(0.6), {}, 0.0, 0.0});
            CHECK(!state.valid && !state.launched);
            CHECK_NEAR(state.pitch_rad, 0.0, 1e-12);
        }
        CHECK(!estimator.update({200 * kMs - 1, gravity_at(0.6), {}, 0.0, 0.0}).valid);
        const auto aligned = estimator.update({200 * kMs, gravity_at(0.6), {}, 0.0, 0.0});
        CHECK(aligned.valid);
        CHECK_NEAR(aligned.pitch_rad, 0.6, 1e-12);
        estimator.reset();
        for (TimestampNs ms = 0; ms <= 20; ms += 5) {
            CHECK(!estimator.update({ms * kMs, gravity_at(0.6), {}, 0.0, 0.0}).valid);
        }
        for (TimestampNs ms = 25; ms <= 60; ms += 5) {
            const auto state = estimator.update({ms * kMs, {30.0, 0.0, 0.0}, {}, 0.0, 0.0});
            CHECK(!state.valid && !state.launched);
        }
    }
    return 0;
}

/** @brief 验证 EMA 在矢量域求均值，扰动后重新确认且不继承旧 EMA。 @return 通过时为 0。 */
int verify_filtered_lock() {
    for (const auto mode : {LaunchDetectionMode::acceleration, LaunchDetectionMode::two_stage}) {
        AttitudeEstimatorConfig config;
        config.launch_detection_mode = mode;
        config.accel_filter_alpha = 0.5;
        AttitudeEstimator estimator(config);
        for (TimestampNs ms = 0; ms < 200; ms += 5)
            estimator.update({ms * kMs, gravity_at(0.4), {}, 0.0, 0.0});
        const auto aligned = estimator.update({200 * kMs, gravity_at(0.8), {}, 0.0, 0.0});
        CHECK(aligned.valid);
        CHECK_NEAR(aligned.pitch_rad, 0.6, 1e-12);
        // 原始角速度模长超限，即使比力模长仍为 g，也必须立即冻结。
        const auto disturbed = estimator.update({205 * kMs, gravity_at(0.9), {0.04, 0.04, 0.0}, 0.0, 0.0});
        CHECK_NEAR(disturbed.pitch_rad, aligned.pitch_rad, 1e-12);
        for (TimestampNs ms = 210; ms < 410; ms += 5) {
            const auto pending = estimator.update({ms * kMs, gravity_at(0.9), {}, 0.0, 0.0});
            CHECK(pending.valid);
            CHECK_NEAR(pending.pitch_rad, aligned.pitch_rad, 1e-12);
        }
        CHECK_NEAR(estimator.update({410 * kMs, gravity_at(0.9), {}, 0.0, 0.0}).pitch_rad, 0.9, 1e-12);
        const auto impulse = estimator.update({415 * kMs, {0.0, kGravity + 1.0, 0.0}, {}, 0.0, 0.0});
        CHECK_NEAR(impulse.pitch_rad, 0.9, 1e-12);
        for (TimestampNs ms = 420; ms < 620; ms += 5)
            CHECK_NEAR(estimator.update({ms * kMs, gravity_at(0.4), {}, 0.0, 0.0}).pitch_rad, 0.9, 1e-12);
        CHECK_NEAR(estimator.update({620 * kMs, gravity_at(0.4), {}, 0.0, 0.0}).pitch_rad, 0.4, 1e-12);

        // 已锁定后无效样本、断流同样要求新的完整窗口，但不清除姿态。
        estimator.update({625 * kMs, {}, {std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0}, 0.0, 0.0});
        for (TimestampNs ms = 630; ms < 830; ms += 5)
            CHECK_NEAR(estimator.update({ms * kMs, gravity_at(0.7), {}, 0.0, 0.0}).pitch_rad, 0.4, 1e-12);
        CHECK_NEAR(estimator.update({830 * kMs, gravity_at(0.7), {}, 0.0, 0.0}).pitch_rad, 0.7, 1e-12);
        for (TimestampNs ms = 900; ms < 1100; ms += 5)
            CHECK_NEAR(estimator.update({ms * kMs, gravity_at(0.2), {}, 0.0, 0.0}).pitch_rad, 0.7, 1e-12);
        CHECK_NEAR(estimator.update({1100 * kMs, gravity_at(0.2), {}, 0.0, 0.0}).pitch_rad, 0.2, 1e-12);
    }
    return 0;
}

/** @brief 未锁定时验证冲击、超限转动、无效样本和断流均中断确认。 @return 通过时为 0。 */
int verify_interrupted_calibration() {
    for (int interruption = 0; interruption < 4; ++interruption) {
        AttitudeEstimator estimator;
        for (TimestampNs ms = 0; ms < 200; ms += 5)
            CHECK(!estimator.update({ms * kMs, gravity_at(0.4), {}, 0.0, 0.0}).valid);
        TimestampNs start_ms = 205;
        if (interruption == 0)
            CHECK(!estimator.update({200 * kMs, gravity_at(0.8), {0.0, 0.0, 0.051}, 0.0, 0.0}).valid);
        else if (interruption == 1)
            CHECK(!estimator.update({200 * kMs, {0.0, kGravity + 1.0, 0.0}, {}, 0.0, 0.0}).valid);
        else if (interruption == 2)
            CHECK(!estimator.update({200 * kMs, {}, {std::numeric_limits<double>::infinity(), 0.0, 0.0}, 0.0, 0.0}).valid);
        else
            start_ms = 230;
        for (TimestampNs ms = start_ms; ms < start_ms + 200; ms += 5)
            CHECK(!estimator.update({ms * kMs, gravity_at(0.8), {}, 0.0, 0.0}).valid);
        const auto aligned = estimator.update({(start_ms + 200) * kMs, gravity_at(0.8), {}, 0.0, 0.0});
        CHECK(aligned.valid);
        CHECK_NEAR(aligned.pitch_rad, 0.8, 1e-12);
    }
    // 重复/回退的损坏样本不属于当前连续段，不能清除已有证据。
    AttitudeEstimator estimator;
    for (TimestampNs ms = 0; ms < 200; ms += 5)
        estimator.update({ms * kMs, gravity_at(0.6), {}, 0.0, 0.0});
    for (const auto ms : {190, 195})
        estimator.update({ms * kMs, {std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0}, {}, 0.0, 0.0});
    CHECK(estimator.update({200 * kMs, gravity_at(0.6), {}, 0.0, 0.0}).valid);
    return 0;
}

/** @brief 验证配置范围和延长确认窗，禁止用零时长绕过静止确认。 @return 通过时为 0。 */
int verify_calibration_config() {
    AttitudeEstimatorConfig config;
    CHECK_NEAR(config.gyro_stationary_threshold_rad_s, 0.05, 1e-12);
    for (const auto duration : {-1, 0, 199'999'999}) {
        config.stationary_confirm_ns = duration;
        CHECK(!AttitudeEstimator::valid_config(config));
    }
    config.stationary_confirm_ns = 300 * kMs;
    for (const auto alpha : {0.0, -0.1, 1.01, std::numeric_limits<double>::quiet_NaN()}) {
        config.accel_filter_alpha = alpha;
        CHECK(!AttitudeEstimator::valid_config(config));
    }
    config.accel_filter_alpha = 1.0;
    CHECK(AttitudeEstimator::valid_config(config));
    AttitudeEstimator estimator(config);
    for (TimestampNs ms = 0; ms < 300; ms += 5)
        CHECK(!estimator.update({ms * kMs, gravity_at(0.6), {}, 0.0, 0.0}).valid);
    CHECK(estimator.update({300 * kMs, gravity_at(0.6), {}, 0.0, 0.0}).valid);
    return 0;
}

/** @brief 验证适配器的确认窗口、触发帧冻结和触发后积分及异常保护。 @return 通过时为 0。 */
int verify_adapter_and_integration() {
    AttitudeEstimatorConfig config;
    config.launch_detection_mode = LaunchDetectionMode::two_stage;
    config.launch_speed_mps = 16.0;
    LegacyFlightStateEstimatorAdapter adapter(config);
    const auto sample_at = [](TimestampNs ms, Vector3 acceleration, Vector3 gyro = {}) {
        mosas::domain::ImuKinematicSample sample{};
        sample.time.monotonic_ns = ms * kMs;
        sample.time.source_domain = mosas::domain::ClockDomain::monotonic;
        sample.time.source = mosas::domain::TimestampSource::imu_uart_receive_estimate;
        sample.time.quality = mosas::domain::TimestampQuality::estimated;
        sample.acceleration = {acceleration.x, acceleration.y, acceleration.z};
        sample.angular_velocity = {gyro.x, gyro.y, gyro.z};
        return sample;
    };
    for (TimestampNs ms = 0; ms < 200; ms += 5)
        CHECK(!adapter.update(sample_at(ms, gravity_at(0.4))).attitude.valid);
    const auto aligned = adapter.update(sample_at(200, gravity_at(0.4)));
    CHECK(aligned.attitude.valid);
    CHECK_NEAR(aligned.attitude.pitch_rad.value, 0.4, 1e-12);
    for (const auto ms : {210, 230})
        CHECK(adapter.update(sample_at(ms, {30.0, 0.0, 0.0})).launch.phase == mosas::domain::LaunchPhase::pre_launch);
    adapter.update(sample_at(240, {1.0, 0.0, 0.0}));
    const auto launched = adapter.update(sample_at(250, {1.0, 0.0, 0.0}, {0.0, 0.0, 1.0}));
    CHECK(launched.launch.phase == mosas::domain::LaunchPhase::launched);
    CHECK(launched.launch.reason == mosas::domain::LaunchReason::threshold_confirmed);
    CHECK(launched.launch.transition_time.monotonic_ns == 250 * kMs);
    CHECK_NEAR(launched.attitude.pitch_rad.value, 0.4, 1e-12);
    CHECK_NEAR(launched.speed_mps.value, 16.0, 1e-12);
    const auto flight = adapter.update(sample_at(260, {2.0, 0.0, 0.0}, {0.0, 0.0, 1.0}));
    CHECK_NEAR(flight.attitude.pitch_rad.value, 0.41, 1e-12);
    CHECK_NEAR(flight.speed_mps.value, 16.0 + (2.0 - kGravity * std::sin(0.41)) * 0.01, 1e-12);
    const auto balanced = adapter.update(sample_at(270, {kGravity * std::sin(0.41), 0.0, 0.0}));
    CHECK_NEAR(balanced.speed_mps.value, flight.speed_mps.value, 1e-12);
    CHECK_NEAR(adapter.update(sample_at(260, {100.0, 0.0, 0.0})).speed_mps.value, flight.speed_mps.value, 1e-12);
    adapter.update(sample_at(280, {std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0}));
    CHECK_NEAR(adapter.update(sample_at(290, {100.0, 0.0, 0.0})).speed_mps.value, flight.speed_mps.value, 1e-12);
    CHECK_NEAR(adapter.update(sample_at(500, {100.0, 0.0, 0.0})).speed_mps.value, flight.speed_mps.value, 1e-12);
    adapter.reset();
    const auto reset = adapter.update(sample_at(0, {0.0, 0.0, kGravity}));
    CHECK(!reset.attitude.valid);
    CHECK_NEAR(reset.speed_mps.value, 16.0, 1e-12);
    for (TimestampNs ms = 5; ms <= 200; ms += 5)
        adapter.update(sample_at(ms, {0.0, 0.0, kGravity}));
    CHECK_NEAR(adapter.update(sample_at(205, {0.0, 0.0, kGravity})).attitude.roll_rad.value, -1.5707963267948966, 1e-12);
    return 0;
}

/** @brief 验证速度随同一姿态历史插值且不跨代际。 @return 通过时为 0。 */
int verify_history() {
    AttitudeHistory history(4);
    AttitudeSample sample;
    sample.timestamp_ns = 10 * kMs;
    sample.launched = true;
    sample.launch_timestamp_ns = 0;
    sample.speed_mps = 16.0;
    CHECK(history.push(sample));
    sample.timestamp_ns = 30 * kMs;
    sample.speed_mps = 14.0;
    CHECK(history.push(sample));
    const auto exact = history.find_interpolated(10 * kMs);
    CHECK(exact && !exact->interpolated);
    CHECK_NEAR(exact->speed_mps, 16.0, 1e-12);
    const auto middle = history.find_interpolated(15 * kMs);
    CHECK(middle && middle->interpolated);
    CHECK_NEAR(middle->speed_mps, 15.5, 1e-12);
    CHECK(!history.find_interpolated(40 * kMs));
    sample.timestamp_ns = 40 * kMs;
    sample.launch_generation = 1;
    CHECK(history.push(sample));
    CHECK(!history.find_interpolated(35 * kMs));
    sample.timestamp_ns = 50 * kMs;
    sample.speed_mps = std::numeric_limits<double>::infinity();
    CHECK(!history.push(sample));
    history.reset();
    CHECK(!history.find_interpolated(10 * kMs));
    return 0;
}

}  // namespace

int main() {
    CHECK(verify_stationary_persistence() == 0);
    CHECK(verify_filtered_lock() == 0);
    CHECK(verify_interrupted_calibration() == 0);
    CHECK(verify_calibration_config() == 0);
    CHECK(verify_adapter_and_integration() == 0);
    CHECK(verify_history() == 0);
    return 0;
}
