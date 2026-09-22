#include <mosas/guidance/guidance_estimator.hpp>

#include <cassert>
#include <cmath>
#include <iostream>

namespace {

void test_two_state_kalman_filter_constant_velocity() {
    LineOfSightRateFilterConfig config{};
    config.sigma_a = 8.0;
    config.sigma_q = 0.0035;
    config.enable_filter = true;

    LineOfSightRateEstimator estimator(config);

    const double true_rate = 0.5;  // rad/s
    const double dt = 0.01;        // 10ms
    double q_b = 0.1;
    const Vector3 zero_gyro{0.0, 0.0, 0.0};

    // First frame initializes
    auto out = estimator.update({true, q_b, 0.0}, zero_gyro, dt);
    assert(!out.valid);

    // Run for 50 frames (0.5s)
    for (int i = 1; i <= 50; ++i) {
        q_b += true_rate * dt;
        out = estimator.update({true, q_b, 0.0}, zero_gyro, dt);
        assert(out.valid);
    }

    // After 50 frames, estimated rate should converge to true_rate
    assert(std::abs(out.q_y - true_rate) < 0.05);
}

void test_two_state_kalman_filter_gyro_feedforward() {
    // 目标在惯性系中静止 (true_rate = 0)，弹体转动产生 c_omega
    LineOfSightRateFilterConfig config{};
    config.sigma_a = 8.0;
    config.sigma_q = 0.0035;
    config.enable_filter = true;

    LineOfSightRateEstimator estimator(config);

    const double dt = 0.01;
    // 镖体以 0.5 rad/s 绕 y 轴转动 (偏航方向产生视线旋转)
    const Vector3 body_gyro{0.0, 0.5, 0.0};
    double q_z = 0.0;

    estimator.update({true, 0.0, q_z}, body_gyro, dt);

    LineOfSightAngularVelocity out{};
    for (int i = 1; i <= 30; ++i) {
        // 由于弹体转动，相机中观测到的视线角按 c_omega 反向移动
        // c_omega_z = -gyro_y = -0.5 => q_z 随时间变化
        q_z += 0.5 * dt;
        out = estimator.update({true, 0.0, q_z}, body_gyro, dt);
    }

    assert(out.valid);
    // 真实惯性视线角速度应保持为 0 附近
    assert(std::abs(out.q_z) < 0.05);
}

void test_two_state_kalman_filter_dead_reckoning_during_loss() {
    LineOfSightRateFilterConfig config{};
    config.sigma_a = 8.0;
    config.sigma_q = 0.0035;
    config.enable_filter = true;

    LineOfSightRateEstimator estimator(config);

    const double dt = 0.01;
    const double true_rate = 0.4;
    double q_y = 0.0;
    const Vector3 zero_gyro{0.0, 0.0, 0.0};

    // 建立稳定跟踪
    estimator.update({true, q_y, 0.0}, zero_gyro, dt);
    LineOfSightAngularVelocity out{};
    for (int i = 1; i <= 30; ++i) {
        q_y += true_rate * dt;
        out = estimator.update({true, q_y, 0.0}, zero_gyro, dt);
    }
    assert(out.valid);
    assert(std::abs(out.q_y - true_rate) < 0.05);

    // 丢失 10 帧 (valid = false)
    for (int i = 0; i < 10; ++i) {
        out = estimator.update({false, 0.0, 0.0}, zero_gyro, dt);
        // 在丢失期间保持惯导外推输出，不会断档为 0
        assert(out.valid);
        assert(std::abs(out.q_y - true_rate) < 0.1);
    }

    // 重捕
    q_y += true_rate * dt * 10;
    out = estimator.update({true, q_y, 0.0}, zero_gyro, dt);
    assert(out.valid);
    assert(std::abs(out.q_y - true_rate) < 0.08);
}

void test_los_rate_filter_config_defaults_and_overrides() {
    LineOfSightRateFilterConfig config{};
    assert(config.enable_filter == true);
    assert(std::abs(config.sigma_a - 8.0) < 1e-9);
    assert(std::abs(config.sigma_q - 0.0035) < 1e-9);
    assert(std::abs(config.sigma_a_pitch - 0.0) < 1e-9);
    assert(std::abs(config.sigma_a_yaw - 0.0) < 1e-9);

    config.sigma_a_pitch = 10.0;
    config.sigma_a_yaw = 3.5;
    LineOfSightRateEstimator estimator(config);
    // Should construct successfully
}

}  // namespace

int main() {
    test_two_state_kalman_filter_constant_velocity();
    test_two_state_kalman_filter_gyro_feedforward();
    test_two_state_kalman_filter_dead_reckoning_during_loss();
    test_los_rate_filter_config_defaults_and_overrides();
    std::cout << "All LOS Kalman Filter tests passed!" << std::endl;
    return 0;
}
