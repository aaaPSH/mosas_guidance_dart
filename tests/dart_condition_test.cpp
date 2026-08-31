#include "dart_condition.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {

constexpr double kGravity = 9.80665;
constexpr double kTolerance = 1e-6;

void expect_true(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(EXIT_FAILURE);
    }
}

void expect_false(bool condition, const char* message) {
    expect_true(!condition, message);
}

void expect_near(double actual, double expected, const char* message,
                 double tolerance = kTolerance) {
    if (std::abs(actual - expected) > tolerance) {
        std::cerr << "FAIL: " << message << ": actual=" << actual
                  << ", expected=" << expected << '\n';
        std::exit(EXIT_FAILURE);
    }
}

void expect_same_attitude(const EulerAngles& actual, const EulerAngles& expected,
                          const char* message) {
    expect_near(actual.roll, expected.roll, message);
    expect_near(actual.pitch, expected.pitch, message);
    expect_near(actual.yaw, expected.yaw, message);
}

Vector3 gravity_for_attitude(double roll, double pitch) {
    const double horizontal_gravity = kGravity * std::cos(pitch);
    return {
        -kGravity * std::sin(pitch),
        horizontal_gravity * std::sin(roll),
        horizontal_gravity * std::cos(roll),
    };
}

void test_uninitialized_and_ready_do_not_integrate() {
    DartCondition estimator;

    const EulerAngles before_initialization =
        estimator.update({0.0, 0.0, 0.0}, {1.0, 2.0, 3.0}, 1.0);
    expect_same_attitude(before_initialization, {0.0, 0.0, 0.0},
                         "未初始化时不应积分角速度");

    expect_true(estimator.initialize({0.0, 0.0, kGravity}),
                "有效重力向量应初始化成功");
    const EulerAngles before_launch =
        estimator.update({0.0, 0.0, 0.0}, {1.0, 2.0, 3.0}, 1.0);
    expect_same_attitude(before_launch, {0.0, 0.0, 0.0},
                         "发射前不应积分角速度");
    expect_true(estimator.initialized(), "初始化后应标记为已初始化");
    expect_false(estimator.flying(), "初始化后但发射前不应处于飞行状态");
}

void test_initialize_from_gravity() {
    constexpr double expected_roll = 0.3;
    constexpr double expected_pitch = -0.2;
    DartCondition estimator;

    expect_true(estimator.initialize(
                    gravity_for_attitude(expected_roll, expected_pitch)),
                "倾斜重力向量应初始化成功");
    const EulerAngles result = estimator.attitude();
    expect_near(result.roll, expected_roll, "重力初始化 roll 不正确");
    expect_near(result.pitch, expected_pitch, "重力初始化 pitch 不正确");
    expect_near(result.yaw, 0.0, "初始化 yaw 应为零");
}

void test_launch_requires_initialization() {
    DartCondition estimator;
    estimator.launch();
    expect_false(estimator.flying(), "未初始化时发射指令应无效");

    expect_true(estimator.initialize({0.0, 0.0, kGravity}),
                "发射测试初始化应成功");
    estimator.launch();
    expect_true(estimator.flying(), "初始化后发射指令应进入飞行状态");
}

void test_flight_integrates_each_axis() {
    {
        DartCondition estimator;
        estimator.initialize({0.0, 0.0, kGravity});
        estimator.launch();
        for (int i = 0; i < 100; ++i) {
            estimator.update({0.0, 0.0, 0.0}, {0.1, 0.0, 0.0}, 0.01);
        }
        const EulerAngles result = estimator.attitude();
        expect_near(result.roll, 0.1, "绕 x 轴角速度应传播到 roll");
        expect_near(result.pitch, 0.0, "绕 x 轴角速度不应产生 pitch");
        expect_near(result.yaw, 0.0, "绕 x 轴角速度不应产生 yaw");
    }

    {
        DartCondition estimator;
        estimator.initialize({0.0, 0.0, kGravity});
        estimator.launch();
        for (int i = 0; i < 100; ++i) {
            estimator.update({0.0, 0.0, 0.0}, {0.0, 0.1, 0.0}, 0.01);
        }
        const EulerAngles result = estimator.attitude();
        expect_near(result.roll, 0.0, "绕 y 轴角速度不应产生 roll");
        expect_near(result.pitch, 0.1, "绕 y 轴角速度应传播到 pitch");
        expect_near(result.yaw, 0.0, "绕 y 轴角速度不应产生 yaw");
    }

    {
        DartCondition estimator;
        estimator.initialize({0.0, 0.0, kGravity});
        estimator.launch();
        for (int i = 0; i < 100; ++i) {
            estimator.update({0.0, 0.0, 0.0}, {0.0, 0.0, 0.1}, 0.01);
        }
        const EulerAngles result = estimator.attitude();
        expect_near(result.roll, 0.0, "绕 z 轴角速度不应产生 roll");
        expect_near(result.pitch, 0.0, "绕 z 轴角速度不应产生 pitch");
        expect_near(result.yaw, 0.1, "绕 z 轴角速度应传播到 yaw");
    }
}

void test_flight_ignores_acceleration() {
    DartCondition zero_acceleration;
    DartCondition large_acceleration;
    zero_acceleration.initialize({0.0, 0.0, kGravity});
    large_acceleration.initialize({0.0, 0.0, kGravity});
    zero_acceleration.launch();
    large_acceleration.launch();

    for (int i = 0; i < 50; ++i) {
        zero_acceleration.update({0.0, 0.0, 0.0}, {0.2, -0.1, 0.3}, 0.01);
        large_acceleration.update({1000000.0, -2000000.0, 3000000.0},
                                  {0.2, -0.1, 0.3}, 0.01);
    }

    expect_same_attitude(large_acceleration.attitude(),
                         zero_acceleration.attitude(),
                         "飞行阶段不应使用加速度计校正姿态");
}

void test_invalid_inputs_preserve_attitude() {
    DartCondition estimator;
    expect_false(estimator.initialize({0.0, 0.0, 0.0}),
                 "零加速度向量不应初始化成功");
    expect_false(estimator.initialize({NAN, 0.0, kGravity}),
                 "非有限加速度向量不应初始化成功");

    estimator.initialize({0.0, 0.0, kGravity});
    estimator.launch();
    estimator.update({0.0, 0.0, 0.0}, {0.1, 0.2, 0.3}, 0.1);
    const EulerAngles saved = estimator.attitude();

    expect_same_attitude(
        estimator.update({0.0, 0.0, 0.0}, {0.1, 0.2, 0.3}, 0.0), saved,
        "零时间步长不应改变姿态");
    expect_same_attitude(
        estimator.update({0.0, 0.0, 0.0}, {0.1, 0.2, 0.3}, -0.1), saved,
        "负时间步长不应改变姿态");
    expect_same_attitude(
        estimator.update({0.0, 0.0, 0.0}, {NAN, 0.2, 0.3}, 0.1), saved,
        "非有限角速度不应改变姿态");
    expect_same_attitude(
        estimator.update({NAN, INFINITY, 0.0}, {0.1, 0.2, 0.3}, NAN), saved,
        "非有限时间步长不应改变姿态");
}

void test_reset_clears_state() {
    DartCondition estimator;
    estimator.initialize({0.0, 0.0, kGravity});
    estimator.launch();
    estimator.update({0.0, 0.0, 0.0}, {0.1, 0.2, 0.3}, 0.1);
    estimator.reset();

    expect_false(estimator.initialized(), "reset 后应清除初始化状态");
    expect_false(estimator.flying(), "reset 后应退出飞行状态");
    expect_same_attitude(estimator.attitude(), {0.0, 0.0, 0.0},
                         "reset 后姿态应归零");
}

}  // namespace

int main() {
    test_uninitialized_and_ready_do_not_integrate();
    test_initialize_from_gravity();
    test_launch_requires_initialization();
    test_flight_integrates_each_axis();
    test_flight_ignores_acceleration();
    test_invalid_inputs_preserve_attitude();
    test_reset_clears_state();

    std::cout << "all tests passed\n";
    return EXIT_SUCCESS;
}
