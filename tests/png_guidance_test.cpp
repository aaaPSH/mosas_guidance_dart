#include <mosas/guidance/guidance_estimator.hpp>

#include <cassert>
#include <cmath>

namespace {

constexpr double kGravity = 9.80665;
constexpr double kPi = 3.14159265358979323846;
constexpr double kTolerance = 1e-9;

void expect_near(double actual, double expected) {
    assert(std::abs(actual - expected) <= kTolerance);
}

void test_axis_aligned_velocity_decouples_pitch_and_yaw_overload() {
    const LineOfSight line_of_sight{true, 0.0, 0.0};
    const LineOfSightAngularVelocity angular_velocity{true, 0.1, 0.2};
    const Vector3 velocity{100.0, 0.0, 0.0};
    const EulerAngles attitude{0.0, 0.0, 0.0};
    const PngGuidanceConfig config{3.0, kGravity};

    const PngGuidanceOutput output = PngGuidance::calculate(
        line_of_sight, angular_velocity, velocity, attitude, config);

    assert(output.valid);
    expect_near(output.navigation_acceleration.x, 0.0);
    expect_near(output.navigation_acceleration.y, kGravity + 30.0);
    expect_near(output.navigation_acceleration.z, -60.0);
    expect_near(output.body_overload.y, 1.0 + 30.0 / kGravity);
    expect_near(output.body_overload.z, -60.0 / kGravity);
    expect_near(output.command_overload,
                std::hypot(1.0 + 30.0 / kGravity, -60.0 / kGravity));
    expect_near(output.command_phase,
                std::atan2(-60.0 / kGravity, 1.0 + 30.0 / kGravity));
}

void test_oblique_velocity_uses_speed_and_ballistic_pitch_compensation() {
    const LineOfSight line_of_sight{true, 0.0, 0.0};
    const LineOfSightAngularVelocity angular_velocity{true, 0.1, 0.1};
    const Vector3 velocity{80.0, 60.0, 0.0};
    const EulerAngles attitude{0.0, 0.0, 0.0};
    const PngGuidanceConfig config{3.0, kGravity};

    const PngGuidanceOutput output = PngGuidance::calculate(
        line_of_sight, angular_velocity, velocity, attitude, config);

    assert(output.valid);
    // v=100 m/s，cos(theta)=80/100，两个通道分别独立计算。
    expect_near(output.navigation_acceleration.x, 0.0);
    expect_near(output.navigation_acceleration.y, 0.8 * kGravity + 30.0);
    expect_near(output.navigation_acceleration.z, -24.0);
    expect_near(output.command_overload,
                std::hypot(0.8 + 30.0 / kGravity, -24.0 / kGravity));
    expect_near(output.command_phase,
                std::atan2(-24.0 / kGravity, 0.8 + 30.0 / kGravity));
}

void test_navigation_acceleration_is_transformed_to_body_overload() {
    const LineOfSight line_of_sight{true, 0.0, 0.0};
    const LineOfSightAngularVelocity angular_velocity{true, 0.1, 0.0};
    const Vector3 velocity{100.0, 0.0, 0.0};
    // 绕弹体 +z 轴俯仰 90°：弹体系 +x 轴映射到导航系 +y 轴。
    const EulerAngles attitude{kPi / 2.0, 0.0, 0.0};
    const PngGuidanceConfig config{3.0, kGravity};

    const PngGuidanceOutput output = PngGuidance::calculate(
        line_of_sight, angular_velocity, velocity, attitude, config);

    assert(output.valid);
    expect_near(output.navigation_acceleration.y, kGravity + 30.0);
    expect_near(output.body_acceleration.x, kGravity + 30.0);
    expect_near(output.body_overload.x, 1.0 + 30.0 / kGravity);
    expect_near(output.body_overload.y, 0.0);
    expect_near(output.command_overload, 1.0 + 30.0 / kGravity);
    expect_near(output.command_phase, 0.0);
}

void test_zero_velocity_returns_invalid_output() {
    const LineOfSight line_of_sight{true, 0.0, 0.0};
    const LineOfSightAngularVelocity angular_velocity{true, 0.1, 0.2};
    const Vector3 velocity{0.0, 0.0, 0.0};
    const EulerAngles attitude{0.0, 0.0, 0.0};

    const PngGuidanceOutput output = PngGuidance::calculate(
        line_of_sight, angular_velocity, velocity, attitude);

    assert(!output.valid);
    expect_near(output.command_overload, 0.0);
    expect_near(output.command_phase, 0.0);
}

}  // namespace

int main() {
    test_axis_aligned_velocity_decouples_pitch_and_yaw_overload();
    test_oblique_velocity_uses_speed_and_ballistic_pitch_compensation();
    test_navigation_acceleration_is_transformed_to_body_overload();
    test_zero_velocity_returns_invalid_output();
    return 0;
}
