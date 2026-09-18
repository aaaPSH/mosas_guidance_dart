#include <mosas/guidance/guidance_estimator.hpp>

#include <cassert>
#include <cmath>

namespace {

Vector3 cross_product(const Vector3& lhs, const Vector3& rhs) {
    return {lhs.y * rhs.z - lhs.z * rhs.y,
            lhs.z * rhs.x - lhs.x * rhs.z,
            lhs.x * rhs.y - lhs.y * rhs.x};
}

Vector3 normalize(const Vector3& value) {
    const double norm = std::hypot(std::hypot(value.x, value.y), value.z);
    return {value.x / norm, value.y / norm, value.z / norm};
}

LineOfSight line_of_sight_from_unit(const Vector3& unit) {
    return {true, std::atan2(unit.y, std::hypot(unit.x, unit.z)),
            std::atan2(unit.z, unit.x)};
}

void test_stationary_target_body_rotation_is_compensated() {
    LineOfSightRateEstimator estimator({0.01, 0.25, 1.0, false});
    assert(!estimator.update({true, 0.0, 0.0}, {0.0, 1.0, 0.0}, 0.01)
                 .valid);
    const auto yaw_rate =
        estimator.update({true, 0.0, 0.01}, {0.0, 1.0, 0.0}, 0.01);
    assert(yaw_rate.valid);
    assert(std::abs(yaw_rate.q_y) < 1e-9);
    assert(std::abs(yaw_rate.q_z) < 1e-9);
}

void test_png_uses_scalar_speed_and_body_frame() {
    const LineOfSight line_of_sight{true, 0.0, 0.0};
    const LineOfSightAngularVelocity angular_velocity{true, 0.0, 0.1};
    const auto output = PngGuidance::calculate(line_of_sight, angular_velocity,
                                               100.0);
    assert(output.valid);
    assert(output.navigation_acceleration.x == output.body_acceleration.x);
    assert(output.navigation_acceleration.y == output.body_acceleration.y);
    assert(output.navigation_acceleration.z == output.body_acceleration.z);
}

void test_all_body_axes_are_compensated_for_stationary_target() {
    const LineOfSight initial{true, 0.2, -0.3};
    const double dt = 1e-4;
    const Vector3 body_angular_velocity{0.4, -0.7, 0.9};
    const Vector3 initial_unit{
        std::cos(initial.q_y) * std::cos(initial.q_z),
        std::sin(initial.q_y),
        std::cos(initial.q_y) * std::sin(initial.q_z)};
    const Vector3 body_frame_derivative =
        cross_product(body_angular_velocity, initial_unit);
    const Vector3 next_unit = normalize(
        {initial_unit.x - body_frame_derivative.x * dt,
         initial_unit.y - body_frame_derivative.y * dt,
         initial_unit.z - body_frame_derivative.z * dt});
    const LineOfSight next = line_of_sight_from_unit(next_unit);

    LineOfSightRateEstimator estimator({0.01, 0.25, 1.0, false});
    assert(!estimator.update(initial, body_angular_velocity, dt).valid);
    const auto compensated =
        estimator.update(next, body_angular_velocity, dt);
    assert(compensated.valid);
    assert(std::abs(compensated.q_y) < 2e-4);
    assert(std::abs(compensated.q_z) < 2e-4);
}

}  // namespace

int main() {
    test_stationary_target_body_rotation_is_compensated();
    test_png_uses_scalar_speed_and_body_frame();
    test_all_body_axes_are_compensated_for_stationary_target();
    return 0;
}
