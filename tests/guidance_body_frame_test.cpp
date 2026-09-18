#include <mosas/guidance/guidance_estimator.hpp>

#include <cassert>
#include <cmath>

namespace {

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

}  // namespace

int main() {
    test_stationary_target_body_rotation_is_compensated();
    test_png_uses_scalar_speed_and_body_frame();
    return 0;
}
