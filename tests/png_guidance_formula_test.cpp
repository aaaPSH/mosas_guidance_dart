#include <mosas/guidance/guidance_estimator.hpp>

#include <cassert>
#include <cmath>
#include <limits>

namespace {

PngGuidanceConfig test_config() {
    PngGuidanceConfig config;
    config.navigation_constant_y = 2.0;
    config.navigation_constant_z = 3.0;
    config.gravity = 10.0;
    config.maximum_command_overload = 100.0;
    config.maximum_command_acceleration =
        std::numeric_limits<double>::infinity();
    return config;
}

void test_decoupled_png_overload_matches_figure() {
    const auto output = PngGuidance::calculate(
        {true, 0.0, 0.0}, {true, 0.2, 0.1}, 10.0, 0.6, test_config());
    assert(output.valid);

    const double cos_pitch = std::cos(0.6);
    const double expected_y = 2.0 * 0.2 * 10.0 / 10.0 + cos_pitch;
    const double expected_z = 3.0 * 0.1 * 10.0 / 10.0 * cos_pitch;
    assert(std::abs(output.body_overload.y - expected_y) < 1e-12);
    assert(std::abs(output.body_overload.z - expected_z) < 1e-12);
}

void test_y_and_z_navigation_constants_are_independent() {
    auto config = test_config();
    const auto first = PngGuidance::calculate(
        {true, 0.0, 0.0}, {true, 0.2, 0.1}, 10.0, 0.6, config);
    config.navigation_constant_y = 7.0;
    const auto changed_y = PngGuidance::calculate(
        {true, 0.0, 0.0}, {true, 0.2, 0.1}, 10.0, 0.6, config);
    assert(std::abs(first.body_overload.z - changed_y.body_overload.z) <
           1e-12);
    config.navigation_constant_y = 2.0;
    config.navigation_constant_z = 8.0;
    const auto changed_z = PngGuidance::calculate(
        {true, 0.0, 0.0}, {true, 0.2, 0.1}, 10.0, 0.6, config);
    assert(std::abs(first.body_overload.y - changed_z.body_overload.y) <
           1e-12);
    assert(changed_z.body_overload.z > 0.0);
}

void test_lateral_gravity_factor_vanishes_at_ninety_degree_pitch() {
    auto config = test_config();
    const auto output = PngGuidance::calculate(
        {true, 0.0, 0.0}, {true, 0.2, 0.1}, 10.0,
        0.5 * 3.14159265358979323846, config);
    assert(output.valid);
    assert(std::abs(output.body_overload.z) < 1e-12);
}

}  // namespace

int main() {
    test_decoupled_png_overload_matches_figure();
    test_y_and_z_navigation_constants_are_independent();
    test_lateral_gravity_factor_vanishes_at_ninety_degree_pitch();
    return 0;
}
