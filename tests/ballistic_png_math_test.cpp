#include <mosas/guidance/ballistic_png_math.hpp>
#include "test_support.hpp"
#include <cmath>
#include <limits>

int main() {
    using namespace mosas;
    const domain::DirectionNavigationUnit l{1, 0, 0};
    const domain::DirectionRateNavigationPerSec u{0, 0.02, -0.03};
    const auto value = guidance::evaluate_ballistic_png(l, u, 50, 7, 11, 9.80665);
    CHECK(value.valid);
    CHECK_NEAR(value.requested_overload.y, 7 * (50 * 0.02 / 9.80665 + 0.5), 1e-12);
    CHECK_NEAR(value.requested_overload.z, -11 * 50 * 0.03 / 9.80665, 1e-12);
    const auto falling = guidance::evaluate_ballistic_png(l, {0, -9.80665 / 100, 0}, 50, 3, 4, 9.80665);
    CHECK(falling.valid);
    CHECK_NEAR(falling.requested_overload.y, 0, 1e-12);
    CHECK_NEAR(falling.requested_overload.z, 0, 1e-12);
    const double rho = 0.8;
    const auto elevated = guidance::evaluate_ballistic_png({rho, 0.6, 0}, {-0.12, 0.16, 0.08}, 50, 2, 5, 9.80665);
    CHECK(elevated.valid);
    CHECK_NEAR(elevated.measured_elevation_rate_rad_s, 0.2, 1e-12);
    CHECK_NEAR(elevated.measured_lateral_rate_rad_s, 0.08, 1e-12);
    CHECK(elevated.requested_overload.z > 0);
    const auto pole = guidance::evaluate_ballistic_png({0, 1, 0}, {0.1, 0, 0}, 50, 3, 3, 9.80665);
    CHECK(!pole.valid);
    CHECK(pole.requested_overload.y == 0 && pole.requested_overload.z == 0);
    CHECK(!guidance::evaluate_ballistic_png(l, {1, 0, 0}, 50, 3, 3, 9.80665).valid);
    CHECK(!guidance::evaluate_ballistic_png({2, 0, 0}, u, 50, 3, 3, 9.80665).valid);
    CHECK(!guidance::evaluate_ballistic_png(l, u, 0, 3, 3, 9.80665).valid);
    CHECK(!guidance::evaluate_ballistic_png(l, u, 50, 0, 3, 9.80665).valid);
    CHECK(!guidance::evaluate_ballistic_png(l, {0, std::numeric_limits<double>::infinity(), 0}, 50, 3, 3, 9.80665).valid);
    return 0;
}
