#include <mosas/guidance/ballistic_png_math.hpp>
#include <algorithm>
#include <cmath>

namespace mosas::guidance {
PngCalculationTrace evaluate_ballistic_png(
    const domain::DirectionNavigationUnit& l,
    const domain::DirectionRateNavigationPerSec& u,
    double speed, double ny, double nz, double gravity) noexcept {
    PngCalculationTrace result;
    if (!std::isfinite(speed) || speed <= 0 || !std::isfinite(ny) || ny <= 0 ||
        !std::isfinite(nz) || nz <= 0 || !std::isfinite(gravity) || gravity <= 0) return result;
    result.invalid_reason = domain::GuidanceInvalidReason::invalid_los;
    const double norm = std::hypot(std::hypot(l.x, l.y), l.z);
    const double rate_norm = std::hypot(std::hypot(u.x, u.y), u.z);
    const double radial = l.x * u.x + l.y * u.y + l.z * u.z;
    if (!std::isfinite(norm) || std::abs(norm - 1) > 1e-6 ||
        !std::isfinite(rate_norm) || !std::isfinite(radial) ||
        std::abs(radial) > 1e-6 * (1 + rate_norm)) return result;
    const double rho = std::hypot(l.x, l.z);
    result.invalid_reason = domain::GuidanceInvalidReason::invalid_rate;
    if (rho < 1e-6) return result;
    result.inertial_los_elevation_rad = std::asin(std::clamp(l.y, -1.0, 1.0));
    result.measured_elevation_rate_rad_s = u.y / rho;
    result.measured_lateral_rate_rad_s = (l.x * u.z - l.z * u.x) / rho;
    result.parabolic_elevation_rate_rad_s = -gravity * rho / (2 * speed);
    result.compensated_elevation_rate_rad_s = result.measured_elevation_rate_rad_s - result.parabolic_elevation_rate_rad_s;
    const double y = speed / gravity * ny * result.compensated_elevation_rate_rad_s;
    const double z = speed / gravity * nz * result.measured_lateral_rate_rad_s;
    if (!std::isfinite(y) || !std::isfinite(z)) return PngCalculationTrace{};
    result.requested_overload = {0, y, z};
    result.invalid_reason = domain::GuidanceInvalidReason::none;
    result.valid = true;
    return result;
}
}
