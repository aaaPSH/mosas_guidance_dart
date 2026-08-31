#include "guidance_estimator.hpp"

#include <cmath>

namespace {

bool is_finite(double value) {
    return std::isfinite(value);
}

LineOfSight invalid_line_of_sight() {
    return {false, 0.0, 0.0};
}

}  // namespace

LineOfSight GuidanceEstimator::calculate(
    const VisionResult& result, const CameraIntrinsics& intrinsics) {
    if (!result.found || !is_finite(result.blob.center_x) ||
        !is_finite(result.blob.center_y) || !is_finite(intrinsics.fx) ||
        !is_finite(intrinsics.fy) || !is_finite(intrinsics.cx) ||
        !is_finite(intrinsics.cy) || intrinsics.fx <= 0.0 ||
        intrinsics.fy <= 0.0) {
        return invalid_line_of_sight();
    }

    const double right =
        (result.blob.center_x - intrinsics.cx) / intrinsics.fx;
    const double up =
        -(result.blob.center_y - intrinsics.cy) / intrinsics.fy;
    if (!is_finite(right) || !is_finite(up)) {
        return invalid_line_of_sight();
    }

    const double q_y = std::atan2(up, std::hypot(1.0, right));
    const double q_z = std::atan2(right, 1.0);
    if (!is_finite(q_y) || !is_finite(q_z)) {
        return invalid_line_of_sight();
    }
    return {true, q_y, q_z};
}
