#include "guidance_estimator.hpp"

#include <cmath>

namespace {

constexpr double kEpsilon = 1e-12;
constexpr double kRotationTolerance = 1e-6;

bool is_finite(double value) {
    return std::isfinite(value);
}

LineOfSight invalid_line_of_sight() {
    return {false, 0.0, 0.0};
}

bool is_finite(const EulerAngles& value) {
    return is_finite(value.roll) && is_finite(value.pitch) &&
           is_finite(value.yaw);
}

bool is_finite(const RotationMatrix3& value) {
    return is_finite(value.m00) && is_finite(value.m01) &&
           is_finite(value.m02) && is_finite(value.m10) &&
           is_finite(value.m11) && is_finite(value.m12) &&
           is_finite(value.m20) && is_finite(value.m21) &&
           is_finite(value.m22);
}

double vector_norm(const Vector3& value);

bool is_rotation_matrix(const RotationMatrix3& value) {
    if (!is_finite(value)) {
        return false;
    }

    const Vector3 column_0{value.m00, value.m10, value.m20};
    const Vector3 column_1{value.m01, value.m11, value.m21};
    const Vector3 column_2{value.m02, value.m12, value.m22};
    const double determinant =
        value.m00 * (value.m11 * value.m22 - value.m12 * value.m21) -
        value.m01 * (value.m10 * value.m22 - value.m12 * value.m20) +
        value.m02 * (value.m10 * value.m21 - value.m11 * value.m20);
    return std::abs(vector_norm(column_0) - 1.0) <= kRotationTolerance &&
           std::abs(vector_norm(column_1) - 1.0) <= kRotationTolerance &&
           std::abs(vector_norm(column_2) - 1.0) <= kRotationTolerance &&
           std::abs(column_0.x * column_1.x + column_0.y * column_1.y +
                    column_0.z * column_1.z) <= kRotationTolerance &&
           std::abs(column_0.x * column_2.x + column_0.y * column_2.y +
                    column_0.z * column_2.z) <= kRotationTolerance &&
           std::abs(column_1.x * column_2.x + column_1.y * column_2.y +
                    column_1.z * column_2.z) <= kRotationTolerance &&
           std::abs(determinant - 1.0) <= kRotationTolerance;
}

bool is_valid_intrinsics(const CameraIntrinsics& intrinsics) {
    return is_finite(intrinsics.fx) && is_finite(intrinsics.fy) &&
           is_finite(intrinsics.cx) && is_finite(intrinsics.cy) &&
           intrinsics.fx > 0.0 && intrinsics.fy > 0.0;
}

bool is_valid_vector(const Vector3& value) {
    return is_finite(value.x) && is_finite(value.y) && is_finite(value.z);
}

double vector_norm(const Vector3& value) {
    return std::hypot(std::hypot(value.x, value.y), value.z);
}

Vector3 normalize(const Vector3& value) {
    const double norm = vector_norm(value);
    if (!is_finite(norm) || norm < kEpsilon) {
        return {0.0, 0.0, 0.0};
    }
    return {value.x / norm, value.y / norm, value.z / norm};
}

Vector3 multiply(const RotationMatrix3& matrix, const Vector3& value) {
    return {
        matrix.m00 * value.x + matrix.m01 * value.y + matrix.m02 * value.z,
        matrix.m10 * value.x + matrix.m11 * value.y + matrix.m12 * value.z,
        matrix.m20 * value.x + matrix.m21 * value.y + matrix.m22 * value.z,
    };
}

RotationMatrix3 identity_rotation() {
    return {1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0};
}

RotationMatrix3 rotation_from_euler(const EulerAngles& angles) {
    const double cr = std::cos(angles.roll);
    const double sr = std::sin(angles.roll);
    const double cp = std::cos(angles.pitch);
    const double sp = std::sin(angles.pitch);
    const double cy = std::cos(angles.yaw);
    const double sy = std::sin(angles.yaw);

    // YZX 欧拉角对应的弹体坐标系到地面坐标系旋转矩阵。
    return {
        cy * cp, -cy * sp * cr + sy * sr, cy * sp * sr + sy * cr,
        sp,      cp * cr,                 -cp * sr,
        -sy * cp, sy * sp * cr + cy * sr, -sy * sp * sr + cy * cr,
    };
}

bool camera_direction(const VisionResult& result,
                      const CameraIntrinsics& intrinsics,
                      Vector3* direction) {
    if (direction == nullptr || !result.found ||
        !is_finite(result.blob.center_x) ||
        !is_finite(result.blob.center_y) ||
        !is_valid_intrinsics(intrinsics)) {
        return false;
    }

    const double right =
        (result.blob.center_x - intrinsics.cx) / intrinsics.fx;
    const double up =
        -(result.blob.center_y - intrinsics.cy) / intrinsics.fy;
    const Vector3 unnormalized{1.0, up, right};
    if (!is_valid_vector(unnormalized) ||
        vector_norm(unnormalized) < kEpsilon) {
        return false;
    }

    *direction = normalize(unnormalized);
    return is_valid_vector(*direction) && vector_norm(*direction) >= kEpsilon;
}

LineOfSight angles_from_direction(const Vector3& direction) {
    if (!is_valid_vector(direction)) {
        return invalid_line_of_sight();
    }

    const double norm = vector_norm(direction);
    if (!is_finite(norm) || norm < kEpsilon) {
        return invalid_line_of_sight();
    }

    const double q_y = std::atan2(direction.y,
                                  std::hypot(direction.x, direction.z));
    const double q_z = std::atan2(direction.z, direction.x);
    if (!is_finite(q_y) || !is_finite(q_z)) {
        return invalid_line_of_sight();
    }
    return {true, q_y, q_z};
}

}  // namespace

LineOfSight GuidanceEstimator::calculate(
    const VisionResult& result, const CameraIntrinsics& intrinsics) {
    Vector3 direction{};
    if (!camera_direction(result, intrinsics, &direction)) {
        return invalid_line_of_sight();
    }
    return angles_from_direction(direction);
}

LineOfSight GuidanceEstimator::calculate_compensated(
    const VisionResult& result, const CameraIntrinsics& intrinsics,
    const EulerAngles& attitude) {
    return calculate_compensated(result, intrinsics, attitude,
                                  identity_rotation());
}

LineOfSight GuidanceEstimator::calculate_compensated(
    const VisionResult& result, const CameraIntrinsics& intrinsics,
    const EulerAngles& attitude, const RotationMatrix3& camera_to_body) {
    Vector3 camera_direction_value{};
    if (!is_finite(attitude) || !is_rotation_matrix(camera_to_body) ||
        !camera_direction(result, intrinsics, &camera_direction_value)) {
        return invalid_line_of_sight();
    }

    const Vector3 body_direction =
        multiply(camera_to_body, camera_direction_value);
    if (!is_valid_vector(body_direction) ||
        vector_norm(body_direction) < kEpsilon) {
        return invalid_line_of_sight();
    }

    const RotationMatrix3 body_to_navigation = rotation_from_euler(attitude);
    const Vector3 navigation_direction =
        multiply(body_to_navigation, normalize(body_direction));
    return angles_from_direction(navigation_direction);
}
