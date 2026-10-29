#include <mosas/guidance/guidance_estimator.hpp>
#include <mosas/runtime/vector3.hpp>
#include "vector_los_math.hpp"
#include <cmath>
namespace {
/** @brief 将像素反投影为相机 +X 前、+Y 上、+Z 右的方向。 */
bool camera_direction(const VisionResult& result, const CameraIntrinsics& k, Vector3& v) {
    if (!result.found || !std::isfinite(result.blob.center_x) || !std::isfinite(result.blob.center_y) ||
        !std::isfinite(k.fx) || !std::isfinite(k.fy) || !std::isfinite(k.cx) || !std::isfinite(k.cy) || k.fx <= 0 || k.fy <= 0) return false;
    v = {1, -(result.blob.center_y - k.cy) / k.fy, (result.blob.center_x - k.cx) / k.fx};
    return std::isfinite(v.y) && std::isfinite(v.z);
}
/** @return 非零有限方向对应的 LOS 角，单位 rad。 */
LineOfSight angles(const Vector3& v) {
    const double norm = std::hypot(std::hypot(v.x, v.y), v.z);
    if (!std::isfinite(norm) || norm < 1e-12) return {false,0,0};
    return {true, std::atan2(v.y, std::hypot(v.x, v.z)), std::atan2(v.z, v.x)};
}
}
LineOfSight GuidanceEstimator::calculate(const VisionResult& result, const CameraIntrinsics& k) {
    Vector3 v{};
    return camera_direction(result,k,v) ? angles(v) : LineOfSight{false,0,0};
}
LineOfSight GuidanceEstimator::calculate_body(const VisionResult& result, const CameraIntrinsics& k,
    const RotationMatrix3& r) {
    if (!mosas::guidance::detail::valid_rotation({r.m00,r.m01,r.m02,r.m10,r.m11,r.m12,r.m20,r.m21,r.m22})) return {false,0,0};
    Vector3 v{};
    if (!camera_direction(result,k,v)) return {false,0,0};
    return angles({r.m00*v.x+r.m01*v.y+r.m02*v.z, r.m10*v.x+r.m11*v.y+r.m12*v.z, r.m20*v.x+r.m21*v.y+r.m22*v.z});
}
