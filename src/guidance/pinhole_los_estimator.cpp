#include <mosas/guidance/pinhole_los_estimator.hpp>

#include "rotation_validation.hpp"

#include <cmath>

namespace mosas::guidance {

domain::DartLineOfSight PinholeLosEstimator::estimate(
    const domain::VisionDetection& observation,
    const domain::CameraIntrinsics& intrinsics,
    const domain::Rotation3<domain::DartFrameTag,
                            domain::CameraFrameTag>& dart_from_camera) {
    const auto& target = observation.observation;
    const auto& k = intrinsics;
    if (!target.valid || !std::isfinite(target.center.u) || !std::isfinite(target.center.v) ||
        !std::isfinite(k.fx) || !std::isfinite(k.fy) ||
        !std::isfinite(k.cx) || !std::isfinite(k.cy) || k.fx <= 0.0 || k.fy <= 0.0 ||
        !detail::valid_rotation(dart_from_camera.matrix())) {
        return {};
    }

    // 像素 v 向下，而项目相机 Y 轴向上；X 是光轴，Z 向右。
    const domain::Vector3<domain::CameraFrameTag, domain::Dimensionless> camera_direction{
        1.0, -(target.center.v - k.cy) / k.fy, (target.center.u - k.cx) / k.fx};
    if (!std::isfinite(camera_direction.y) || !std::isfinite(camera_direction.z)) {
        return {};
    }
    const auto direction = dart_from_camera.apply(camera_direction);
    const double norm = std::hypot(std::hypot(direction.x, direction.y), direction.z);
    if (!std::isfinite(norm) || norm < 1e-12) return {};
    return {true, {direction.x / norm, direction.y / norm, direction.z / norm}};
}

}  // namespace mosas::guidance
