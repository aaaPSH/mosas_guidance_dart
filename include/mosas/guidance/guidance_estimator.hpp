#ifndef GUIDANCE_ESTIMATOR_HPP
#define GUIDANCE_ESTIMATOR_HPP
#include <mosas/runtime/rotation_matrix3.hpp>
#include <mosas/vision/vision_types.hpp>

struct CameraIntrinsics { double fx, fy, cx, cy; };
struct LineOfSight { bool valid; double q_y, q_z; };
/** @brief 像素/安装外参的静态几何运算，不承担滤波或制导。 */
class GuidanceEstimator {
public:
    /** @param result 视觉目标。 @param intrinsics 相机内参。
     * @return 相机系 LOS 角；非法输入时无效。
     */
    static LineOfSight calculate(const VisionResult& result, const CameraIntrinsics& intrinsics);
    /** @param result 视觉目标。 @param intrinsics 相机内参。
     * @param camera_to_body SO(3) 相机安装旋转。
     * @return 镖体系 LOS 角；非法输入时无效。
     */
    static LineOfSight calculate_body(const VisionResult& result, const CameraIntrinsics& intrinsics,
                                    const RotationMatrix3& camera_to_body);
};
#endif
