#ifndef MOSAS_GUIDANCE_PINHOLE_LOS_ESTIMATOR_HPP
#define MOSAS_GUIDANCE_PINHOLE_LOS_ESTIMATOR_HPP

#include <mosas/ports/los_estimator.hpp>

namespace mosas::guidance {

/** @brief 针孔反投影及安装旋转，直接生成镖体系单位视线，不计算中间角度。 */
class PinholeLosEstimator final : public ports::ILosEstimator {
public:
    /**
     * @brief 将目标像素反投影、旋转并单位化为镖体系方向。
     * @param observation 目标检测；像素 u 向右、v 向下。
     * @param intrinsics 有限内参，fx 和 fy 必须为正，单位为像素。
     * @param dart_from_camera 相机系至镖体系的 SO(3) 安装旋转。
     * @return 相机光心指向目标的单位向量；无效输入或数值异常时 valid=false。
     * @note 相机系和镖体系均采用 X 前、Y 上、Z 右；不平移视线起点。
     */
    domain::DartLineOfSight estimate(
        const domain::VisionDetection& observation,
        const domain::CameraIntrinsics& intrinsics,
        const domain::Rotation3<domain::DartFrameTag,
                                domain::CameraFrameTag>& dart_from_camera) override;
};

}  // namespace mosas::guidance

#endif  // MOSAS_GUIDANCE_PINHOLE_LOS_ESTIMATOR_HPP
