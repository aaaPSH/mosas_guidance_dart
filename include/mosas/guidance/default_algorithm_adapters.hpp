#ifndef MOSAS_GUIDANCE_DEFAULT_ALGORITHM_ADAPTERS_HPP
#define MOSAS_GUIDANCE_DEFAULT_ALGORITHM_ADAPTERS_HPP
#include <mosas/ports/los_estimator.hpp>
namespace mosas::guidance {
/** @brief 复用像素转视线的静态几何，不包含旧滤波/导引适配器。 */
class ExistingLosEstimatorAdapter final : public ports::ILosEstimator {
public:
    /** @param observation 目标检测。 @param intrinsics 相机内参。
     * @param dart_from_camera 固定安装旋转。 @return 镖体系 LOS 角。
     */
    domain::LineOfSight estimate(const domain::VisionDetection& observation,
        const domain::CameraIntrinsics& intrinsics,
        const domain::Rotation3<domain::DartFrameTag, domain::CameraFrameTag>& dart_from_camera) override;
};
}
#endif
