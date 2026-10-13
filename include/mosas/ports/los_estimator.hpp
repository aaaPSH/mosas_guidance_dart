#ifndef MOSAS_PORTS_LOS_ESTIMATOR_HPP
#define MOSAS_PORTS_LOS_ESTIMATOR_HPP

#include <mosas/domain/algorithm_results.hpp>
#include <mosas/domain/geometry.hpp>

namespace mosas::ports {

/// 将像素目标观测转换为镖体系视线角的算法端口。
class ILosEstimator {
public:
    virtual ~ILosEstimator() = default;

    /**
     * @brief 根据相机内参和相机到镖体系旋转估计目标视线。
     * @param observation 单帧目标观测及其有效性。
     * @param intrinsics 相机内参，焦距与主点使用像素单位。
     * @param dart_from_camera 将相机系向量旋转到镖体系的外参矩阵。
     * @return 镖体系视线角（rad）；输入无效时通过结果的 valid 状态表示失败。
     */
    virtual domain::LineOfSight estimate(
        const domain::VisionDetection& observation,
        const domain::CameraIntrinsics& intrinsics,
        const domain::Rotation3<domain::DartFrameTag,
                                domain::CameraFrameTag>& dart_from_camera) = 0;
};

}  // namespace mosas::ports

#endif  // MOSAS_PORTS_LOS_ESTIMATOR_HPP
