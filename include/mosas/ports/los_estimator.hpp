#ifndef MOSAS_PORTS_LOS_ESTIMATOR_HPP
#define MOSAS_PORTS_LOS_ESTIMATOR_HPP

#include <mosas/domain/algorithm_results.hpp>
#include <mosas/domain/geometry.hpp>

namespace mosas::ports {

class ILosEstimator {
public:
    virtual ~ILosEstimator() = default;
    virtual domain::LineOfSight estimate(
        const domain::VisionDetection& observation,
        const domain::CameraIntrinsics& intrinsics,
        const domain::Rotation3<domain::DartFrameTag,
                                domain::CameraFrameTag>& dart_from_camera) = 0;
};

}  // namespace mosas::ports

#endif  // MOSAS_PORTS_LOS_ESTIMATOR_HPP
