#include <mosas/guidance/default_algorithm_adapters.hpp>
#include <mosas/guidance/guidance_estimator.hpp>
namespace mosas::guidance {
domain::LineOfSight ExistingLosEstimatorAdapter::estimate(const domain::VisionDetection& observation,
    const domain::CameraIntrinsics& k,
    const domain::Rotation3<domain::DartFrameTag, domain::CameraFrameTag>& rotation) {
    VisionResult target{};
    target.found = observation.observation.valid;
    target.blob.center_x = observation.observation.center.u;
    target.blob.center_y = observation.observation.center.v;
    const auto& m = rotation.matrix();
    const auto los = GuidanceEstimator::calculate_body(target, {k.fx,k.fy,k.cx,k.cy},
        {m[0],m[1],m[2],m[3],m[4],m[5],m[6],m[7],m[8]});
    return {los.valid,{los.q_y},{los.q_z}};
}
}
