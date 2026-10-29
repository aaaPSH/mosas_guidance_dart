#include <mosas/guidance/default_algorithm_adapters.hpp>
#include <mosas/guidance/guidance_estimator.hpp>
#include "test_support.hpp"
#include <limits>
int main() {
    using namespace mosas;
    VisionResult observation{};
    observation.found=true; observation.blob.center_x=420; observation.blob.center_y=190;
    const CameraIntrinsics intrinsics{320,320,320,240};
    const RotationMatrix3 identity{1,0,0,0,1,0,0,0,1};
    const auto los=GuidanceEstimator::calculate_body(observation,intrinsics,identity);
    CHECK(los.valid);
    CHECK_NEAR(los.q_y,0.14804631692728484,1e-12);
    CHECK_NEAR(los.q_z,0.3028848683749714,1e-12);
    CHECK(!GuidanceEstimator::calculate_body(observation,{0,320,320,240},identity).valid);
    observation.found=false;
    CHECK(!GuidanceEstimator::calculate(observation,intrinsics).valid);
    observation.found=true;
    domain::VisionDetection detected;
    detected.observation.valid=true;detected.observation.center={420,190};
    guidance::ExistingLosEstimatorAdapter adapter;
    auto result=adapter.estimate(detected,{320,320,320,240},{});
    CHECK(result.valid);
    CHECK_NEAR(result.q_y.value,los.q_y,1e-12);
    CHECK_NEAR(result.q_z.value,los.q_z,1e-12);
    // 安装旋转只改变几何轴，不涉及任何 IMU 前馈。
    const domain::Rotation3<domain::DartFrameTag,domain::CameraFrameTag> flipped({1,0,0,0,-1,0,0,0,-1});
    result=adapter.estimate(detected,{320,320,320,240},flipped);
    CHECK(result.valid);
    CHECK_NEAR(result.q_y.value,-los.q_y,1e-12);
    CHECK_NEAR(result.q_z.value,-los.q_z,1e-12);
    for(const auto& bad: {domain::Rotation3<domain::DartFrameTag,domain::CameraFrameTag>({-1,0,0,0,1,0,0,0,1}),
                          domain::Rotation3<domain::DartFrameTag,domain::CameraFrameTag>({1,0,0,1,0,0,0,0,1})})
        CHECK(!adapter.estimate(detected,{320,320,320,240},bad).valid);
    detected.observation.center.u=std::numeric_limits<double>::quiet_NaN();
    CHECK(!adapter.estimate(detected,{320,320,320,240},{}).valid);
    return 0;
}
