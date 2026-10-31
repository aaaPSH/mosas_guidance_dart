#include <mosas/guidance/pinhole_los_estimator.hpp>

#include "test_support.hpp"

#include <array>
#include <cmath>
#include <limits>

/** @return 针孔反投影、安装旋转和无效输入保护符合向量契约时返回 0。 */
int main() {
    using namespace mosas;
    guidance::PinholeLosEstimator estimator;
    domain::VisionDetection detection;
    detection.observation.valid = true;
    detection.observation.center = {320.0, 240.0};
    const domain::CameraIntrinsics intrinsics{320.0, 320.0, 320.0, 240.0};

    auto los = estimator.estimate(detection, intrinsics, {});
    CHECK(los.valid);
    CHECK_NEAR(los.direction.x, 1.0, 1e-12);
    CHECK_NEAR(los.direction.y, 0.0, 1e-12);
    CHECK_NEAR(los.direction.z, 0.0, 1e-12);

    detection.observation.center = {420.0, 190.0};
    const double norm = std::sqrt(1.0 + 0.15625 * 0.15625 + 0.3125 * 0.3125);
    los = estimator.estimate(detection, intrinsics, {});
    CHECK(los.valid);
    CHECK_NEAR(los.direction.x, 1.0 / norm, 1e-12);
    CHECK_NEAR(los.direction.y, 0.15625 / norm, 1e-12);
    CHECK_NEAR(los.direction.z, 0.3125 / norm, 1e-12);
    CHECK_NEAR(std::hypot(std::hypot(los.direction.x, los.direction.y), los.direction.z),
               1.0, 1e-12);

    using Installation = domain::Rotation3<domain::DartFrameTag, domain::CameraFrameTag>;
    const Installation flipped({1, 0, 0, 0, -1, 0, 0, 0, -1});
    los = estimator.estimate(detection, intrinsics, flipped);
    CHECK(los.valid);
    CHECK_NEAR(los.direction.x, 1.0 / norm, 1e-12);
    CHECK_NEAR(los.direction.y, -0.15625 / norm, 1e-12);
    CHECK_NEAR(los.direction.z, -0.3125 / norm, 1e-12);

    const Installation compound({0.8, -0.6, 0.0, 0.48, 0.64, -0.6, 0.36, 0.48, 0.8});
    los = estimator.estimate(detection, intrinsics, compound);
    CHECK(los.valid);
    CHECK_NEAR(los.direction.x, 0.70625 / norm, 1e-12);
    CHECK_NEAR(los.direction.y, 0.3925 / norm, 1e-12);
    CHECK_NEAR(los.direction.z, 0.685 / norm, 1e-12);

    // 精确极点也是合法方向，不需要借助方位角表达。
    detection.observation.center = {320.0, 240.0};
    los = estimator.estimate(detection, intrinsics,
                             Installation({0, 0, 1, 1, 0, 0, 0, 1, 0}));
    CHECK(los.valid);
    CHECK_NEAR(los.direction.x, 0.0, 1e-12);
    CHECK_NEAR(los.direction.y, 1.0, 1e-12);
    CHECK_NEAR(los.direction.z, 0.0, 1e-12);

    detection.observation.valid = false;
    CHECK(!estimator.estimate(detection, intrinsics, {}).valid);
    detection.observation.valid = true;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    for (const double invalid : {nan, inf, -inf}) {
        detection.observation.center = {invalid, 240.0};
        CHECK(!estimator.estimate(detection, intrinsics, {}).valid);
        detection.observation.center = {320.0, invalid};
        CHECK(!estimator.estimate(detection, intrinsics, {}).valid);
        detection.observation.center = {320.0, 240.0};
        for (const auto& k : std::array<domain::CameraIntrinsics, 4>{{
                 {invalid, 320, 320, 240}, {320, invalid, 320, 240},
                 {320, 320, invalid, 240}, {320, 320, 320, invalid}}}) {
            CHECK(!estimator.estimate(detection, k, {}).valid);
        }
    }
    for (const auto& k : std::array<domain::CameraIntrinsics, 4>{{
             {0, 320, 320, 240}, {-1, 320, 320, 240},
             {320, 0, 320, 240}, {320, -1, 320, 240}}}) {
        CHECK(!estimator.estimate(detection, k, {}).valid);
    }
    for (const auto& rotation : {
             Installation({-1, 0, 0, 0, 1, 0, 0, 0, 1}),
             Installation({1, 0, 0, 1, 0, 0, 0, 0, 1}),
             Installation({nan, 0, 0, 0, 1, 0, 0, 0, 1}),
             Installation({1, 0, 0, 0, 1, 0, 0, 0, inf})}) {
        CHECK(!estimator.estimate(detection, intrinsics, rotation).valid);
    }

    detection.observation.center = {std::numeric_limits<double>::max(), 240.0};
    CHECK(!estimator.estimate(detection, {320, 320, -std::numeric_limits<double>::max(), 240}, {}).valid);
    detection.observation.center = {420.0, 190.0};
    CHECK(!estimator.estimate(detection, {std::numeric_limits<double>::min(), 320, 320, 240}, {}).valid);
    return 0;
}
