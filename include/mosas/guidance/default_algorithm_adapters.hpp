#ifndef MOSAS_GUIDANCE_DEFAULT_ALGORITHM_ADAPTERS_HPP
#define MOSAS_GUIDANCE_DEFAULT_ALGORITHM_ADAPTERS_HPP

#include <mosas/guidance/guidance_estimator.hpp>
#include <mosas/ports/guidance_law.hpp>
#include <mosas/ports/los_estimator.hpp>
#include <mosas/ports/los_rate_filter.hpp>

namespace mosas::guidance {

class ExistingLosEstimatorAdapter final : public ports::ILosEstimator {
public:
    domain::LineOfSight estimate(
        const domain::VisionDetection& observation,
        const domain::CameraIntrinsics& intrinsics,
        const domain::Rotation3<domain::DartFrameTag,
                                domain::CameraFrameTag>& dart_from_camera)
        override;
};

class ExistingLosRateFilterAdapter final : public ports::ILosRateFilter {
public:
    explicit ExistingLosRateFilterAdapter(
        const LineOfSightRateFilterConfig& config = {});

    domain::LineOfSightAngularVelocity update(
        const domain::LineOfSight& line_of_sight,
        const domain::AngularVelocityDartRadPerSec& body_angular_velocity,
        double dt_seconds) override;
    domain::LineOfSightRateSample sample() const override;
    void reset() noexcept override;

private:
    LineOfSightRateEstimator estimator_;
};

class ExistingPngGuidanceLawAdapter final : public ports::IGuidanceLaw {
public:
    explicit ExistingPngGuidanceLawAdapter(
        const PngGuidanceConfig& config = {});

    domain::GuidanceOutput calculate(
        const domain::GuidanceInput& input) override;

private:
    PngGuidanceConfig config_;
};

}  // namespace mosas::guidance

#endif  // MOSAS_GUIDANCE_DEFAULT_ALGORITHM_ADAPTERS_HPP
