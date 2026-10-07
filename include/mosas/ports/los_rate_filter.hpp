#ifndef MOSAS_PORTS_LOS_RATE_FILTER_HPP
#define MOSAS_PORTS_LOS_RATE_FILTER_HPP

#include <mosas/domain/algorithm_results.hpp>
#include <mosas/domain/geometry.hpp>

namespace mosas::ports {

class ILosRateFilter {
public:
    virtual ~ILosRateFilter() = default;
    virtual domain::LineOfSightAngularVelocity update(
        const domain::LineOfSight& line_of_sight,
        const domain::AngularVelocityDartRadPerSec& body_angular_velocity,
        double dt_seconds) = 0;
    virtual domain::LineOfSightRateSample sample() const = 0;
    virtual void reset() noexcept = 0;
};

}  // namespace mosas::ports

#endif  // MOSAS_PORTS_LOS_RATE_FILTER_HPP
