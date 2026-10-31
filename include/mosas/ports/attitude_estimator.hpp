#ifndef MOSAS_PORTS_ATTITUDE_ESTIMATOR_HPP
#define MOSAS_PORTS_ATTITUDE_ESTIMATOR_HPP

#include <mosas/domain/contracts.hpp>

namespace mosas::ports {

class IAttitudeEstimator {
public:
    virtual ~IAttitudeEstimator() = default;
    virtual domain::AttitudeState update(
        const domain::ImuKinematicSample& sample) = 0;
    virtual void reset() noexcept = 0;
};

}  // namespace mosas::ports

#endif  // MOSAS_PORTS_ATTITUDE_ESTIMATOR_HPP
