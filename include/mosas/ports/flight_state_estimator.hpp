#ifndef MOSAS_PORTS_FLIGHT_STATE_ESTIMATOR_HPP
#define MOSAS_PORTS_FLIGHT_STATE_ESTIMATOR_HPP

#include <mosas/domain/contracts.hpp>

namespace mosas::ports {

// Runtime boundary for one IMU sample producing one coherent attitude/launch
// snapshot. The legacy implementation intentionally remains a single update.
class IFlightStateEstimator {
public:
    virtual ~IFlightStateEstimator() = default;
    virtual domain::FlightStateSnapshot update(
        const domain::ImuKinematicSample& sample) = 0;
    virtual void reset() noexcept = 0;
};

}  // namespace mosas::ports

#endif  // MOSAS_PORTS_FLIGHT_STATE_ESTIMATOR_HPP
