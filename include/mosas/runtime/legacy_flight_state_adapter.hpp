#ifndef MOSAS_RUNTIME_LEGACY_FLIGHT_STATE_ADAPTER_HPP
#define MOSAS_RUNTIME_LEGACY_FLIGHT_STATE_ADAPTER_HPP

#include <mosas/ports/flight_state_estimator.hpp>
#include <mosas/runtime/attitude_estimator.hpp>

namespace mosas::runtime {

class LegacyFlightStateEstimatorAdapter final
    : public ports::IFlightStateEstimator {
public:
    explicit LegacyFlightStateEstimatorAdapter(
        const AttitudeEstimatorConfig& config = {});

    domain::FlightStateSnapshot update(
        const domain::ImuKinematicSample& sample) override;
    void reset() noexcept override;

private:
    AttitudeEstimator estimator_;
};

}  // namespace mosas::runtime

#endif  // MOSAS_RUNTIME_LEGACY_FLIGHT_STATE_ADAPTER_HPP
