#ifndef MOSAS_PORTS_LAUNCH_DETECTOR_HPP
#define MOSAS_PORTS_LAUNCH_DETECTOR_HPP

#include <mosas/domain/contracts.hpp>

namespace mosas::ports {

class ILaunchDetector {
public:
    virtual ~ILaunchDetector() = default;
    virtual domain::LaunchState update(
        const domain::ImuKinematicSample& sample) = 0;
    virtual void reset() noexcept = 0;
};

}  // namespace mosas::ports

#endif  // MOSAS_PORTS_LAUNCH_DETECTOR_HPP
