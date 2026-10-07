#ifndef MOSAS_PORTS_GUIDANCE_LAW_HPP
#define MOSAS_PORTS_GUIDANCE_LAW_HPP

#include <mosas/domain/algorithm_results.hpp>

namespace mosas::ports {

class IGuidanceLaw {
public:
    virtual ~IGuidanceLaw() = default;
    virtual domain::GuidanceOutput calculate(
        const domain::GuidanceInput& input) = 0;
};

}  // namespace mosas::ports

#endif  // MOSAS_PORTS_GUIDANCE_LAW_HPP
