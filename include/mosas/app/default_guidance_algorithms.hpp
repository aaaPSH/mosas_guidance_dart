#ifndef MOSAS_APP_DEFAULT_GUIDANCE_ALGORITHMS_HPP
#define MOSAS_APP_DEFAULT_GUIDANCE_ALGORITHMS_HPP

#include <mosas/app/app_config.hpp>

namespace mosas::app {

runtime::GuidanceRuntimeDependencies make_default_guidance_algorithms(
    const GuidanceAlgorithmConfig& config);

}  // namespace mosas::app

#endif  // MOSAS_APP_DEFAULT_GUIDANCE_ALGORITHMS_HPP
