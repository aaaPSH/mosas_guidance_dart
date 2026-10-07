#include <mosas/app/default_guidance_algorithms.hpp>

#include <mosas/guidance/default_algorithm_adapters.hpp>
#include <mosas/runtime/clock.hpp>
#include <mosas/runtime/legacy_flight_state_adapter.hpp>
#include <mosas/vision/opencv_vision_detector.hpp>

#include <memory>

namespace mosas::app {

runtime::GuidanceRuntimeDependencies make_default_guidance_algorithms(
    const GuidanceAlgorithmConfig& config) {
    runtime::GuidanceRuntimeDependencies dependencies;
    dependencies.vision_detector =
        std::make_unique<vision::OpenCvVisionDetector>(config.vision_config);
    dependencies.flight_state_estimator =
        std::make_unique<runtime::LegacyFlightStateEstimatorAdapter>(
            config.attitude);
    dependencies.los_estimator =
        std::make_unique<guidance::ExistingLosEstimatorAdapter>();
    dependencies.los_rate_filter =
        std::make_unique<guidance::ExistingLosRateFilterAdapter>(
            config.los_rate_filter);
    dependencies.guidance_law =
        std::make_unique<guidance::ExistingPngGuidanceLawAdapter>(
            config.png_guidance);
    dependencies.monotonic_clock =
        std::make_unique<runtime::SystemMonotonicClock>();
    return dependencies;
}

}  // namespace mosas::app
