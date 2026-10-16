#include <mosas/app/default_guidance_algorithms.hpp>

#include <mosas/guidance/default_algorithm_adapters.hpp>
#include <mosas/runtime/clock.hpp>
#include <mosas/runtime/legacy_flight_state_adapter.hpp>
#include <mosas/vision/opencv_vision_detector.hpp>

#include <memory>
#include <utility>

namespace mosas::app {

runtime::RuntimeSupervisorDependencies make_default_guidance_algorithms(
    const GuidanceAlgorithmConfig& config, guidance::PngTraceObserver trace_observer) {
    runtime::RuntimeSupervisorDependencies dependencies;
    dependencies.vision_detector =
        std::make_unique<vision::OpenCvVisionDetector>(config.vision_config);
    dependencies.flight_state_estimator =
        std::make_unique<runtime::LegacyFlightStateEstimatorAdapter>(
            config.attitude);
    dependencies.los_estimator =
        std::make_unique<guidance::ExistingLosEstimatorAdapter>();
    if (config.algorithm == GuidanceAlgorithm::body_vector_pn) {
        dependencies.los_rate_filter =
            std::make_unique<guidance::BodyVectorLosFilter>(config.body_vector_filter);
        auto pn = config.body_vector_pn;
        pn.gravity = config.attitude.gravity_mps2;
        dependencies.guidance_law = std::make_unique<guidance::BodyVectorPnGuidance>(pn);
    } else {
        dependencies.los_rate_filter =
            std::make_unique<guidance::ExistingLosRateFilterAdapter>(config.los_rate_filter);
        dependencies.guidance_law =
            std::make_unique<guidance::ExistingPngGuidanceLawAdapter>(
                config.png_guidance, std::move(trace_observer));
    }
    dependencies.monotonic_clock =
        std::make_unique<runtime::SystemMonotonicClock>();
    return dependencies;
}

}  // namespace mosas::app
