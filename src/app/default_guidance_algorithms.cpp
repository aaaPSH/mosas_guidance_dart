#include <mosas/app/default_guidance_algorithms.hpp>

#include <mosas/guidance/ballistic_png_guidance.hpp>
#include <mosas/guidance/command_output_mapper.hpp>
#include <mosas/guidance/default_algorithm_adapters.hpp>
#include <mosas/guidance/inertial_vector_los_filter.hpp>
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

    dependencies.los_rate_filter =
        std::make_unique<guidance::InertialVectorLosFilter>(config.inertial_filter);
    auto png = config.ballistic_png;
    png.gravity = config.attitude.gravity_mps2;
    dependencies.guidance_law =
        std::make_unique<guidance::BallisticPngGuidance>(png, std::move(trace_observer));
    auto mapping = config.command_mapping;
    mapping.gravity = config.attitude.gravity_mps2;
    dependencies.command_mapper = std::make_unique<guidance::CommandOutputMapper>(mapping);

    dependencies.monotonic_clock =
        std::make_unique<runtime::SystemMonotonicClock>();
    return dependencies;
}

}  // namespace mosas::app
