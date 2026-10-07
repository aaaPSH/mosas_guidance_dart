#ifndef MOSAS_DOMAIN_ALGORITHM_RESULTS_HPP
#define MOSAS_DOMAIN_ALGORITHM_RESULTS_HPP

#include <mosas/domain/contracts.hpp>

#include <cstdint>

namespace mosas::domain {

struct CameraIntrinsics {
    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;
};

struct PixelRect {
    int x = 0;
    int y = 0;
    int width = 0;
    int height = 0;
};

// Rendering/tracker metadata is kept separate from the control observation.
struct VisionDiagnostics {
    PixelRect bounding_box{};
    PixelRect next_search_region{};
    int area_pixels = 0;
    int minimum_blob_area_pixels = 0;
};

struct VisionDetection {
    TargetObservation observation{};
    VisionDiagnostics diagnostics{};
};

struct LineOfSight {
    bool valid = false;
    Quantity<Radian> q_y{};
    Quantity<Radian> q_z{};
};

struct LineOfSightAngularVelocity {
    bool valid = false;
    Quantity<RadiansPerSecond> q_y{};
    Quantity<RadiansPerSecond> q_z{};
};

struct LineOfSightRateSample {
    LineOfSight line_of_sight{};
    LineOfSightAngularVelocity raw_angular_velocity{};
    LineOfSightAngularVelocity filtered_angular_velocity{};
};

struct GuidanceInput {
    std::uint64_t source_frame_id = 0;
    SensorTime time{};
    LineOfSight line_of_sight{};
    LineOfSightAngularVelocity angular_velocity{};
    Quantity<MetersPerSecond> speed_mps{};
    AttitudeState attitude{};
};

struct GuidanceDiagnostics {
    AccelerationBallisticMps2 navigation_acceleration{};
    AccelerationDartMps2 body_acceleration{};
    DartOverloadG body_overload{};
    Quantity<GravityG> command_overload{};
    Quantity<Radian> command_phase{};
};

struct GuidanceOutput {
    bool valid = false;
    GuidanceInvalidReason invalid_reason =
        GuidanceInvalidReason::algorithm_rejected;
    GuidanceCommand command{};
    GuidanceDiagnostics diagnostics{};
};

struct GuidanceOverlay {
    bool line_of_sight_valid = false;
    Quantity<Radian> line_of_sight_q_y{};
    Quantity<Radian> line_of_sight_q_z{};
    bool line_of_sight_rate_valid = false;
    Quantity<RadiansPerSecond> line_of_sight_rate_q_y{};
    Quantity<RadiansPerSecond> line_of_sight_rate_q_z{};
    bool body_overload_valid = false;
    DartOverloadG body_overload{};
    bool attitude_valid = false;
    Quantity<Radian> attitude_pitch{};
    Quantity<Radian> attitude_roll{};
    LaunchPhase launch_phase = LaunchPhase::unknown;
    double processing_fps = 0.0;
};

}  // namespace mosas::domain

#endif  // MOSAS_DOMAIN_ALGORITHM_RESULTS_HPP
