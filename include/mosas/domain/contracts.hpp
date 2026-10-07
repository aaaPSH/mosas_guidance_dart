#ifndef MOSAS_DOMAIN_CONTRACTS_HPP
#define MOSAS_DOMAIN_CONTRACTS_HPP

#include <mosas/domain/geometry.hpp>
#include <mosas/domain/time.hpp>

#include <cstdint>

namespace mosas::domain {

struct PixelPoint {
    double u = 0.0;
    double v = 0.0;
};

enum class ObservationReason { found, not_found, invalid_frame, stale_frame };

struct ImuKinematicSample {
    SensorTime time{};
    AccelerationDartMps2 acceleration{};
    AngularVelocityDartRadPerSec angular_velocity{};
    double servo_control_p_normalized = 0.0;
    double servo_control_y_normalized = 0.0;
};

struct TargetObservation {
    std::uint64_t source_frame_id = 0;
    SensorTime time{};
    bool valid = false;
    PixelPoint center{};
    ObservationReason reason = ObservationReason::not_found;
};

struct AttitudeState {
    SensorTime time{};
    Rotation3<NavigationFrameTag, DartFrameTag> navigation_from_dart{};
    Quantity<Radian> pitch_rad{};
    Quantity<Radian> roll_rad{};
    AngularVelocityDartRadPerSec corrected_angular_velocity{};
    bool valid = false;
};

enum class LaunchPhase { unknown, pre_launch, launched };
enum class LaunchReason { threshold_confirmed, not_started, invalid_imu, stale_imu };

struct LaunchState {
    LaunchPhase phase = LaunchPhase::unknown;
    SensorTime transition_time{};
    std::uint64_t generation = 0;
    LaunchReason reason = LaunchReason::not_started;
};

struct FlightStateSnapshot {
    SensorTime time{};
    ImuKinematicSample imu_sample{};
    AttitudeState attitude{};
    LaunchState launch{};
};

enum class GuidanceInvalidReason {
    none,
    launch_not_confirmed,
    stale_input,
    no_target,
    invalid_los,
    invalid_rate,
    invalid_attitude,
    algorithm_rejected,
};

struct GuidanceCommand {
    std::uint64_t generation = 0;
    std::uint64_t source_frame_id = 0;
    SensorTime source_time{};
    bool valid = false;
    GuidanceInvalidReason invalid_reason =
        GuidanceInvalidReason::launch_not_confirmed;
    DartOverloadG overload_g{};
};

}  // namespace mosas::domain

#endif  // MOSAS_DOMAIN_CONTRACTS_HPP
