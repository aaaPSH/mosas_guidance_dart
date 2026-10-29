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

/** @brief launched 表示所选发射判据已确认；仅 two_stage 模式额外要求离架确认。 */
enum class LaunchPhase { unknown, pre_launch, launched };
enum class LaunchReason { threshold_confirmed, not_started, invalid_imu, stale_imu };

struct LaunchState {
    LaunchPhase phase = LaunchPhase::unknown;
    /** @brief 所选发射判据的确认帧时间；不回溯到候选起点。 */
    SensorTime transition_time{};
    std::uint64_t generation = 0;
    LaunchReason reason = LaunchReason::not_started;
};

struct FlightStateSnapshot {
    SensorTime time{};
    ImuKinematicSample imu_sample{};
    AttitudeState attitude{};
    LaunchState launch{};
    /** @brief 与姿态同一时刻的纵向速度，单位 m/s；触发前及确认帧保持标定初速。 */
    Quantity<MetersPerSecond> speed_mps{};
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
    /** @brief 未选轴、未限幅的弹道请求；必须经输出映射后才能编码。 */
    BallisticOverloadG overload_g{};
    /** @brief 当前周期是否已按所选发射判据触发，独立于制导有效性。 */
    bool launched = false;
    /** @brief 当前帧是否识别到目标；未执行识别或故障时为 false。 */
    bool target_found = false;
};

}  // namespace mosas::domain

#endif  // MOSAS_DOMAIN_CONTRACTS_HPP
