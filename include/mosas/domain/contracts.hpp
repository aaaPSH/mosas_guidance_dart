#ifndef MOSAS_DOMAIN_CONTRACTS_HPP
#define MOSAS_DOMAIN_CONTRACTS_HPP

#include <mosas/domain/geometry.hpp>
#include <mosas/domain/time.hpp>

#include <cstdint>
#include <optional>

namespace mosas::domain {

struct PixelPoint {
    double u = 0.0;
    double v = 0.0;
};

enum class ObservationReason { found, not_found, invalid_frame, stale_frame };

/** @brief 带时间的 IMU 运动样本及仅供内录的飞控元数据；外环输出单位 rad/s。 */
struct ImuKinematicSample {
    SensorTime time{};
    AccelerationDartMps2 acceleration{};
    AngularVelocityDartRadPerSec angular_velocity{};
    double servo_control_p_normalized = 0.0;
    double servo_control_y_normalized = 0.0;
    double outer_loop_y_rad_s = 0.0;
    double outer_loop_p_rad_s = 0.0;
    /** @brief 原始设备帧号；不提供帧号的模拟源保持为空。 */
    std::optional<std::uint32_t> imu_index = std::nullopt;
    std::uint8_t trigger_flag = 0;
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

/** @brief 制导链路及 CSV 共用的稳定原因码；0 表示有效，非零包括失败、等待及未执行。 */
enum class GuidanceInvalidReason {
    none = 0,
    launch_not_confirmed = 1,
    stale_input = 2,
    no_target = 3,
    invalid_los = 4,
    invalid_rate = 5,
    invalid_attitude = 6,
    algorithm_rejected = 7,
    not_evaluated = 8,
    imu_initializing = 9,
    imu_history_unavailable = 10,
    imu_history_too_old = 11,
    attitude_history_unavailable = 12,
    non_increasing_timestamp = 13,
    los_initializing = 14,
    los_gap = 15,
    invalid_timestep = 16,
    output_not_submitted = 17,
    output_replaced = 18,
    output_wait_timeout = 19,
    output_invalidated = 20,
    output_cancelled = 21,
    generation_mismatch = 22,
    output_receipt_mismatch = 23,
    io_rejected = 24,
    debug_inactive = 25,
    output_disabled = 26,
    command_expired = 27,
    invalid_source_time = 28,
    future_source_time = 29,
    source_too_old = 30,
    invalid_attitude_time = 31,
    attitude_too_old = 32,
    invalid_configuration = 33,
    state_source_failed = 34,
    los_pole = 35,
    invalid_speed = 36,
    filter_rejected = 37,
    invalid_frame = 38,
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
