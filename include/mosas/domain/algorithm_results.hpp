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

/** @brief 相机光心指向目标的镖体系观测；有效方向必须有限且单位化。 */
struct DartLineOfSight {
    bool valid = false;
    DirectionDartUnit direction{};
};

/** @brief 导航惯性系仰角导数及横向有效角速率，不包含抛物线基准补偿。 */
struct LineOfSightAngularVelocity {
    bool valid = false;
    Quantity<RadiansPerSecond> q_y{};
    /** @brief cos(q_y) * dq_z/dt，而非未经投影的方位角导数。 */
    Quantity<RadiansPerSecond> q_z{};
};

/** @brief 导航系惯性 LOS 状态；方向单位长度，导数与方向正交。 */
struct VectorLosEstimate {
    bool valid = false;
    DirectionNavigationUnit direction{};
    DirectionRateNavigationPerSec inertial_direction_rate{};
};

/** @brief 镖体系单位视线及同帧曝光姿态；首帧仅建立基准，不传播 dt。 */
struct LosRateFilterInput {
    DartLineOfSight line_of_sight{};
    AttitudeState attitude{};
    double dt_seconds = 0.0;
    std::uint64_t generation = 0;
};

/** @brief 导航系制导状态及速率诊断；角度滤波在方位极点拒绝输出。 */
struct LosRateEstimate {
    bool valid = false;
    LineOfSightAngularVelocity angular_velocity{};
    VectorLosEstimate vector{};
};

/** @brief 最近镖体系单位观测、补偿后惯性角度差分及滤波角速率诊断。 */
struct LineOfSightRateSample {
    DartLineOfSight line_of_sight{};
    LineOfSightAngularVelocity raw_angular_velocity{};
    LineOfSightAngularVelocity filtered_angular_velocity{};
};

struct GuidanceInput {
    std::uint64_t source_frame_id = 0;
    SensorTime time{};
    /** @brief 插值到相机曝光时刻的动态纵向速度，单位 m/s。 */
    Quantity<MetersPerSecond> speed_mps{};
    VectorLosEstimate vector_los{};
};

/** @brief 计算阶段只记录原始弹道请求；最终输出诊断由发送端提供。 */
struct GuidanceDiagnostics {
    BallisticOverloadG requested_overload{};
};

struct GuidanceOutput {
    bool valid = false;
    GuidanceInvalidReason invalid_reason =
        GuidanceInvalidReason::algorithm_rejected;
    GuidanceCommand command{};
    GuidanceDiagnostics diagnostics{};
};

/** @brief 当前帧的观测快照；LOS 角度及滤波角速度统一使用导航惯性系。 */
struct GuidanceOverlay {
    bool line_of_sight_valid = false;
    Quantity<Radian> line_of_sight_q_y{};
    Quantity<Radian> line_of_sight_q_z{};
    bool line_of_sight_rate_valid = false;
    Quantity<RadiansPerSecond> line_of_sight_rate_q_y{};
    /** @brief 横向有效角速率 cos(q_y) * dq_z/dt，不是未经投影的方位角导数。 */
    Quantity<RadiansPerSecond> line_of_sight_rate_q_z{};
    bool body_overload_valid = false;
    DartOverloadG body_overload{};
    bool attitude_valid = false;
    Quantity<Radian> attitude_pitch{};
    Quantity<Radian> attitude_roll{};
    bool output_attitude_valid = false;
    Quantity<Radian> output_attitude_pitch{};
    Quantity<Radian> output_attitude_roll{};
    LaunchPhase launch_phase = LaunchPhase::unknown;
    double processing_fps = 0.0;
};

}  // namespace mosas::domain

#endif  // MOSAS_DOMAIN_ALGORITHM_RESULTS_HPP
