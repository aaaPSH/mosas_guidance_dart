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

/** @brief 惯性 LOS 状态，以镖体系表达；方向单位长度，惯性导数与方向正交。 */
struct VectorLosEstimate {
    bool valid = false;
    DirectionDartUnit direction{};
    DirectionRateDartPerSec inertial_direction_rate{};
};

/** @brief 同步到同一相机帧的滤波输入；首帧仅建立基准，不传播 dt。 */
struct LosRateFilterInput {
    LineOfSight line_of_sight{};
    AttitudeState attitude{};
    double dt_seconds = 0.0;
    std::uint64_t generation = 0;
};

/** @brief 制导速率估计；共同 valid 不依赖角度诊断在极点处是否有效。 */
struct LosRateEstimate {
    bool valid = false;
    LineOfSightAngularVelocity angular_velocity{};
    VectorLosEstimate vector{};
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
    /** @brief 插值到相机曝光时刻的动态纵向速度，单位 m/s。 */
    Quantity<MetersPerSecond> speed_mps{};
    AttitudeState attitude{};
    VectorLosEstimate vector_los{};
};

struct GuidanceDiagnostics {
    // 历史字段名：实际保存弹道系指令比力，而非净运动学导航加速度。
    AccelerationBallisticMps2 navigation_acceleration{};
    // 限幅后的镖体系指令比力（m/s²）；仅矢量 PN 在输出端另加横向重力补偿。
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
