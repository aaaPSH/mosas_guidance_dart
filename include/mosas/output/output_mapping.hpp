#ifndef MOSAS_OUTPUT_OUTPUT_MAPPING_HPP
#define MOSAS_OUTPUT_OUTPUT_MAPPING_HPP

#include <mosas/domain/algorithm_results.hpp>
#include <mosas/domain/contracts.hpp>
#include <mosas/vision/vision_types.hpp>

namespace mosas::output::detail {

/** @brief 同一完整姿态的观测用欧拉角，单位 rad；不反馈到制导。 */
struct AttitudeAngles {
    bool valid = false;
    double roll_rad = 0.0;
    double pitch_rad = 0.0;
    double yaw_rad = 0.0;
};

/**
 * @brief 按 R_N_from_D=R_Y(-yaw) R_Z(pitch) R_X(roll) 提取三轴姿态角。
 * @param attitude 指定时刻的姿态；有效输入的矩阵须为正交旋转矩阵。
 * @return 观测角；无效或非有限姿态返回 valid=false。俯仰极点取 yaw=0 的等价姿态。
 */
AttitudeAngles make_attitude_angles(const domain::AttitudeState& attitude) noexcept;

/**
 * @brief 将当前帧目标检测转换为绘图数据。
 * @param detection 当前相机帧的检测快照。
 * @return 不含制导状态的目标绘图数据。
 */
VisionResult make_legacy_vision_result(
    const domain::VisionDetection& detection);

/**
 * @brief 将导航惯性系 LOS 和曝光时刻完整姿态转换为显示数据。
 * @param overlay 当前帧的 LOS、真实输出快照及处理帧率。
 * @param state 插值到同一曝光时刻的飞行状态，不查询最新 IMU。
 * @return 角度为 rad、角速度为 rad/s 的显示快照。
 */
VisionOverlayData make_legacy_overlay(
    const domain::GuidanceOverlay& overlay,
    const domain::FlightStateSnapshot& state);

}  // namespace mosas::output::detail

#endif  // MOSAS_OUTPUT_OUTPUT_MAPPING_HPP
