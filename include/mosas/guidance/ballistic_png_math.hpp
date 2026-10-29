#ifndef MOSAS_GUIDANCE_BALLISTIC_PNG_MATH_HPP
#define MOSAS_GUIDANCE_BALLISTIC_PNG_MATH_HPP
#include <mosas/domain/contracts.hpp>

namespace mosas::guidance {
/** @brief 曝光时刻弹道 PNG 纯计算结果；不含最终输出姿态或限幅结果。 */
struct PngCalculationTrace {
    bool valid = false;
    domain::GuidanceInvalidReason invalid_reason = domain::GuidanceInvalidReason::algorithm_rejected;
    double inertial_los_elevation_rad = 0;
    double measured_elevation_rate_rad_s = 0;
    double measured_lateral_rate_rad_s = 0;
    double parabolic_elevation_rate_rad_s = 0;
    double compensated_elevation_rate_rad_s = 0;
    domain::BallisticOverloadG requested_overload{};
};
/**
 * @brief 从导航系单位视线与切向导数解析未限幅弹道请求。
 * @param direction 导航系单位方向。
 * @param direction_rate 导航系切向导数，单位 1/s。
 * @param speed_mps 曝光时刻实际速度，须为有限正数。
 * @param navigation_constant_y 纵向正导航常数。
 * @param navigation_constant_z 横向正导航常数。
 * @param gravity 共用的正重力常数，单位 m/s²。
 * @return 极点、非法参数或数值异常时返回无效零请求；不进行 Roll 或限幅。
 */
PngCalculationTrace evaluate_ballistic_png(
    const domain::DirectionNavigationUnit& direction,
    const domain::DirectionRateNavigationPerSec& direction_rate,
    double speed_mps, double navigation_constant_y,
    double navigation_constant_z, double gravity) noexcept;
}
#endif
