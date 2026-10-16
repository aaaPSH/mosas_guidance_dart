#ifndef MOSAS_BODY_VECTOR_TEST_SUPPORT_HPP
#define MOSAS_BODY_VECTOR_TEST_SUPPORT_HPP

#include <mosas/domain/algorithm_results.hpp>

#include <array>
#include <cmath>

namespace mosas::test {

/** @brief 构造 R_N_from_D=R_Y(-yaw) R_Z(pitch) R_X(roll) 的有效测试姿态。 */
inline domain::AttitudeState vector_test_attitude(double pitch = 0.0, double roll = 0.0,
                                                  double yaw = 0.0) {
    const double cp = std::cos(pitch), sp = std::sin(pitch);
    const double cr = std::cos(roll), sr = std::sin(roll);
    const double cy = std::cos(yaw), sy = std::sin(yaw);
    domain::AttitudeState result;
    result.valid = true;
    result.pitch_rad.value = pitch;
    result.roll_rad.value = roll;
    result.navigation_from_dart =
        domain::Rotation3<domain::NavigationFrameTag, domain::DartFrameTag>(
            {cy * cp, -cy * sp * cr - sy * sr, cy * sp * sr - sy * cr, sp, cp * cr, -cp * sr,
             sy * cp, -sy * sp * cr + cy * sr, sy * sp * sr + cy * cr});
    return result;
}

/** @brief 将同一导航系物理向量转换到给定测试姿态的镖体系。 */
inline std::array<double, 3> to_test_dart(const domain::AttitudeState &attitude,
                                          const std::array<double, 3> &value) {
    const auto &r = attitude.navigation_from_dart.matrix();
    return {r[0] * value[0] + r[3] * value[1] + r[6] * value[2],
            r[1] * value[0] + r[4] * value[1] + r[7] * value[2],
            r[2] * value[0] + r[5] * value[1] + r[8] * value[2]};
}

/** @brief 将镖体系结果转换回同一导航系，用于检验旋转一致性。 */
inline std::array<double, 3> to_test_navigation(const domain::AttitudeState &attitude,
                                                const std::array<double, 3> &value) {
    const auto &r = attitude.navigation_from_dart.matrix();
    return {r[0] * value[0] + r[1] * value[1] + r[2] * value[2],
            r[3] * value[0] + r[4] * value[1] + r[5] * value[2],
            r[6] * value[0] + r[7] * value[1] + r[8] * value[2]};
}

/** @brief 将测试单位方向表示为现有 LOS 俯仰/偏航观测角。 */
inline domain::LineOfSight test_los(const std::array<double, 3> &direction) {
    return {true,
            {std::atan2(direction[1], std::hypot(direction[0], direction[2]))},
            {std::atan2(direction[2], direction[0])}};
}

} // namespace mosas::test
#endif
