#ifndef MOSAS_GUIDANCE_ROTATION_VALIDATION_HPP
#define MOSAS_GUIDANCE_ROTATION_VALIDATION_HPP

#include <array>
#include <cmath>

namespace mosas::guidance::detail {

/**
 * @brief 验证旋转属于 SO(3)，拒绝重复单位行和反射矩阵。
 * @param m 按行存储的旋转矩阵。
 * @return 所有元素有限、行正交且行列式为 +1 时为 true。
 */
inline bool valid_rotation(const std::array<double, 9>& m) noexcept {
    for (double v : m) if (!std::isfinite(v)) return false;
    constexpr double tolerance = 1e-6;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j <= i; ++j) {
            double dot = 0.0;
            for (int k = 0; k < 3; ++k) dot += m[3 * i + k] * m[3 * j + k];
            if (std::abs(dot - (i == j ? 1.0 : 0.0)) > tolerance) return false;
        }
    const double det = m[0] * (m[4] * m[8] - m[5] * m[7]) -
                       m[1] * (m[3] * m[8] - m[5] * m[6]) +
                       m[2] * (m[3] * m[7] - m[4] * m[6]);
    return std::abs(det - 1.0) <= tolerance;
}

}  // namespace mosas::guidance::detail
#endif
