#ifndef MOSAS_GUIDANCE_VECTOR_LOS_MATH_HPP
#define MOSAS_GUIDANCE_VECTOR_LOS_MATH_HPP

#include <array>
#include <cmath>

namespace mosas::guidance::detail {

/** @return 三个分量均为有限数时为 true。 */
inline bool finite_vector(const std::array<double, 3>& v) noexcept {
    return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

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

/**
 * @brief 单位化方向并将其导数投影到切空间，同时保留单位化的缩放。
 * @param l 输入/输出方向。
 * @param u 输入/输出方向导数。
 * @return 状态有限且方向不退化时为 true；失败值不得继续作为有效估计使用。
 */
inline bool project_to_sphere(std::array<double, 3>& l,
                              std::array<double, 3>& u) noexcept {
    if (!finite_vector(l) || !finite_vector(u)) return false;
    const double n = std::hypot(std::hypot(l[0], l[1]), l[2]);
    if (!std::isfinite(n) || n < 1e-12) return false;
    for (double& v : l) v /= n;
    const double radial = l[0] * u[0] + l[1] * u[1] + l[2] * u[2];
    for (int i = 0; i < 3; ++i) u[i] = (u[i] - radial * l[i]) / n;
    return finite_vector(l) && finite_vector(u);
}

/** @return 有限对称 2×2 协方差满足严格正定条件时为 true。 */
inline bool positive_covariance(double p00, double p01, double p11) noexcept {
    return std::isfinite(p00) && std::isfinite(p01) && std::isfinite(p11) &&
           p00 > 0.0 && p11 > 0.0 && std::fma(-p01 / p00, p01, p11) > 0.0;
}

}  // namespace mosas::guidance::detail
#endif
