#ifndef MOSAS_BODY_VECTOR_MATH_HPP
#define MOSAS_BODY_VECTOR_MATH_HPP

#include <mosas/domain/algorithm_results.hpp>

#include <array>
#include <cmath>
#include <cstddef>

namespace mosas::guidance::detail {

using Vec3 = std::array<double, 3>;
template <std::size_t Rows, std::size_t Cols>
using Matrix = std::array<std::array<double, Cols>, Rows>;
using Mat3 = Matrix<3, 3>;

inline double dot(const Vec3 &a, const Vec3 &b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
inline double norm(const Vec3 &a) { return std::hypot(std::hypot(a[0], a[1]), a[2]); }
inline Vec3 cross(const Vec3 &a, const Vec3 &b) {
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}
inline bool finite(const Vec3 &v) {
    return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}
template <class Frame, class Unit> Vec3 components(const domain::Vector3<Frame, Unit> &v) {
    return {v.x, v.y, v.z};
}

template <std::size_t Rows, std::size_t Cols> bool finite(const Matrix<Rows, Cols> &a) {
    for (const auto &row : a)
        for (double v : row)
            if (!std::isfinite(v))
                return false;
    return true;
}
template <std::size_t Rows, std::size_t Cols>
Matrix<Cols, Rows> transpose(const Matrix<Rows, Cols> &a) {
    Matrix<Cols, Rows> result{};
    for (std::size_t i = 0; i < Rows; ++i)
        for (std::size_t j = 0; j < Cols; ++j)
            result[j][i] = a[i][j];
    return result;
}
template <std::size_t Rows, std::size_t Inner, std::size_t Cols>
Matrix<Rows, Cols> multiply(const Matrix<Rows, Inner> &a, const Matrix<Inner, Cols> &b) {
    Matrix<Rows, Cols> result{};
    for (std::size_t i = 0; i < Rows; ++i)
        for (std::size_t j = 0; j < Cols; ++j)
            for (std::size_t k = 0; k < Inner; ++k)
                result[i][j] += a[i][k] * b[k][j];
    return result;
}
inline Vec3 apply(const Mat3 &r, const Vec3 &v) {
    return {dot(r[0], v), dot(r[1], v), dot(r[2], v)};
}
inline Mat3 attitude_matrix(const domain::AttitudeState &attitude) {
    const auto &m = attitude.navigation_from_dart.matrix();
    return {{{m[0], m[1], m[2]}, {m[3], m[4], m[5]}, {m[6], m[7], m[8]}}};
}

/**
 * @brief 检查有限、正交且 det=+1 的旋转矩阵。
 * @param r 待校验旋转，行/列均应为单位正交基。
 * @return 所有条件在 1e-6 容差内满足时为 true。
 */
inline bool valid_rotation(const Mat3 &r) {
    if (!finite(r))
        return false;
    constexpr double tolerance = 1e-6;
    for (std::size_t i = 0; i < 3; ++i) {
        if (std::abs(norm(r[i]) - 1.0) > tolerance)
            return false;
        for (std::size_t j = 0; j < i; ++j)
            if (std::abs(dot(r[i], r[j])) > tolerance)
                return false;
    }
    return std::abs(dot(r[0], cross(r[1], r[2])) - 1.0) <= tolerance;
}

/**
 * @brief 将角度观测转换为单位方向。
 * @param los 镖体系 LOS 角观测。
 * @param direction 非空方向输出指针。
 * @return 观测有效且数值有限时为 true。
 */
inline bool los_direction(const domain::LineOfSight &los, Vec3 *direction) {
    if (!los.valid || !std::isfinite(los.q_y.value) || !std::isfinite(los.q_z.value))
        return false;
    const double c = std::cos(los.q_y.value);
    *direction = {c * std::cos(los.q_z.value), std::sin(los.q_y.value),
                  c * std::sin(los.q_z.value)};
    return finite(*direction);
}

/**
 * @brief 将镖体系惯性方向导数投影到 LOS 角度诊断通道。
 * @param l 单位 LOS。
 * @param u 同一坐标系下的惯性导数（1/s）。
 * @return 惯性角速率诊断；接近仰角极点时无效，不影响矢量制导。
 */
inline domain::LineOfSightAngularVelocity angle_rates(const Vec3 &l, const Vec3 &u) {
    const double horizontal = std::hypot(l[0], l[2]);
    if (!finite(l) || !finite(u) || horizontal < 1e-8)
        return {};
    const Vec3 elevation{-l[1] * l[0] / horizontal, horizontal, -l[1] * l[2] / horizontal};
    const double q_y = dot(elevation, u);
    const double q_z = (l[0] * u[2] - l[2] * u[0]) / (horizontal * horizontal);
    if (!std::isfinite(q_y) || !std::isfinite(q_z))
        return {};
    return {true, {q_y}, {q_z}};
}

/**
 * @brief Cholesky 分解 3×3 正定系统。
 * @param a 对称正定输入矩阵。
 * @param lower 非空输出指针，成功时 a=lower*lowerᵀ。
 * @return 输入正定且计算有限时为 true。
 */
inline bool cholesky(const Mat3 &a, Mat3 *lower) {
    *lower = {};
    for (std::size_t i = 0; i < 3; ++i) {
        for (std::size_t j = 0; j <= i; ++j) {
            double value = a[i][j];
            for (std::size_t k = 0; k < j; ++k)
                value -= (*lower)[i][k] * (*lower)[j][k];
            if (!std::isfinite(value) || (i == j && value <= 0.0))
                return false;
            (*lower)[i][j] = i == j ? std::sqrt(value) : value / (*lower)[j][j];
        }
    }
    return finite(*lower);
}

/**
 * @brief 用 Cholesky 因子解 L Lᵀ x=b，不显式计算逆矩阵。
 * @param lower 已校验的 Cholesky 下三角因子。
 * @param rhs 右端向量 b。
 * @return 解 x，调用者须检查数值有限性。
 */
inline Vec3 solve_cholesky(const Mat3 &lower, const Vec3 &rhs) {
    Vec3 result{}, intermediate{};
    for (std::size_t i = 0; i < 3; ++i) {
        double value = rhs[i];
        for (std::size_t j = 0; j < i; ++j)
            value -= lower[i][j] * intermediate[j];
        intermediate[i] = value / lower[i][i];
    }
    for (int i = 2; i >= 0; --i) {
        double value = intermediate[i];
        for (int j = i + 1; j < 3; ++j)
            value -= lower[j][i] * result[j];
        result[i] = value / lower[i][i];
    }
    return result;
}

} // namespace mosas::guidance::detail
#endif
