#include <mosas/guidance/inertial_vector_los_filter.hpp>

#include "vector_los_math.hpp"

#include <cmath>
#include <stdexcept>

namespace mosas::guidance {
namespace {

constexpr double kEpsilon = 1e-12;
constexpr double kPoleThreshold = 1e-6;

bool finite(double v) noexcept { return std::isfinite(v); }

}  // namespace

InertialVectorLosFilter::InertialVectorLosFilter(
    const InertialVectorLosFilterConfig& config)
    : config_(config) {
    if (!valid_config(config)) throw std::invalid_argument("惯性 LOS 滤波配置无效");
    reset();
}

bool InertialVectorLosFilter::valid_config(
    const InertialVectorLosFilterConfig& config) noexcept {
    return finite(config.sigma_a * config.sigma_a) && config.sigma_a > 0.0 &&
           config.sigma_a * config.sigma_a > 0.0 &&
           finite(config.sigma_l * config.sigma_l) && config.sigma_l > 0.0 &&
           config.sigma_l * config.sigma_l > 0.0 &&
           finite(config.initial_rate_covariance) && config.initial_rate_covariance > 0.0 &&
           finite(config.maximum_gap_seconds) && config.maximum_gap_seconds > 0.0;
}

void InertialVectorLosFilter::reset() noexcept {
    direction_ = {1.0, 0.0, 0.0};
    rate_ = {0.0, 0.0, 0.0};
    p00_ = config_.sigma_l * config_.sigma_l;
    p01_ = 0.0;
    p11_ = config_.initial_rate_covariance;
    initialized_ = false;
    generation_ = 0;
    sample_ = {};
}

domain::LineOfSightRateSample InertialVectorLosFilter::sample() const {
    return sample_;
}

bool InertialVectorLosFilter::project_to_manifold() noexcept {
    return detail::project_to_sphere(direction_, rate_);
}

domain::LosRateEstimate InertialVectorLosFilter::update(
    const domain::LosRateFilterInput& input) {
    if (!input.line_of_sight.valid || !input.attitude.valid ||
        !finite(input.line_of_sight.q_y.value) ||
        !finite(input.line_of_sight.q_z.value) ||
        !finite(input.dt_seconds) || input.dt_seconds < 0.0 ||
        (initialized_ && input.dt_seconds == 0.0)) {
        reset();
        return {};
    }

    const auto& m = input.attitude.navigation_from_dart.matrix();
    if (!detail::valid_rotation(m)) {
        reset();
        return {};
    }

    // 1. 将镖体系视线转换为单位矢量 l_D
    const double cy = std::cos(input.line_of_sight.q_y.value);
    const double sy = std::sin(input.line_of_sight.q_y.value);
    const double cz = std::cos(input.line_of_sight.q_z.value);
    const double sz = std::sin(input.line_of_sight.q_z.value);
    const std::array<double, 3> l_D{cy * cz, sy, cy * sz};

    // 2. 曝光时刻姿态投影到局部导航惯性系 z_k = R_{N<-D} * l_D
    const std::array<double, 3> z{
        m[0] * l_D[0] + m[1] * l_D[1] + m[2] * l_D[2],
        m[3] * l_D[0] + m[4] * l_D[1] + m[5] * l_D[2],
        m[6] * l_D[0] + m[7] * l_D[1] + m[8] * l_D[2]};

    // 3. 初始化或超时断流处理
    const bool timeout = initialized_ && input.dt_seconds > config_.maximum_gap_seconds;
    const bool generation_changed = initialized_ && input.generation != generation_;
    if (!initialized_ || timeout || generation_changed) {
        const double z_norm = std::hypot(std::hypot(z[0], z[1]), z[2]);
        if (!finite(z_norm) || z_norm < kEpsilon) {
            reset();
            return {};
        }
        direction_ = {z[0] / z_norm, z[1] / z_norm, z[2] / z_norm};
        rate_ = {0.0, 0.0, 0.0};
        p00_ = config_.sigma_l * config_.sigma_l;
        p01_ = 0.0;
        p11_ = config_.initial_rate_covariance;
        initialized_ = true;
        generation_ = input.generation;
        sample_ = {input.line_of_sight, {}, {}};
        return {}; // 首帧仅建基准，不输出角速率
    }

    // 4. 卡尔曼滤波更新 (DWNA 匀速模型)
    const double dt = input.dt_seconds;
    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;
    const double dt4 = dt2 * dt2;
    const double q_scale = config_.sigma_a * config_.sigma_a;

    // 状态先验传播
    direction_[0] += dt * rate_[0];
    direction_[1] += dt * rate_[1];
    direction_[2] += dt * rate_[2];
    if (!project_to_manifold()) {
        reset();
        return {};
    }

    // 协方差先验传播
    const double p00_pred = p00_ + 2.0 * dt * p01_ + dt2 * p11_ + q_scale * (dt4 / 4.0);
    const double p01_pred = p01_ + dt * p11_ + q_scale * (dt3 / 2.0);
    const double p11_pred = p11_ + q_scale * dt2;

    // 测量更新
    const double r = config_.sigma_l * config_.sigma_l;
    const double s = p00_pred + r;
    if (!finite(s) || s <= 0.0 ||
        !detail::positive_covariance(p00_pred, p01_pred, p11_pred)) {
        reset();
        return {};
    }
    const double k0 = p00_pred / s;
    const double k1 = p01_pred / s;

    const double dy0 = z[0] - direction_[0];
    const double dy1 = z[1] - direction_[1];
    const double dy2 = z[2] - direction_[2];

    direction_[0] += k0 * dy0;
    direction_[1] += k0 * dy1;
    direction_[2] += k0 * dy2;

    rate_[0] += k1 * dy0;
    rate_[1] += k1 * dy1;
    rate_[2] += k1 * dy2;

    // 协方差 Joseph 形式更新
    const double a = 1.0 - k0;
    p00_ = a * a * p00_pred + k0 * k0 * r;
    p01_ = a * (p01_pred - k1 * p00_pred) + k0 * k1 * r;
    p11_ = p11_pred - 2.0 * k1 * p01_pred + k1 * k1 * (p00_pred + r);

    if (!project_to_manifold() || !detail::positive_covariance(p00_, p01_, p11_)) {
        reset();
        return {};
    }

    // 5. 解析求取惯性视线角速度
    const double rho_xz = std::hypot(direction_[0], direction_[2]);
    const bool rate_valid = rho_xz >= kPoleThreshold;

    double dot_q_y = 0.0;
    double dot_q_z = 0.0;
    if (rate_valid) {
        dot_q_y = rate_[1] / rho_xz;
        // 横向有效视线角速度（右手系修正符号）
        dot_q_z = (direction_[0] * rate_[2] - direction_[2] * rate_[0]) / rho_xz;
    }

    // 保持导航系状态，不借用任何输出时刻的姿态重解释历史观测。
    domain::LosRateEstimate estimate{};
    estimate.valid = true;
    estimate.angular_velocity.valid = rate_valid;
    estimate.angular_velocity.q_y.value = dot_q_y;
    estimate.angular_velocity.q_z.value = dot_q_z;
    estimate.vector.valid = true;
    estimate.vector.direction = {direction_[0], direction_[1], direction_[2]};
    estimate.vector.inertial_direction_rate = {rate_[0], rate_[1], rate_[2]};

    sample_.line_of_sight = input.line_of_sight;
    sample_.filtered_angular_velocity = estimate.angular_velocity;
    sample_.raw_angular_velocity = estimate.angular_velocity;

    return estimate;
}

}  // namespace mosas::guidance
