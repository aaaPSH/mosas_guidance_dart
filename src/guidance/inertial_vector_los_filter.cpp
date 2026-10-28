#include <mosas/guidance/inertial_vector_los_filter.hpp>

#include <algorithm>
#include <cmath>

namespace mosas::guidance {
namespace {

constexpr double kEpsilon = 1e-12;
constexpr double kPoleThreshold = 1e-6;

bool finite(double v) noexcept { return std::isfinite(v); }

bool valid_rotation_matrix(const std::array<double, 9>& m) noexcept {
    for (double v : m) {
        if (!finite(v)) return false;
    }
    // 检查第 0, 1, 2 行的模长接近 1
    const double r0 = std::hypot(std::hypot(m[0], m[1]), m[2]);
    const double r1 = std::hypot(std::hypot(m[3], m[4]), m[5]);
    const double r2 = std::hypot(std::hypot(m[6], m[7]), m[8]);
    if (std::abs(r0 - 1.0) > 1e-4 || std::abs(r1 - 1.0) > 1e-4 || std::abs(r2 - 1.0) > 1e-4) {
        return false;
    }
    return true;
}

}  // namespace

InertialVectorLosFilter::InertialVectorLosFilter(
    const InertialVectorLosFilterConfig& config)
    : config_(valid_config(config) ? config : InertialVectorLosFilterConfig{}) {
    reset();
}

bool InertialVectorLosFilter::valid_config(
    const InertialVectorLosFilterConfig& config) noexcept {
    return finite(config.sigma_a) && config.sigma_a > 0.0 &&
           finite(config.sigma_l) && config.sigma_l > 0.0 &&
           finite(config.initial_rate_covariance) && config.initial_rate_covariance >= 0.0 &&
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

void InertialVectorLosFilter::project_to_manifold() noexcept {
    const double n = std::hypot(std::hypot(direction_[0], direction_[1]), direction_[2]);
    if (!finite(n) || n < kEpsilon) {
        direction_ = {1.0, 0.0, 0.0};
        rate_ = {0.0, 0.0, 0.0};
        return;
    }
    const double inv_n = 1.0 / n;
    direction_[0] *= inv_n;
    direction_[1] *= inv_n;
    direction_[2] *= inv_n;

    const double radial_rate = direction_[0] * rate_[0] +
                               direction_[1] * rate_[1] +
                               direction_[2] * rate_[2];
    rate_[0] = (rate_[0] - radial_rate * direction_[0]) * inv_n;
    rate_[1] = (rate_[1] - radial_rate * direction_[1]) * inv_n;
    rate_[2] = (rate_[2] - radial_rate * direction_[2]) * inv_n;
}

domain::LosRateEstimate InertialVectorLosFilter::update(
    const domain::LosRateFilterInput& input) {
    if (!input.line_of_sight.valid || !input.attitude.valid ||
        !finite(input.line_of_sight.q_y.value) ||
        !finite(input.line_of_sight.q_z.value) ||
        !finite(input.dt_seconds) || input.dt_seconds < 0.0) {
        reset();
        return {};
    }

    const auto& m = input.attitude.navigation_from_dart.matrix();
    if (!valid_rotation_matrix(m)) {
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

    // 协方差先验传播
    const double p00_pred = p00_ + 2.0 * dt * p01_ + dt2 * p11_ + q_scale * (dt4 / 4.0);
    const double p01_pred = p01_ + dt * p11_ + q_scale * (dt3 / 2.0);
    const double p11_pred = p11_ + q_scale * dt2;

    // 测量更新
    const double r = config_.sigma_l * config_.sigma_l;
    const double s = p00_pred + r;
    if (!finite(s) || s < kEpsilon) {
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
    p00_ = (1.0 - k0) * (1.0 - k0) * p00_pred + k0 * k0 * r;
    p01_ = (1.0 - k0) * p01_pred - k0 * k1 * r;
    p11_ = p11_pred - k1 * p01_pred + k1 * k1 * r;

    // 流形单位化与正交截断
    project_to_manifold();

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

    // 6. 构造镖体系矢量状态输出 (R_{D<-N} = R_{N<-D}^T)
    const std::array<double, 3> dart_direction{
        m[0] * direction_[0] + m[3] * direction_[1] + m[6] * direction_[2],
        m[1] * direction_[0] + m[4] * direction_[1] + m[7] * direction_[2],
        m[2] * direction_[0] + m[5] * direction_[1] + m[8] * direction_[2]};
    const std::array<double, 3> dart_rate{
        m[0] * rate_[0] + m[3] * rate_[1] + m[6] * rate_[2],
        m[1] * rate_[0] + m[4] * rate_[1] + m[7] * rate_[2],
        m[2] * rate_[0] + m[5] * rate_[1] + m[8] * rate_[2]};

    domain::LosRateEstimate estimate{};
    estimate.valid = true;
    estimate.angular_velocity.valid = rate_valid;
    estimate.angular_velocity.q_y.value = dot_q_y;
    estimate.angular_velocity.q_z.value = dot_q_z;
    estimate.vector.valid = true;
    estimate.vector.direction = {dart_direction[0], dart_direction[1], dart_direction[2]};
    estimate.vector.inertial_direction_rate = {dart_rate[0], dart_rate[1], dart_rate[2]};

    sample_.line_of_sight = input.line_of_sight;
    sample_.filtered_angular_velocity = estimate.angular_velocity;
    sample_.raw_angular_velocity = estimate.angular_velocity;

    return estimate;
}

}  // namespace mosas::guidance
