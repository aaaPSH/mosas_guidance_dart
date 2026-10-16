#include <mosas/guidance/body_vector_los_filter.hpp>

#include "body_vector_math.hpp"

#include <algorithm>
#include <cmath>

namespace mosas::guidance {

BodyVectorLosFilter::BodyVectorLosFilter(const BodyVectorLosFilterConfig &config)
    : config_(config) {}

bool BodyVectorLosFilter::valid_config(const BodyVectorLosFilterConfig &c) noexcept {
    const double process_variance = c.sigma_a * c.sigma_a;
    const double measurement_variance = c.sigma_l * c.sigma_l;
    return std::isfinite(c.sigma_a) && c.sigma_a > 0.0 && std::isfinite(process_variance) &&
           process_variance > 0.0 && std::isfinite(c.sigma_l) && c.sigma_l > 0.0 &&
           std::isfinite(measurement_variance) && measurement_variance > 0.0 &&
           std::isfinite(c.initial_rate_covariance) && c.initial_rate_covariance >= 0.0 &&
           std::isfinite(c.maximum_gap_seconds) && c.maximum_gap_seconds > 0.0;
}

void BodyVectorLosFilter::reset() noexcept {
    direction_ = inertial_rate_ = previous_measurement_ = {};
    covariance_ = {};
    previous_attitude_ = {};
    generation_ = 0;
    initialized_ = false;
    sample_ = {};
}

domain::LineOfSightRateSample BodyVectorLosFilter::sample() const { return sample_; }

bool BodyVectorLosFilter::initialize(const Vec3 &measurement,
                                     const domain::LosRateFilterInput &input) {
    direction_ = previous_measurement_ = measurement;
    inertial_rate_ = {};
    previous_attitude_ = input.attitude;
    generation_ = input.generation;
    covariance_ = {};
    for (std::size_t i = 0; i < 3; ++i) {
        covariance_[i][i] = config_.sigma_l * config_.sigma_l;
        covariance_[i + 3][i + 3] = config_.initial_rate_covariance;
    }
    initialized_ = constrain_state();
    return initialized_;
}

bool BodyVectorLosFilter::constrain_state() {
    const double length = detail::norm(direction_);
    if (!detail::finite(direction_) || !detail::finite(inertial_rate_) ||
        !detail::finite(covariance_) || !std::isfinite(length) || length < 1e-12)
        return false;
    Vec3 h{};
    for (std::size_t i = 0; i < 3; ++i)
        h[i] = direction_[i] / length;
    const double radial_rate = detail::dot(h, inertial_rate_);
    Covariance jacobian{};
    for (std::size_t i = 0; i < 3; ++i) {
        for (std::size_t j = 0; j < 3; ++j) {
            const double identity = i == j ? 1.0 : 0.0;
            const double tangent = identity - h[i] * h[j];
            jacobian[i][j] = jacobian[i + 3][j + 3] = tangent / length;
            // 导数归一化 u'=u/s-l(l·u)/s³ 的 ∂u'/∂l，保留位置/速率交叉协方差。
            jacobian[i + 3][j] = (-inertial_rate_[i] * h[j] - radial_rate * identity -
                                  h[i] * inertial_rate_[j] + 3.0 * radial_rate * h[i] * h[j]) /
                                 (length * length);
        }
    }
    covariance_ =
        detail::multiply(detail::multiply(jacobian, covariance_), detail::transpose(jacobian));
    for (std::size_t i = 0; i < 3; ++i)
        inertial_rate_[i] = (inertial_rate_[i] - h[i] * radial_rate) / length;
    direction_ = h;
    if (!detail::finite(covariance_) || !detail::finite(inertial_rate_))
        return false;
    double largest_diagonal = 0.0;
    for (std::size_t i = 0; i < 6; ++i)
        largest_diagonal = std::max(largest_diagonal, covariance_[i][i]);
    for (std::size_t i = 0; i < 6; ++i) {
        if (covariance_[i][i] < -1e-12 * std::max(1.0, largest_diagonal))
            return false;
        covariance_[i][i] = std::max(0.0, covariance_[i][i]);
        for (std::size_t j = 0; j < i; ++j) {
            const double symmetric = 0.5 * covariance_[i][j] + 0.5 * covariance_[j][i];
            covariance_[i][j] = covariance_[j][i] = symmetric;
        }
    }
    return true;
}

domain::LosRateEstimate BodyVectorLosFilter::update(const domain::LosRateFilterInput &input) {
    Vec3 measurement{};
    const auto rotation = detail::attitude_matrix(input.attitude);
    if (!valid_config(config_) || !input.attitude.valid || !detail::valid_rotation(rotation) ||
        !detail::los_direction(input.line_of_sight, &measurement) ||
        !std::isfinite(input.dt_seconds) || input.dt_seconds < 0.0) {
        reset();
        return {};
    }
    sample_ = {input.line_of_sight, {}, {}};
    if (!initialized_ || generation_ != input.generation ||
        input.dt_seconds > config_.maximum_gap_seconds) {
        // 第一帧和断段后的当前帧仅建基准；下一连续帧再输出速率。
        if (!initialize(measurement, input))
            reset();
        return {};
    }
    if (input.dt_seconds <= 0.0) {
        reset();
        return {};
    }

    const double dt = input.dt_seconds;
    const auto transport =
        detail::multiply(detail::transpose(rotation), detail::attitude_matrix(previous_attitude_));
    const auto old_measurement = detail::apply(transport, previous_measurement_);
    Vec3 raw_rate{};
    for (std::size_t i = 0; i < 3; ++i)
        raw_rate[i] = (measurement[i] - old_measurement[i]) / dt;
    const double radial_raw = detail::dot(measurement, raw_rate);
    for (std::size_t i = 0; i < 3; ++i)
        raw_rate[i] -= measurement[i] * radial_raw;
    sample_.raw_angular_velocity = detail::angle_rates(measurement, raw_rate);

    // C 同时运输位置与惯性导数；它已经补偿转动，不再叠加陀螺仪叉乘项。
    Vec3 extrapolated{};
    for (std::size_t i = 0; i < 3; ++i)
        extrapolated[i] = direction_[i] + dt * inertial_rate_[i];
    direction_ = detail::apply(transport, extrapolated);
    inertial_rate_ = detail::apply(transport, inertial_rate_);
    Covariance transition{};
    for (std::size_t i = 0; i < 3; ++i) {
        for (std::size_t j = 0; j < 3; ++j) {
            transition[i][j] = transition[i + 3][j + 3] = transport[i][j];
            transition[i][j + 3] = dt * transport[i][j];
        }
    }
    covariance_ =
        detail::multiply(detail::multiply(transition, covariance_), detail::transpose(transition));
    const double dt2 = dt * dt;
    const double variance = config_.sigma_a * config_.sigma_a;
    const double q00 = variance * dt2 * dt2 / 4.0;
    const double q01 = variance * dt2 * dt / 2.0;
    const double q11 = variance * dt2;
    for (std::size_t i = 0; i < 3; ++i) {
        covariance_[i][i] += q00;
        covariance_[i][i + 3] += q01;
        covariance_[i + 3][i] += q01;
        covariance_[i + 3][i + 3] += q11;
    }
    if (!detail::finite(covariance_) || !detail::finite(direction_) ||
        !detail::finite(inertial_rate_)) {
        reset();
        return {};
    }

    const double measurement_variance = config_.sigma_l * config_.sigma_l;
    detail::Mat3 innovation_covariance{};
    for (std::size_t i = 0; i < 3; ++i)
        for (std::size_t j = 0; j < 3; ++j)
            innovation_covariance[i][j] = covariance_[i][j] + (i == j ? measurement_variance : 0.0);
    detail::Mat3 lower{};
    if (!detail::cholesky(innovation_covariance, &lower)) {
        reset();
        return {};
    }
    detail::Matrix<6, 3> gain{};
    for (std::size_t i = 0; i < 6; ++i)
        gain[i] = detail::solve_cholesky(lower,
                                         {covariance_[i][0], covariance_[i][1], covariance_[i][2]});
    const Vec3 innovation{measurement[0] - direction_[0], measurement[1] - direction_[1],
                          measurement[2] - direction_[2]};
    for (std::size_t i = 0; i < 3; ++i) {
        direction_[i] += detail::dot(gain[i], innovation);
        inertial_rate_[i] += detail::dot(gain[i + 3], innovation);
    }

    // Joseph 形式：(I-KH)P(I-KH)ᵀ + KRKᵀ，避免减法导致协方差失去正半定性。
    Covariance residual{};
    for (std::size_t i = 0; i < 6; ++i) {
        residual[i][i] = 1.0;
        for (std::size_t j = 0; j < 3; ++j)
            residual[i][j] -= gain[i][j];
    }
    covariance_ =
        detail::multiply(detail::multiply(residual, covariance_), detail::transpose(residual));
    for (std::size_t i = 0; i < 6; ++i)
        for (std::size_t j = 0; j < 6; ++j)
            covariance_[i][j] += measurement_variance * detail::dot(gain[i], gain[j]);
    if (!constrain_state()) {
        reset();
        return {};
    }
    previous_measurement_ = measurement;
    previous_attitude_ = input.attitude;
    sample_.filtered_angular_velocity = detail::angle_rates(direction_, inertial_rate_);
    return {true,
            sample_.filtered_angular_velocity,
            {true,
             {direction_[0], direction_[1], direction_[2]},
             {inertial_rate_[0], inertial_rate_[1], inertial_rate_[2]}}};
}

} // namespace mosas::guidance
