#include <mosas/guidance/inertial_angle_los_filter.hpp>

#include "rotation_validation.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace mosas::guidance {
namespace {

constexpr double kEpsilon = 1e-12;
constexpr double kPoleThreshold = 1e-6;
constexpr double kPi = 3.14159265358979323846;
constexpr double kMinEstimatedDistance = 1.0;

/** @param angle 有限角度，rad。 @return [-pi, pi) 内的等价角度。 */
double normalize_angle(double angle) noexcept {
    double result = std::fmod(angle + kPi, 2.0 * kPi);
    if (result < 0.0) result += 2.0 * kPi;
    return result - kPi;
}

/** @param sigma 噪声标准差。 @return 平方有限、非零且标准差为正时为 true。 */
bool valid_noise(double sigma) noexcept {
    return sigma > 0.0 && std::isfinite(sigma * sigma) && sigma * sigma > 0.0;
}

}  // namespace

InertialAngleLosFilter::InertialAngleLosFilter(const InertialAngleLosFilterConfig& config)
    : config_(config) {
    if (!valid_config(config)) throw std::invalid_argument("惯性 LOS 角度滤波配置无效");
    reset();
}

bool InertialAngleLosFilter::valid_config(const InertialAngleLosFilterConfig& config) noexcept {
    return valid_noise(config.sigma_a) && valid_noise(config.sigma_q) &&
           (config.sigma_a_pitch == 0.0 || valid_noise(config.sigma_a_pitch)) &&
           (config.sigma_a_yaw == 0.0 || valid_noise(config.sigma_a_yaw)) &&
           std::isfinite(config.initial_covariance) && config.initial_covariance >= 0.0 &&
           std::isfinite(config.maximum_gap_seconds) && config.maximum_gap_seconds > 0.0 &&
           std::isfinite(config.camera_lever_arm_x) &&
           std::isfinite(config.estimated_target_distance) && config.estimated_target_distance > 0.0;
}

void InertialAngleLosFilter::reset() noexcept {
    reset_angles();
    distance_initialized_ = false;
    distance_generation_ = 0;
    previous_exposure_timestamp_ns_ = 0;
    estimated_target_distance_ = std::max(config_.estimated_target_distance, kMinEstimatedDistance);
    previous_speed_mps_ = 0.0;
}

void InertialAngleLosFilter::reset_angles() noexcept {
    pitch_ = {};
    yaw_ = {};
    initialized_ = false;
    generation_ = 0;
    previous_pitch_ = previous_yaw_ = 0.0;
    sample_ = {};
}

domain::LineOfSightRateSample InertialAngleLosFilter::sample() const {
    auto result = sample_;
    result.lever_arm_distance_valid = initialized_ && config_.enable_lever_arm_compensation;
    result.lever_arm_distance_m = result.lever_arm_distance_valid ? estimated_target_distance_ : 0.0;
    return result;
}

void InertialAngleLosFilter::AngleState::initialize(
    double measurement, const InertialAngleLosFilterConfig& config) noexcept {
    angle = measurement;
    rate = 0.0;
    p00 = config.sigma_q * config.sigma_q;
    p01 = 0.0;
    p11 = config.initial_covariance;
}

bool InertialAngleLosFilter::AngleState::update(
    double measurement, double dt, double sigma_a, double measurement_variance, bool wrap) noexcept {
    const double dt2 = dt * dt;
    const double q = sigma_a * sigma_a;
    const double predicted_angle = angle + rate * dt;
    const double predicted_p00 = p00 + 2.0 * dt * p01 + dt2 * p11 + q * dt2 * dt2 / 4.0;
    const double predicted_p01 = p01 + dt * p11 + q * dt2 * dt / 2.0;
    const double predicted_p11 = p11 + q * dt2;
    const double s = predicted_p00 + measurement_variance;
    if (!std::isfinite(predicted_angle) || !std::isfinite(s) || s <= 0.0 ||
        !std::isfinite(predicted_p01) || !std::isfinite(predicted_p11)) return false;

    // 姿态与杆臂已在观测端补偿，预测仅为 F=[1,dt;0,1]，没有机体转动输入项。
    const double innovation = wrap ? normalize_angle(measurement - predicted_angle)
                                   : measurement - predicted_angle;
    const double k0 = predicted_p00 / s;
    const double k1 = predicted_p01 / s;
    angle = predicted_angle + k0 * innovation;
    if (wrap) angle = normalize_angle(angle);
    rate += k1 * innovation;

    // Joseph 形式与原二状态更新等价，避免小测量噪声下相减丢失协方差精度。
    const double a = 1.0 - k0;
    p00 = a * a * predicted_p00 + k0 * k0 * measurement_variance;
    p01 = a * (predicted_p01 - k1 * predicted_p00) + k0 * k1 * measurement_variance;
    p11 = predicted_p11 - 2.0 * k1 * predicted_p01 + k1 * k1 * s;
    return std::isfinite(angle) && std::isfinite(rate) &&
           std::isfinite(p00) && std::isfinite(p01) && std::isfinite(p11) &&
           p00 > 0.0 && p11 > 0.0 && std::fma(-p01 / p00, p01, p11) > 0.0;
}

domain::LosRateEstimate InertialAngleLosFilter::update(const domain::LosRateFilterInput& input) {
    using Reason = domain::GuidanceInvalidReason;
    if (!input.line_of_sight.valid) {
        reset_angles();
        return {false, {}, {}, Reason::invalid_los};
    }
    if (!input.attitude.valid) {
        reset_angles();
        return {false, {}, {}, Reason::invalid_attitude};
    }
    if (!std::isfinite(input.dt_seconds) || input.dt_seconds < 0.0 ||
        (initialized_ && input.dt_seconds == 0.0)) {
        reset_angles();
        return {false, {}, {}, Reason::invalid_timestep};
    }
    if (config_.enable_lever_arm_compensation &&
        (!std::isfinite(input.speed_mps.value) || input.speed_mps.value < 0.0)) {
        reset_angles();
        return {false, {}, {}, Reason::invalid_speed};
    }
    if (config_.enable_lever_arm_compensation && input.attitude.time.monotonic_ns < 0) {
        reset_angles();
        return {false, {}, {}, Reason::invalid_attitude_time};
    }
    if (!detail::valid_rotation(input.attitude.navigation_from_dart.matrix())) {
        reset_angles();
        return {false, {}, {}, Reason::invalid_attitude};
    }

    const auto& direction = input.line_of_sight.direction;
    const double norm = std::hypot(std::hypot(direction.x, direction.y), direction.z);
    if (!std::isfinite(norm) || norm < kEpsilon) {
        reset_angles();
        return {false, {}, {}, Reason::invalid_los};
    }
    const domain::DartLineOfSight normalized_los{
        true, {direction.x / norm, direction.y / norm, direction.z / norm}};
    const auto navigation_direction = input.attitude.navigation_from_dart.apply(normalized_los.direction);
    const bool gap_exceeded = initialized_ && input.generation == generation_ &&
                              input.dt_seconds > config_.maximum_gap_seconds;
    const bool new_segment = !initialized_ || input.generation != generation_ || gap_exceeded;
    if (new_segment) {
        reset_angles();
    }

    double x = navigation_direction.x;
    double y = navigation_direction.y;
    double z = navigation_direction.z;
    if (config_.enable_lever_arm_compensation) {
        const auto exposure_timestamp_ns = input.attitude.time.monotonic_ns;
        if (!distance_initialized_ || input.generation != distance_generation_) {
            estimated_target_distance_ = std::max(config_.estimated_target_distance, kMinEstimatedDistance);
            distance_initialized_ = true;
            distance_generation_ = input.generation;
        } else {
            if (exposure_timestamp_ns <= previous_exposure_timestamp_ns_) {
                reset_angles();
                return {false, {}, {}, Reason::non_increasing_timestamp};
            }
            // 距离时间轴独立于 LOS 连续段，重捕获时仍计入缺失区间的飞行路程。
            const double distance_dt = static_cast<double>(exposure_timestamp_ns - previous_exposure_timestamp_ns_) * 1e-9;
            const double average_speed = 0.5 * previous_speed_mps_ + 0.5 * input.speed_mps.value;
            const double travelled = average_speed * distance_dt;
            if (!std::isfinite(travelled)) {
                reset_angles();
                return {false, {}, {}, Reason::invalid_speed};
            }
            estimated_target_distance_ = std::max(kMinEstimatedDistance, estimated_target_distance_ - travelled);
        }
        previous_exposure_timestamp_ns_ = exposure_timestamp_ns;
        previous_speed_mps_ = input.speed_mps.value;
        // 绝对杆臂补偿将视线起点移至质心；1 m 距离下限限制近距离视差放大。
        const auto& m = input.attitude.navigation_from_dart.matrix();
        x += m[0] * config_.camera_lever_arm_x / estimated_target_distance_;
        y += m[3] * config_.camera_lever_arm_x / estimated_target_distance_;
        z += m[6] * config_.camera_lever_arm_x / estimated_target_distance_;
    }
    const double corrected_norm = std::hypot(std::hypot(x, y), z);
    if (!std::isfinite(corrected_norm) || corrected_norm < kEpsilon) {
        reset_angles();
        return {false, {}, {}, Reason::invalid_los};
    }
    x /= corrected_norm;
    y /= corrected_norm;
    z /= corrected_norm;
    const double horizontal = std::hypot(x, z);
    if (horizontal < kPoleThreshold) {
        reset_angles();
        return {false, {}, {}, Reason::los_pole};
    }
    const double measured_pitch = std::atan2(y, horizontal);
    const double measured_yaw = std::atan2(z, x);
    sample_.line_of_sight = normalized_los;

    if (new_segment) {
        pitch_.initialize(measured_pitch, config_);
        yaw_.initialize(measured_yaw, config_);
        previous_pitch_ = measured_pitch;
        previous_yaw_ = measured_yaw;
        generation_ = input.generation;
        initialized_ = true;
        return {false, {}, {}, gap_exceeded ? Reason::los_gap : Reason::los_initializing};
    }

    const double raw_pitch_rate = (measured_pitch - previous_pitch_) / input.dt_seconds;
    const double raw_yaw_rate = normalize_angle(measured_yaw - previous_yaw_) / input.dt_seconds;
    previous_pitch_ = measured_pitch;
    previous_yaw_ = measured_yaw;
    if (!std::isfinite(raw_pitch_rate) || !std::isfinite(raw_yaw_rate)) {
        reset_angles();
        return {false, {}, {}, Reason::invalid_rate};
    }
    sample_.raw_angular_velocity = {true, {raw_pitch_rate}, {horizontal * raw_yaw_rate}};

    if (config_.enable_filter) {
        const double variance = config_.sigma_q * config_.sigma_q;
        const double pitch_noise = config_.sigma_a_pitch > 0.0 ? config_.sigma_a_pitch : config_.sigma_a;
        const double yaw_noise = config_.sigma_a_yaw > 0.0 ? config_.sigma_a_yaw : config_.sigma_a;
        if (!pitch_.update(measured_pitch, input.dt_seconds, pitch_noise, variance, false) ||
            !yaw_.update(measured_yaw, input.dt_seconds, yaw_noise, variance, true)) {
            reset_angles();
            return {false, {}, {}, Reason::filter_rejected};
        }
    } else {
        pitch_.angle = measured_pitch;
        pitch_.rate = raw_pitch_rate;
        yaw_.angle = measured_yaw;
        yaw_.rate = raw_yaw_rate;
    }

    const double cy = std::cos(pitch_.angle), sy = std::sin(pitch_.angle);
    const double cz = std::cos(yaw_.angle), sz = std::sin(yaw_.angle);
    if (std::abs(pitch_.angle) >= kPi / 2.0 || cy < kPoleThreshold) {
        reset_angles();
        return {false, {}, {}, Reason::los_pole};
    }

    domain::LosRateEstimate estimate;
    estimate.valid = true;
    estimate.invalid_reason = Reason::none;
    // q_z 的公共诊断仍为横向有效速率，不能将方位角导数直接送入现有 PNG。
    estimate.angular_velocity = {true, {pitch_.rate}, {cy * yaw_.rate}};
    estimate.vector.valid = true;
    estimate.vector.direction = {cy * cz, sy, cy * sz};
    // 对角度重建公式解析求导，天然满足 |l|=1、l·dl/dt=0，不再保留球面投影滤波。
    estimate.vector.inertial_direction_rate = {
        -sy * cz * pitch_.rate - cy * sz * yaw_.rate,
        cy * pitch_.rate,
        -sy * sz * pitch_.rate + cy * cz * yaw_.rate};
    const auto& rate = estimate.vector.inertial_direction_rate;
    if (!std::isfinite(rate.x) || !std::isfinite(rate.y) || !std::isfinite(rate.z)) {
        reset_angles();
        return {false, {}, {}, Reason::invalid_rate};
    }
    sample_.filtered_angular_velocity = estimate.angular_velocity;
    return estimate;
}

}  // namespace mosas::guidance
