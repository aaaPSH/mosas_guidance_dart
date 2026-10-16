#include <mosas/runtime/attitude_estimator.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace mosas::runtime {
namespace {

constexpr double kEpsilon = 1e-12;
constexpr TimestampNs kMinimumIntegrationGapNs = 1'000'000;

bool finite(double value) noexcept { return std::isfinite(value); }

TimestampNs maximum_integration_gap_ns(
    const AttitudeEstimatorConfig& config) noexcept {
    if (config.launch_confirm_ns >
        std::numeric_limits<TimestampNs>::max() / 100) {
        return std::numeric_limits<TimestampNs>::max();
    }
    return std::max(kMinimumIntegrationGapNs,
                    config.launch_confirm_ns * 100);
}

}  // namespace

AttitudeEstimator::AttitudeEstimator(const AttitudeEstimatorConfig& config)
    : config_(valid_config(config) ? config : AttitudeEstimatorConfig{}) {
    reset();
}

bool AttitudeEstimator::valid_config(
    const AttitudeEstimatorConfig& config) noexcept {
    return finite(config.gravity_mps2) && config.gravity_mps2 > 0.0 &&
           finite(config.launch_speed_mps) && config.launch_speed_mps > 0.0 &&
           finite(config.stationary_acceleration_tolerance_mps2) &&
           config.stationary_acceleration_tolerance_mps2 > 0.0 &&
           finite(config.gyro_stationary_threshold_rad_s) &&
           config.gyro_stationary_threshold_rad_s > 0.0 &&
           finite(config.gyro_bias_filter_alpha) &&
           config.gyro_bias_filter_alpha > 0.0 &&
           config.gyro_bias_filter_alpha <= 1.0 &&
           finite(config.launch_ax_threshold_mps2) &&
           config.launch_ax_threshold_mps2 > 0.0 &&
           config.launch_confirm_ns > 0;
}

bool AttitudeEstimator::finite_vector(const Vector3& value) noexcept {
    return finite(value.x) && finite(value.y) && finite(value.z);
}

double AttitudeEstimator::vector_norm(const Vector3& value) noexcept {
    return std::hypot(std::hypot(value.x, value.y), value.z);
}

AttitudeEstimator::Quaternion AttitudeEstimator::multiply(
    const Quaternion& lhs, const Quaternion& rhs) noexcept {
    return {
        lhs.w * rhs.w - lhs.x * rhs.x - lhs.y * rhs.y - lhs.z * rhs.z,
        lhs.w * rhs.x + lhs.x * rhs.w + lhs.y * rhs.z - lhs.z * rhs.y,
        lhs.w * rhs.y - lhs.x * rhs.z + lhs.y * rhs.w + lhs.z * rhs.x,
        lhs.w * rhs.z + lhs.x * rhs.y - lhs.y * rhs.x + lhs.z * rhs.w,
    };
}

AttitudeEstimator::Quaternion AttitudeEstimator::normalize(
    const Quaternion& value) noexcept {
    const double norm = std::sqrt(value.w * value.w + value.x * value.x +
                                   value.y * value.y + value.z * value.z);
    if (!finite(norm) || norm < kEpsilon) {
        return {};
    }
    return {value.w / norm, value.x / norm, value.y / norm, value.z / norm};
}

AttitudeEstimator::Quaternion AttitudeEstimator::from_roll_pitch(
    double roll_rad, double pitch_rad) noexcept {
    const double half_roll = 0.5 * roll_rad;
    const double half_pitch = 0.5 * pitch_rad;
    const Quaternion roll{std::cos(half_roll), std::sin(half_roll), 0.0,
                          0.0};
    const Quaternion pitch{std::cos(half_pitch), 0.0, 0.0,
                           std::sin(half_pitch)};
    // 使用 pitch 绕 Z、roll 绕 X 的顺序重建无航向初始化姿态。
    return normalize(multiply(pitch, roll));
}

AttitudeEstimator::Quaternion AttitudeEstimator::delta_from_body_rate(
    const Vector3& angular_velocity, double dt_seconds) noexcept {
    const double half_angle_x = 0.5 * angular_velocity.x * dt_seconds;
    const double half_angle_y = 0.5 * angular_velocity.y * dt_seconds;
    const double half_angle_z = 0.5 * angular_velocity.z * dt_seconds;
    const double angle = std::sqrt(half_angle_x * half_angle_x +
                                   half_angle_y * half_angle_y +
                                   half_angle_z * half_angle_z);
    if (!finite(angle)) {
        return {};
    }
    if (angle < kEpsilon) {
        return normalize({1.0, half_angle_x, half_angle_y, half_angle_z});
    }
    const double scale = std::sin(angle) / angle;
    return normalize({std::cos(angle), half_angle_x * scale,
                      half_angle_y * scale, half_angle_z * scale});
}

RotationMatrix3 AttitudeEstimator::to_matrix(
    const Quaternion& value) noexcept {
    const double ww = value.w * value.w;
    const double xx = value.x * value.x;
    const double yy = value.y * value.y;
    const double zz = value.z * value.z;
    const double xy = value.x * value.y;
    const double xz = value.x * value.z;
    const double yz = value.y * value.z;
    const double wx = value.w * value.x;
    const double wy = value.w * value.y;
    const double wz = value.w * value.z;
    return {
        ww + xx - yy - zz, 2.0 * (xy - wz), 2.0 * (xz + wy),
        2.0 * (xy + wz), ww - xx + yy - zz, 2.0 * (yz - wx),
        2.0 * (xz - wy), 2.0 * (yz + wx), ww - xx - yy + zz,
    };
}

double AttitudeEstimator::pitch_from_matrix(
    const RotationMatrix3& matrix) noexcept {
    return std::atan2(matrix.m10, std::hypot(matrix.m00, matrix.m20));
}

double AttitudeEstimator::roll_from_matrix(
    const RotationMatrix3& matrix) noexcept {
    return std::atan2(-matrix.m12, matrix.m11);
}

AttitudeEstimate AttitudeEstimator::make_estimate(
    TimestampNs timestamp_ns, bool valid) const noexcept {
    const RotationMatrix3 matrix = to_matrix(orientation_);
    const double pitch = pitch_from_matrix(matrix);
    const double roll = roll_from_matrix(matrix);
    AttitudeEstimate estimate{};
    estimate.timestamp_ns = timestamp_ns;
    estimate.body_to_navigation = matrix;
    estimate.pitch_rad = pitch;
    estimate.roll_rad = roll;
    estimate.speed_mps = speed_mps_;
    estimate.launched = launched_;
    estimate.valid = valid;
    estimate.launch_timestamp_ns = launch_timestamp_ns_;
    estimate.corrected_angular_velocity = corrected_angular_velocity_;
    return estimate;
}

bool AttitudeEstimator::is_stationary(
    const Vector3& acceleration,
    const Vector3& angular_velocity) const noexcept {
    const double acceleration_norm = vector_norm(acceleration);
    const double angular_velocity_norm = vector_norm(angular_velocity);
    return finite(acceleration_norm) && finite(angular_velocity_norm) &&
           std::abs(acceleration_norm - config_.gravity_mps2) <=
               config_.stationary_acceleration_tolerance_mps2 &&
           angular_velocity_norm <= config_.gyro_stationary_threshold_rad_s;
}

void AttitudeEstimator::update_gyro_bias(
    const Vector3& angular_velocity) noexcept {
    if (!has_gyro_bias_) {
        gyro_bias_ = angular_velocity;
        has_gyro_bias_ = true;
        return;
    }

    const double alpha = config_.gyro_bias_filter_alpha;
    gyro_bias_.x = alpha * angular_velocity.x +
                   (1.0 - alpha) * gyro_bias_.x;
    gyro_bias_.y = alpha * angular_velocity.y +
                   (1.0 - alpha) * gyro_bias_.y;
    gyro_bias_.z = alpha * angular_velocity.z +
                   (1.0 - alpha) * gyro_bias_.z;
}

bool AttitudeEstimator::update_prelaunch_attitude(
    const Vector3& acceleration) noexcept {
    const Vector3 up_force = config_.accelerometer_specific_force
                                 ? acceleration
                                 : Vector3{-acceleration.x, -acceleration.y,
                                           -acceleration.z};
    const double norm = std::hypot(
        std::hypot(up_force.x, up_force.y), up_force.z);
    if (!finite(norm) || norm < kEpsilon) {
        return false;
    }

    const double pitch = std::atan2(
        up_force.x, std::hypot(up_force.y, up_force.z));
    const double roll = std::atan2(-up_force.z, up_force.y);
    if (!finite(pitch) || !finite(roll)) {
        return false;
    }
    // 重力只用于滚转和俯仰初始化，航向保持初始值。
    orientation_ = from_roll_pitch(roll, pitch);
    return true;
}

void AttitudeEstimator::update_launch_state(
    double current_ax, TimestampNs timestamp_ns) noexcept {
    const double delta_x = current_ax - launch_baseline_x_;
    if (delta_x < config_.launch_ax_threshold_mps2) {
        launch_candidate_start_ns_ = -1;
        return;
    }
    if (launch_candidate_start_ns_ < 0) {
        launch_candidate_start_ns_ = timestamp_ns;
        return;
    }
    if (timestamp_ns >= launch_candidate_start_ns_ &&
        timestamp_ns - launch_candidate_start_ns_ >=
            config_.launch_confirm_ns) {
        launched_ = true;
        launch_timestamp_ns_ = timestamp_ns;
        launch_candidate_start_ns_ = -1;
    }
}

AttitudeEstimate AttitudeEstimator::update(
    const ImuAngularVelocitySample& sample) noexcept {
    if (sample.timestamp_ns < 0 || !finite_vector(sample.acceleration) ||
        !finite_vector(sample.angular_velocity)) {
        return last_estimate_;
    }
    if (has_timestamp_ && sample.timestamp_ns <= last_timestamp_ns_) {
        return last_estimate_;
    }

    const TimestampNs maximum_gap = maximum_integration_gap_ns(config_);
    const bool timestamp_gap_valid =
        !has_timestamp_ || sample.timestamp_ns - last_timestamp_ns_ <= maximum_gap;
    if (!timestamp_gap_valid) {
        // 长时间没有连续 IMU 样本时，不能用跨断流时长确认起飞。
        launch_candidate_start_ns_ = -1;
    }

    if (!has_launch_baseline_) {
        launch_baseline_x_ = sample.acceleration.x;
        has_launch_baseline_ = true;
    }

    const bool was_launched = launched_;
    const bool stationary =
        !was_launched &&
        is_stationary(sample.acceleration, sample.angular_velocity);
    if (stationary) {
        update_gyro_bias(sample.angular_velocity);
        launch_baseline_x_ = sample.acceleration.x;
    }
    corrected_angular_velocity_ = {
        sample.angular_velocity.x - gyro_bias_.x,
        sample.angular_velocity.y - gyro_bias_.y,
        sample.angular_velocity.z - gyro_bias_.z};

    bool attitude_valid = launched_ || last_estimate_.valid;
    if (!was_launched && has_launch_baseline_) {
        update_launch_state(sample.acceleration.x, sample.timestamp_ns);
    }

    const auto integrate = [&] {
        if (!has_timestamp_) {
            return;
        }
        const double dt_seconds = static_cast<double>(
                                      sample.timestamp_ns - last_timestamp_ns_) *
                                  1e-9;
        if (finite(dt_seconds) && dt_seconds > 0.0 &&
            timestamp_gap_valid && attitude_valid) {
            orientation_ = normalize(multiply(
                orientation_,
                delta_from_body_rate(corrected_angular_velocity_,
                                     dt_seconds)));
            if (was_launched) {
                // 标定初速已经包含发射加速；确认起飞帧不再重复累加。
                const double pitch = pitch_from_matrix(to_matrix(orientation_));
                speed_mps_ += (sample.acceleration.x -
                               config_.gravity_mps2 * std::sin(pitch)) *
                              dt_seconds;
            }
        }
    };

    if (!launched_ && stationary) {
        attitude_valid = update_prelaunch_attitude(sample.acceleration);
        if (!attitude_valid) {
            launch_candidate_start_ns_ = -1;
        }
    } else {
        // 非静止时传播姿态；确认起飞后同时积分扣除纵向重力的速度。
        const double acceleration_norm = vector_norm(sample.acceleration);
        const bool prelaunch_near_zero_acceleration =
            !launched_ &&
            (!finite(acceleration_norm) || acceleration_norm < kEpsilon);
        if (prelaunch_near_zero_acceleration) {
            attitude_valid = false;
            launch_candidate_start_ns_ = -1;
        } else {
            integrate();
        }
    }

    last_timestamp_ns_ = sample.timestamp_ns;
    has_timestamp_ = true;
    last_estimate_ = make_estimate(sample.timestamp_ns, attitude_valid);
    return last_estimate_;
}

void AttitudeEstimator::reset() noexcept {
    orientation_ = {};
    gyro_bias_ = {0.0, 0.0, 0.0};
    corrected_angular_velocity_ = {0.0, 0.0, 0.0};
    speed_mps_ = config_.launch_speed_mps;
    launch_baseline_x_ = 0.0;
    launch_candidate_start_ns_ = -1;
    last_timestamp_ns_ = -1;
    has_launch_baseline_ = false;
    has_gyro_bias_ = false;
    has_timestamp_ = false;
    launched_ = false;
    launch_timestamp_ns_ = -1;
    last_estimate_ = make_estimate(-1, false);
}

}  // namespace mosas::runtime
