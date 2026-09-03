#include <mosas/guidance/guidance_estimator.hpp>

#include <cmath>
#include <limits>

namespace {

constexpr double kEpsilon = 1e-12;
constexpr double kRotationTolerance = 1e-6;
constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

bool is_finite(double value) {
    return std::isfinite(value);
}

LineOfSight invalid_line_of_sight() {
    return {false, 0.0, 0.0};
}

bool is_finite(const EulerAngles& value) {
    return is_finite(value.roll) && is_finite(value.pitch) &&
           is_finite(value.yaw);
}

bool is_valid_line_of_sight(const LineOfSight& value) {
    return value.valid && is_finite(value.q_y) && is_finite(value.q_z);
}

bool is_valid_rate_filter_config(const LineOfSightRateFilterConfig& config) {
    return is_finite(config.process_noise) &&
           is_finite(config.measurement_noise) &&
           is_finite(config.initial_covariance) && config.process_noise >= 0.0 &&
           config.measurement_noise > 0.0 && config.initial_covariance >= 0.0;
}

bool is_valid_png_config(const PngGuidanceConfig& config) {
    return is_finite(config.navigation_constant_y) &&
           is_finite(config.navigation_constant_z) &&
           is_finite(config.gravity) && config.navigation_constant_y > 0.0 &&
           config.navigation_constant_z > 0.0 && config.gravity > 0.0;
}

double normalize_angle(double angle) {
    angle = std::fmod(angle + kPi, kTwoPi);
    if (angle < 0.0) {
        angle += kTwoPi;
    }
    return angle - kPi;
}

bool is_finite(const RotationMatrix3& value) {
    return is_finite(value.m00) && is_finite(value.m01) &&
           is_finite(value.m02) && is_finite(value.m10) &&
           is_finite(value.m11) && is_finite(value.m12) &&
           is_finite(value.m20) && is_finite(value.m21) &&
           is_finite(value.m22);
}

double vector_norm(const Vector3& value);

bool is_rotation_matrix(const RotationMatrix3& value) {
    if (!is_finite(value)) {
        return false;
    }

    const Vector3 column_0{value.m00, value.m10, value.m20};
    const Vector3 column_1{value.m01, value.m11, value.m21};
    const Vector3 column_2{value.m02, value.m12, value.m22};
    const double determinant =
        value.m00 * (value.m11 * value.m22 - value.m12 * value.m21) -
        value.m01 * (value.m10 * value.m22 - value.m12 * value.m20) +
        value.m02 * (value.m10 * value.m21 - value.m11 * value.m20);
    return std::abs(vector_norm(column_0) - 1.0) <= kRotationTolerance &&
           std::abs(vector_norm(column_1) - 1.0) <= kRotationTolerance &&
           std::abs(vector_norm(column_2) - 1.0) <= kRotationTolerance &&
           std::abs(column_0.x * column_1.x + column_0.y * column_1.y +
                    column_0.z * column_1.z) <= kRotationTolerance &&
           std::abs(column_0.x * column_2.x + column_0.y * column_2.y +
                    column_0.z * column_2.z) <= kRotationTolerance &&
           std::abs(column_1.x * column_2.x + column_1.y * column_2.y +
                    column_1.z * column_2.z) <= kRotationTolerance &&
           std::abs(determinant - 1.0) <= kRotationTolerance;
}

bool is_valid_intrinsics(const CameraIntrinsics& intrinsics) {
    return is_finite(intrinsics.fx) && is_finite(intrinsics.fy) &&
           is_finite(intrinsics.cx) && is_finite(intrinsics.cy) &&
           intrinsics.fx > 0.0 && intrinsics.fy > 0.0;
}

bool is_valid_vector(const Vector3& value) {
    return is_finite(value.x) && is_finite(value.y) && is_finite(value.z);
}

double vector_norm(const Vector3& value) {
    return std::hypot(std::hypot(value.x, value.y), value.z);
}

Vector3 normalize(const Vector3& value) {
    const double norm = vector_norm(value);
    if (!is_finite(norm) || norm < kEpsilon) {
        return {0.0, 0.0, 0.0};
    }
    return {value.x / norm, value.y / norm, value.z / norm};
}

Vector3 multiply(const RotationMatrix3& matrix, const Vector3& value) {
    return {
        matrix.m00 * value.x + matrix.m01 * value.y + matrix.m02 * value.z,
        matrix.m10 * value.x + matrix.m11 * value.y + matrix.m12 * value.z,
        matrix.m20 * value.x + matrix.m21 * value.y + matrix.m22 * value.z,
    };
}

RotationMatrix3 identity_rotation() {
    return {1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0};
}

RotationMatrix3 rotation_from_euler(const EulerAngles& angles) {
    const double cr = std::cos(angles.roll);
    const double sr = std::sin(angles.roll);
    const double cp = std::cos(angles.pitch);
    const double sp = std::sin(angles.pitch);
    const double cy = std::cos(angles.yaw);
    const double sy = std::sin(angles.yaw);

    // YZX 欧拉角对应的弹体坐标系到地面坐标系旋转矩阵。
    return {
        cy * cp, -cy * sp * cr + sy * sr, cy * sp * sr + sy * cr,
        sp,      cp * cr,                 -cp * sr,
        -sy * cp, sy * sp * cr + cy * sr, -sy * sp * sr + cy * cr,
    };
}

bool camera_direction(const VisionResult& result,
                      const CameraIntrinsics& intrinsics,
                      Vector3* direction) {
    if (direction == nullptr || !result.found ||
        !is_finite(result.blob.center_x) ||
        !is_finite(result.blob.center_y) ||
        !is_valid_intrinsics(intrinsics)) {
        return false;
    }

    const double right =
        (result.blob.center_x - intrinsics.cx) / intrinsics.fx;
    const double up =
        -(result.blob.center_y - intrinsics.cy) / intrinsics.fy;
    const Vector3 unnormalized{1.0, up, right};
    if (!is_valid_vector(unnormalized) ||
        vector_norm(unnormalized) < kEpsilon) {
        return false;
    }

    *direction = normalize(unnormalized);
    return is_valid_vector(*direction) && vector_norm(*direction) >= kEpsilon;
}

LineOfSight angles_from_direction(const Vector3& direction) {
    if (!is_valid_vector(direction)) {
        return invalid_line_of_sight();
    }

    const double norm = vector_norm(direction);
    if (!is_finite(norm) || norm < kEpsilon) {
        return invalid_line_of_sight();
    }

    const double q_y = std::atan2(direction.y,
                                  std::hypot(direction.x, direction.z));
    const double q_z = std::atan2(direction.z, direction.x);
    if (!is_finite(q_y) || !is_finite(q_z)) {
        return invalid_line_of_sight();
    }
    return {true, q_y, q_z};
}

}  // namespace

LineOfSightRateEstimator::ScalarKalmanFilter::ScalarKalmanFilter(
    const LineOfSightRateFilterConfig& config)
    : config_(is_valid_rate_filter_config(config)
                  ? config
                  : LineOfSightRateFilterConfig{}) {
    reset();
}

void LineOfSightRateEstimator::ScalarKalmanFilter::reset() {
    angle_ = 0.0;
    rate_ = 0.0;
    angle_covariance_ = config_.measurement_noise;
    angle_rate_covariance_ = 0.0;
    rate_covariance_ = config_.initial_covariance;
}

void LineOfSightRateEstimator::ScalarKalmanFilter::initialize(double angle,
                                                               double rate) {
    angle_ = angle;
    rate_ = rate;
    angle_covariance_ = config_.measurement_noise;
    angle_rate_covariance_ = 0.0;
    rate_covariance_ = config_.initial_covariance;
}

double LineOfSightRateEstimator::ScalarKalmanFilter::update(
    double measurement, double dt) {
    const double dt_squared = dt * dt;
    const double dt_cubed = dt_squared * dt;
    const double dt_fourth = dt_cubed * dt;

    // 使用常角加速度模型预测角度和角速度。
    const double predicted_angle = angle_ + rate_ * dt;
    const double predicted_rate = rate_;
    const double predicted_angle_covariance =
        angle_covariance_ + 2.0 * dt * angle_rate_covariance_ +
        dt_squared * rate_covariance_ +
        config_.process_noise * dt_fourth * 0.25;
    const double predicted_angle_rate_covariance =
        angle_rate_covariance_ + dt * rate_covariance_ +
        config_.process_noise * dt_cubed * 0.5;
    const double predicted_rate_covariance =
        rate_covariance_ + config_.process_noise * dt_squared;

    const double innovation = measurement - predicted_angle;
    const double innovation_covariance =
        predicted_angle_covariance + config_.measurement_noise;
    if (!is_finite(predicted_angle) || !is_finite(predicted_rate) ||
        !is_finite(predicted_angle_covariance) ||
        !is_finite(predicted_angle_rate_covariance) ||
        !is_finite(predicted_rate_covariance) ||
        !is_finite(innovation_covariance) || innovation_covariance <= 0.0) {
        return std::numeric_limits<double>::quiet_NaN();
    }

    const double angle_gain =
        predicted_angle_covariance / innovation_covariance;
    const double rate_gain =
        predicted_angle_rate_covariance / innovation_covariance;
    angle_ = predicted_angle + angle_gain * innovation;
    rate_ = predicted_rate + rate_gain * innovation;
    angle_covariance_ =
        (1.0 - angle_gain) * predicted_angle_covariance;
    angle_rate_covariance_ =
        (1.0 - angle_gain) * predicted_angle_rate_covariance;
    rate_covariance_ = predicted_rate_covariance -
                       rate_gain * predicted_angle_rate_covariance;
    return rate_;
}

LineOfSight GuidanceEstimator::calculate(
    const VisionResult& result, const CameraIntrinsics& intrinsics) {
    Vector3 direction{};
    if (!camera_direction(result, intrinsics, &direction)) {
        return invalid_line_of_sight();
    }
    return angles_from_direction(direction);
}

LineOfSight GuidanceEstimator::calculate_compensated(
    const VisionResult& result, const CameraIntrinsics& intrinsics,
    const EulerAngles& attitude) {
    return calculate_compensated(result, intrinsics, attitude,
                                  identity_rotation());
}

LineOfSight GuidanceEstimator::calculate_compensated(
    const VisionResult& result, const CameraIntrinsics& intrinsics,
    const EulerAngles& attitude, const RotationMatrix3& camera_to_body) {
    Vector3 camera_direction_value{};
    if (!is_finite(attitude) || !is_rotation_matrix(camera_to_body) ||
        !camera_direction(result, intrinsics, &camera_direction_value)) {
        return invalid_line_of_sight();
    }

    const Vector3 body_direction =
        multiply(camera_to_body, camera_direction_value);
    if (!is_valid_vector(body_direction) ||
        vector_norm(body_direction) < kEpsilon) {
        return invalid_line_of_sight();
    }

    const RotationMatrix3 body_to_navigation = rotation_from_euler(attitude);
    const Vector3 navigation_direction =
        multiply(body_to_navigation, normalize(body_direction));
    return angles_from_direction(navigation_direction);
}

LineOfSightRateEstimator::LineOfSightRateEstimator(
    const LineOfSightRateFilterConfig& config)
    : filter_enabled_(is_valid_rate_filter_config(config)
                          ? config.enable_filter
                          : true),
      q_y_filter_(config),
      q_z_filter_(config),
      sample_{} {
    reset();
}

void LineOfSightRateEstimator::reset() {
    has_previous_ = false;
    has_rate_ = false;
    previous_q_y_ = 0.0;
    previous_q_z_ = 0.0;
    q_y_filter_.reset();
    q_z_filter_.reset();
    angular_velocity_ = {false, 0.0, 0.0};
    sample_ = {};
}

LineOfSightAngularVelocity LineOfSightRateEstimator::update(
    const LineOfSight& line_of_sight, double dt) {
    if (!is_valid_line_of_sight(line_of_sight) || !is_finite(dt) ||
        dt <= 0.0) {
        reset();
        return angular_velocity_;
    }

    if (!has_previous_) {
        previous_q_y_ = line_of_sight.q_y;
        previous_q_z_ = line_of_sight.q_z;
        has_previous_ = true;
        angular_velocity_ = {false, 0.0, 0.0};
        sample_.line_of_sight = line_of_sight;
        sample_.raw_angular_velocity = {false, 0.0, 0.0};
        sample_.filtered_angular_velocity = angular_velocity_;
        return angular_velocity_;
    }

    const double measured_q_y = (line_of_sight.q_y - previous_q_y_) / dt;
    const double measured_q_z =
        normalize_angle(line_of_sight.q_z - previous_q_z_) / dt;
    previous_q_y_ = line_of_sight.q_y;
    previous_q_z_ = line_of_sight.q_z;
    if (!is_finite(measured_q_y) || !is_finite(measured_q_z)) {
        reset();
        return angular_velocity_;
    }

    const LineOfSightAngularVelocity raw_angular_velocity{
        true, measured_q_y, measured_q_z};
    sample_.line_of_sight = line_of_sight;
    sample_.raw_angular_velocity = raw_angular_velocity;

    if (!filter_enabled_) {
        angular_velocity_ = raw_angular_velocity;
        sample_.filtered_angular_velocity = angular_velocity_;
        return angular_velocity_;
    }

    if (!has_rate_) {
        // 第二帧直接提供初始角速度，避免短时飞行中多帧等待滤波收敛。
        q_y_filter_.initialize(line_of_sight.q_y, measured_q_y);
        q_z_filter_.initialize(line_of_sight.q_z, measured_q_z);
        has_rate_ = true;
        angular_velocity_ = raw_angular_velocity;
        sample_.filtered_angular_velocity = angular_velocity_;
        return angular_velocity_;
    }

    const double filtered_q_y = q_y_filter_.update(line_of_sight.q_y, dt);
    const double filtered_q_z = q_z_filter_.update(line_of_sight.q_z, dt);
    if (!is_finite(filtered_q_y) || !is_finite(filtered_q_z)) {
        reset();
        return angular_velocity_;
    }

    angular_velocity_ = {true, filtered_q_y, filtered_q_z};
    sample_.filtered_angular_velocity = angular_velocity_;
    return angular_velocity_;
}

LineOfSightAngularVelocity LineOfSightRateEstimator::angular_velocity() const {
    return angular_velocity_;
}

LineOfSightRateSample LineOfSightRateEstimator::sample() const {
    return sample_;
}

PngGuidanceOutput PngGuidance::calculate(
    const LineOfSight& line_of_sight,
    const LineOfSightAngularVelocity& angular_velocity,
    const Vector3& dart_velocity, const EulerAngles& attitude,
    const PngGuidanceConfig& config) {
    const PngGuidanceOutput invalid_output{
        false,
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
        0.0,
        0.0,
    };
    if (!is_valid_line_of_sight(line_of_sight) ||
        !angular_velocity.valid || !is_finite(angular_velocity.q_y) ||
        !is_finite(angular_velocity.q_z) || !is_valid_vector(dart_velocity) ||
        !is_finite(attitude) || !is_valid_png_config(config)) {
        return invalid_output;
    }

    const double speed = vector_norm(dart_velocity);
    if (!is_finite(speed) || speed < kEpsilon) {
        return invalid_output;
    }

    // theta 为飞镖速度矢量的弹道俯仰角，v 为速度模长。
    const double cos_theta =
        std::hypot(dart_velocity.x, dart_velocity.z) / speed;
    const double speed_over_gravity = speed / config.gravity;
    const double vertical_overload =
        config.navigation_constant_y * angular_velocity.q_y *
            speed_over_gravity +
        cos_theta;
    const double lateral_overload =
        -config.navigation_constant_z * angular_velocity.q_z *
        speed_over_gravity * cos_theta;
    if (!is_finite(cos_theta) || !is_finite(vertical_overload) ||
        !is_finite(lateral_overload)) {
        return invalid_output;
    }

    const double command_overload =
        std::hypot(vertical_overload, lateral_overload);
    const double command_phase =
        std::atan2(lateral_overload, vertical_overload);
    if (!is_finite(command_overload) || !is_finite(command_phase)) {
        return invalid_output;
    }

    // 二维解耦：纵向和横向通道分别生成导航系 y/z 过载，x 轴不产生指令。
    const Vector3 navigation_acceleration{
        0.0,
        config.gravity * vertical_overload,
        config.gravity * lateral_overload,
    };
    if (!is_valid_vector(navigation_acceleration)) {
        return invalid_output;
    }

    const RotationMatrix3 body_to_navigation = rotation_from_euler(attitude);
    const Vector3 body_acceleration{
        body_to_navigation.m00 * navigation_acceleration.x +
            body_to_navigation.m10 * navigation_acceleration.y +
            body_to_navigation.m20 * navigation_acceleration.z,
        body_to_navigation.m01 * navigation_acceleration.x +
            body_to_navigation.m11 * navigation_acceleration.y +
            body_to_navigation.m21 * navigation_acceleration.z,
        body_to_navigation.m02 * navigation_acceleration.x +
            body_to_navigation.m12 * navigation_acceleration.y +
            body_to_navigation.m22 * navigation_acceleration.z,
    };
    if (!is_valid_vector(body_acceleration)) {
        return invalid_output;
    }

    const Vector3 body_overload{
        body_acceleration.x / config.gravity,
        body_acceleration.y / config.gravity,
        body_acceleration.z / config.gravity,
    };
    if (!is_valid_vector(body_overload)) {
        return invalid_output;
    }
    return {true,
            navigation_acceleration,
            body_acceleration,
            body_overload,
            command_overload,
            command_phase};
}
