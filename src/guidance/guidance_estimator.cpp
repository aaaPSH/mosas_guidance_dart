#include <mosas/guidance/guidance_estimator.hpp>

#include <cmath>
#include <limits>

namespace {

constexpr double kEpsilon = 1e-12;
constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

bool is_finite(double value) { return std::isfinite(value); }

bool is_finite(const Vector3& value) {
    return is_finite(value.x) && is_finite(value.y) && is_finite(value.z);
}

bool is_valid_line_of_sight(const LineOfSight& value) {
    return value.valid && is_finite(value.q_y) && is_finite(value.q_z);
}

bool is_valid_rate_filter_config(const LineOfSightRateFilterConfig& config) {
    return is_finite(config.sigma_a) && config.sigma_a > 0.0 &&
           is_finite(config.sigma_q) && config.sigma_q > 0.0 &&
           is_finite(config.initial_covariance) &&
           config.initial_covariance >= 0.0 &&
           is_finite(config.sigma_a_pitch) && config.sigma_a_pitch >= 0.0 &&
           is_finite(config.sigma_a_yaw) && config.sigma_a_yaw >= 0.0;
}

bool is_valid_png_config(const PngGuidanceConfig& config) {
    const bool maximum_acceleration_valid =
        (is_finite(config.maximum_command_acceleration) &&
         config.maximum_command_acceleration >= 0.0) ||
        (std::isinf(config.maximum_command_acceleration) &&
         config.maximum_command_acceleration > 0.0);
    return is_finite(config.navigation_constant_y) &&
           is_finite(config.navigation_constant_z) &&
           is_finite(config.gravity) && config.navigation_constant_y > 0.0 &&
           config.navigation_constant_z > 0.0 && config.gravity > 0.0 &&
           is_finite(config.maximum_command_overload) &&
           config.maximum_command_overload >= 0.0 &&
           maximum_acceleration_valid;
}

double normalize_angle(double angle) {
    angle = std::fmod(angle + kPi, kTwoPi);
    if (angle < 0.0) {
        angle += kTwoPi;
    }
    return angle - kPi;
}

LineOfSight invalid_line_of_sight() { return {false, 0.0, 0.0}; }

bool is_valid_intrinsics(const CameraIntrinsics& intrinsics) {
    return is_finite(intrinsics.fx) && is_finite(intrinsics.fy) &&
           is_finite(intrinsics.cx) && is_finite(intrinsics.cy) &&
           intrinsics.fx > 0.0 && intrinsics.fy > 0.0;
}

bool is_valid_rotation(const RotationMatrix3& value) {
    if (!is_finite(value.m00) || !is_finite(value.m01) ||
        !is_finite(value.m02) || !is_finite(value.m10) ||
        !is_finite(value.m11) || !is_finite(value.m12) ||
        !is_finite(value.m20) || !is_finite(value.m21) ||
        !is_finite(value.m22)) {
        return false;
    }
    const Vector3 c0{value.m00, value.m10, value.m20};
    const Vector3 c1{value.m01, value.m11, value.m21};
    const Vector3 c2{value.m02, value.m12, value.m22};
    const auto dot = [](const Vector3& lhs, const Vector3& rhs) {
        return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
    };
    const double determinant =
        value.m00 * (value.m11 * value.m22 - value.m12 * value.m21) -
        value.m01 * (value.m10 * value.m22 - value.m12 * value.m20) +
        value.m02 * (value.m10 * value.m21 - value.m11 * value.m20);
    return std::abs(std::hypot(std::hypot(c0.x, c0.y), c0.z) - 1.0) <= 1e-6 &&
           std::abs(std::hypot(std::hypot(c1.x, c1.y), c1.z) - 1.0) <= 1e-6 &&
           std::abs(std::hypot(std::hypot(c2.x, c2.y), c2.z) - 1.0) <= 1e-6 &&
           std::abs(dot(c0, c1)) <= 1e-6 && std::abs(dot(c0, c2)) <= 1e-6 &&
           std::abs(dot(c1, c2)) <= 1e-6 && std::abs(determinant - 1.0) <= 1e-6;
}

Vector3 multiply(const RotationMatrix3& matrix, const Vector3& value) {
    return {
        matrix.m00 * value.x + matrix.m01 * value.y + matrix.m02 * value.z,
        matrix.m10 * value.x + matrix.m11 * value.y + matrix.m12 * value.z,
        matrix.m20 * value.x + matrix.m21 * value.y + matrix.m22 * value.z,
    };
}

Vector3 cross_product(const Vector3& lhs, const Vector3& rhs) {
    return {
        lhs.y * rhs.z - lhs.z * rhs.y,
        lhs.z * rhs.x - lhs.x * rhs.z,
        lhs.x * rhs.y - lhs.y * rhs.x,
    };
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

bool camera_direction(const VisionResult& result,
                      const CameraIntrinsics& intrinsics,
                      Vector3* direction) {
    if (direction == nullptr || !result.found ||
        !is_finite(result.blob.center_x) ||
        !is_finite(result.blob.center_y) || !is_valid_intrinsics(intrinsics)) {
        return false;
    }
    const double right =
        (result.blob.center_x - intrinsics.cx) / intrinsics.fx;
    const double up =
        -(result.blob.center_y - intrinsics.cy) / intrinsics.fy;
    const Vector3 unnormalized{1.0, up, right};
    if (!is_finite(unnormalized) || vector_norm(unnormalized) < kEpsilon) {
        return false;
    }
    *direction = normalize(unnormalized);
    return is_finite(*direction) && vector_norm(*direction) >= kEpsilon;
}

LineOfSight angles_from_direction(const Vector3& direction) {
    if (!is_finite(direction)) {
        return invalid_line_of_sight();
    }
    const double norm = vector_norm(direction);
    if (!is_finite(norm) || norm < kEpsilon) {
        return invalid_line_of_sight();
    }
    const double q_y =
        std::atan2(direction.y, std::hypot(direction.x, direction.z));
    const double q_z = std::atan2(direction.z, direction.x);
    return is_finite(q_y) && is_finite(q_z) ? LineOfSight{true, q_y, q_z}
                                           : invalid_line_of_sight();
}

Vector3 line_of_sight_unit(const LineOfSight& line_of_sight) {
    const double sin_elevation = std::sin(line_of_sight.q_y);
    const double cos_elevation = std::cos(line_of_sight.q_y);
    const double sin_azimuth = std::sin(line_of_sight.q_z);
    const double cos_azimuth = std::cos(line_of_sight.q_z);
    return {cos_elevation * cos_azimuth, sin_elevation,
            cos_elevation * sin_azimuth};
}

double dot_product(const Vector3& lhs, const Vector3& rhs) {
    return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

bool body_rotation_rate(const LineOfSight& line_of_sight,
                        const Vector3& body_angular_velocity,
                        double* q_y_rate, double* q_z_rate) {
    if (q_y_rate == nullptr || q_z_rate == nullptr ||
        !is_finite(body_angular_velocity)) {
        return false;
    }
    const double cos_elevation = std::cos(line_of_sight.q_y);
    if (!is_finite(cos_elevation) || std::abs(cos_elevation) < kEpsilon) {
        return false;
    }
    const Vector3 unit = line_of_sight_unit(line_of_sight);
    const Vector3 rotation_rate = cross_product(body_angular_velocity, unit);
    const Vector3 elevation_basis{
        -std::sin(line_of_sight.q_y) * std::cos(line_of_sight.q_z),
        cos_elevation,
        -std::sin(line_of_sight.q_y) * std::sin(line_of_sight.q_z)};
    const Vector3 azimuth_derivative{
        -cos_elevation * std::sin(line_of_sight.q_z), 0.0,
        cos_elevation * std::cos(line_of_sight.q_z)};
    *q_y_rate = dot_product(rotation_rate, elevation_basis);
    *q_z_rate = dot_product(rotation_rate, azimuth_derivative) /
                (cos_elevation * cos_elevation);
    return is_finite(*q_y_rate) && is_finite(*q_z_rate);
}

}  // namespace

LineOfSightRateEstimator::TwoStateLosKalmanFilter::TwoStateLosKalmanFilter(
    double sigma_a, double sigma_q, double initial_covariance)
    : sigma_a_(sigma_a > 0.0 ? sigma_a : 8.0),
      r_(sigma_q > 0.0 ? sigma_q * sigma_q : 0.0035 * 0.0035),
      initial_covariance_(initial_covariance >= 0.0 ? initial_covariance : 1.0) {
    reset();
}

void LineOfSightRateEstimator::TwoStateLosKalmanFilter::reset() {
    initialized_ = false;
    q_b_ = 0.0;
    q_dot_i_ = 0.0;
    p00_ = r_;
    p01_ = 0.0;
    p10_ = 0.0;
    p11_ = initial_covariance_;
}

void LineOfSightRateEstimator::TwoStateLosKalmanFilter::initialize(
    double q_b, double initial_rate) {
    q_b_ = q_b;
    q_dot_i_ = initial_rate;
    p00_ = r_;
    p01_ = 0.0;
    p10_ = 0.0;
    p11_ = initial_covariance_;
    initialized_ = true;
}

double LineOfSightRateEstimator::TwoStateLosKalmanFilter::update(
    bool valid, double z_qb, double c_omega, double dt) {
    if (!is_finite(dt) || dt <= 0.0) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const double dt_eff = std::max(dt, 1e-4);

    if (!initialized_) {
        if (valid) {
            initialize(z_qb, 0.0);
        }
        return 0.0;
    }

    // 1. 状态预测: x_pred = F * x + B * c_omega
    // F = [1, dt; 0, 1], B = [-dt; 0]
    const double q_b_pred = q_b_ + (q_dot_i_ - c_omega) * dt_eff;
    const double q_dot_i_pred = q_dot_i_;

    // 2. 协方差预测: P_pred = F * P * F^T + Q
    const double dt2 = dt_eff * dt_eff;
    const double dt3 = dt2 * dt_eff;
    const double dt4 = dt2 * dt2;
    const double q_scale = sigma_a_ * sigma_a_;
    const double q00 = q_scale * dt4 / 4.0;
    const double q01 = q_scale * dt3 / 2.0;
    const double q11 = q_scale * dt2;

    const double p00_pred = p00_ + 2.0 * dt_eff * p01_ + dt2 * p11_ + q00;
    const double p01_pred = p01_ + dt_eff * p11_ + q01;
    const double p10_pred = p01_pred;
    const double p11_pred = p11_ + q11;

    if (!is_finite(q_b_pred) || !is_finite(q_dot_i_pred) ||
        !is_finite(p00_pred) || !is_finite(p11_pred)) {
        reset();
        return std::numeric_limits<double>::quiet_NaN();
    }

    // 若视觉目标丢失 (valid == false): 保持纯惯导外推
    if (!valid) {
        q_b_ = q_b_pred;
        q_dot_i_ = q_dot_i_pred;
        p00_ = p00_pred;
        p01_ = p01_pred;
        p10_ = p10_pred;
        p11_ = p11_pred;
        return q_dot_i_;
    }

    // 3. 测量更新: z = [1, 0] * x + v
    const double innovation = z_qb - q_b_pred;
    const double s = p00_pred + r_;
    if (!is_finite(s) || s <= 0.0) {
        reset();
        return std::numeric_limits<double>::quiet_NaN();
    }

    const double k0 = p00_pred / s;
    const double k1 = p10_pred / s;

    q_b_ = q_b_pred + k0 * innovation;
    q_dot_i_ = q_dot_i_pred + k1 * innovation;

    p00_ = std::max(0.0, (1.0 - k0) * p00_pred);
    p01_ = (1.0 - k0) * p01_pred;
    p10_ = p01_;
    p11_ = std::max(0.0, p11_pred - k1 * p01_pred);

    return q_dot_i_;
}

LineOfSight GuidanceEstimator::calculate(
    const VisionResult& result, const CameraIntrinsics& intrinsics) {
    Vector3 direction{};
    return camera_direction(result, intrinsics, &direction)
               ? angles_from_direction(direction)
               : invalid_line_of_sight();
}

LineOfSight GuidanceEstimator::calculate_body(
    const VisionResult& result, const CameraIntrinsics& intrinsics,
    const RotationMatrix3& camera_to_body) {
    Vector3 camera_direction_value{};
    if (!is_valid_rotation(camera_to_body) ||
        !camera_direction(result, intrinsics, &camera_direction_value)) {
        return invalid_line_of_sight();
    }
    const Vector3 body_direction =
        normalize(multiply(camera_to_body, camera_direction_value));
    return vector_norm(body_direction) < kEpsilon
               ? invalid_line_of_sight()
               : angles_from_direction(body_direction);
}

LineOfSightRateEstimator::LineOfSightRateEstimator(
    const LineOfSightRateFilterConfig& config)
    : filter_enabled_(is_valid_rate_filter_config(config)
                          ? config.enable_filter
                          : true),
      q_y_filter_(
          is_valid_rate_filter_config(config)
              ? (config.sigma_a_pitch > 0.0 ? config.sigma_a_pitch : config.sigma_a)
              : 8.0,
          is_valid_rate_filter_config(config) ? config.sigma_q : 0.0035,
          is_valid_rate_filter_config(config) ? config.initial_covariance : 1.0),
      q_z_filter_(
          is_valid_rate_filter_config(config)
              ? (config.sigma_a_yaw > 0.0 ? config.sigma_a_yaw : config.sigma_a)
              : 8.0,
          is_valid_rate_filter_config(config) ? config.sigma_q : 0.0035,
          is_valid_rate_filter_config(config) ? config.initial_covariance : 1.0),
      sample_{} {
    reset();
}

void LineOfSightRateEstimator::reset() {
    has_previous_ = false;
    previous_q_y_ = 0.0;
    previous_q_z_ = 0.0;
    q_y_filter_.reset();
    q_z_filter_.reset();
    angular_velocity_ = {false, 0.0, 0.0};
    sample_ = {};
}

LineOfSightAngularVelocity LineOfSightRateEstimator::update(
    const LineOfSight& line_of_sight, const Vector3& body_angular_velocity,
    double dt) {
    if (!is_finite(body_angular_velocity) || !is_finite(dt) || dt <= 0.0) {
        reset();
        return angular_velocity_;
    }

    LineOfSight los_for_rotation = line_of_sight;
    if (!is_valid_line_of_sight(line_of_sight)) {
        if (filter_enabled_ && q_y_filter_.is_initialized() &&
            q_z_filter_.is_initialized()) {
            los_for_rotation = {true, q_y_filter_.estimated_angle(),
                                q_z_filter_.estimated_angle()};
        } else {
            reset();
            return angular_velocity_;
        }
    }

    double rotation_q_y = 0.0;
    double rotation_q_z = 0.0;
    if (!body_rotation_rate(los_for_rotation, body_angular_velocity,
                            &rotation_q_y, &rotation_q_z)) {
        reset();
        return angular_velocity_;
    }

    if (!filter_enabled_) {
        if (!is_valid_line_of_sight(line_of_sight)) {
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

        const double observed_q_y = (line_of_sight.q_y - previous_q_y_) / dt;
        const double observed_q_z =
            normalize_angle(line_of_sight.q_z - previous_q_z_) / dt;
        previous_q_y_ = line_of_sight.q_y;
        previous_q_z_ = line_of_sight.q_z;
        if (!is_finite(observed_q_y) || !is_finite(observed_q_z)) {
            reset();
            return angular_velocity_;
        }

        const LineOfSightAngularVelocity compensated_rate{
            true, observed_q_y + rotation_q_y, observed_q_z + rotation_q_z};
        if (!is_finite(compensated_rate.q_y) ||
            !is_finite(compensated_rate.q_z)) {
            reset();
            return angular_velocity_;
        }
        angular_velocity_ = compensated_rate;
        sample_.line_of_sight = line_of_sight;
        sample_.raw_angular_velocity = compensated_rate;
        sample_.filtered_angular_velocity = angular_velocity_;
        return angular_velocity_;
    }

    if (is_valid_line_of_sight(line_of_sight)) {
        if (!has_previous_) {
            q_y_filter_.initialize(line_of_sight.q_y);
            q_z_filter_.initialize(line_of_sight.q_z);
            has_previous_ = true;
            previous_q_y_ = line_of_sight.q_y;
            previous_q_z_ = line_of_sight.q_z;
            angular_velocity_ = {false, 0.0, 0.0};
            sample_.line_of_sight = line_of_sight;
            sample_.raw_angular_velocity = {false, 0.0, 0.0};
            sample_.filtered_angular_velocity = angular_velocity_;
            return angular_velocity_;
        }

        const double observed_q_y = (line_of_sight.q_y - previous_q_y_) / dt;
        const double observed_q_z =
            normalize_angle(line_of_sight.q_z - previous_q_z_) / dt;
        previous_q_y_ = line_of_sight.q_y;
        previous_q_z_ = line_of_sight.q_z;

        const LineOfSightAngularVelocity raw_rate{
            is_finite(observed_q_y) && is_finite(observed_q_z),
            observed_q_y + rotation_q_y, observed_q_z + rotation_q_z};
        sample_.line_of_sight = line_of_sight;
        sample_.raw_angular_velocity = raw_rate;

        const double filtered_q_y =
            q_y_filter_.update(true, line_of_sight.q_y, rotation_q_y, dt);
        const double filtered_q_z =
            q_z_filter_.update(true, line_of_sight.q_z, rotation_q_z, dt);
        if (!is_finite(filtered_q_y) || !is_finite(filtered_q_z)) {
            reset();
            return angular_velocity_;
        }
        angular_velocity_ = {true, filtered_q_y, filtered_q_z};
        sample_.filtered_angular_velocity = angular_velocity_;
        return angular_velocity_;
    } else {
        // 视觉目标丢失时，利用 IMU 角速度执行状态预测外推
        if (q_y_filter_.is_initialized() && q_z_filter_.is_initialized()) {
            const double filtered_q_y =
                q_y_filter_.update(false, 0.0, rotation_q_y, dt);
            const double filtered_q_z =
                q_z_filter_.update(false, 0.0, rotation_q_z, dt);
            if (!is_finite(filtered_q_y) || !is_finite(filtered_q_z)) {
                reset();
                return angular_velocity_;
            }
            angular_velocity_ = {true, filtered_q_y, filtered_q_z};
            sample_.line_of_sight = {false, q_y_filter_.estimated_angle(),
                                     q_z_filter_.estimated_angle()};
            sample_.raw_angular_velocity = {false, 0.0, 0.0};
            sample_.filtered_angular_velocity = angular_velocity_;
            return angular_velocity_;
        }
        reset();
        return angular_velocity_;
    }
}

LineOfSightAngularVelocity LineOfSightRateEstimator::angular_velocity() const {
    return angular_velocity_;
}

LineOfSightRateSample LineOfSightRateEstimator::sample() const {
    return sample_;
}

PngGuidanceOutput PngGuidance::calculate(
    const LineOfSight& line_of_sight,
    const LineOfSightAngularVelocity& angular_velocity, double speed_mps,
    double pitch_rad, double roll_rad, const PngGuidanceConfig& config) {
    const PngGuidanceOutput invalid_output{
        false, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, 0.0,
        0.0};
    if (!is_valid_line_of_sight(line_of_sight) || !angular_velocity.valid ||
        !is_finite(angular_velocity.q_y) ||
        !is_finite(angular_velocity.q_z) || !is_finite(speed_mps) ||
        speed_mps <= kEpsilon || !is_finite(pitch_rad) ||
        !is_finite(roll_rad) ||
        !is_valid_png_config(config)) {
        return invalid_output;
    }

    const double cos_pitch = std::cos(pitch_rad);
    const double speed_over_gravity = speed_mps / config.gravity;
    double vertical_overload =
        config.navigation_constant_y * angular_velocity.q_y *
            speed_over_gravity + cos_pitch;
    double lateral_overload =
        config.navigation_constant_z * angular_velocity.q_z *
        speed_over_gravity * cos_pitch;
    if (!is_finite(vertical_overload) || !is_finite(lateral_overload)) {
        return invalid_output;
    }

    double command_overload =
        std::hypot(vertical_overload, lateral_overload);
    if (!is_finite(command_overload)) {
        return invalid_output;
    }
    if (command_overload > config.maximum_command_overload) {
        const double scale =
            config.maximum_command_overload / command_overload;
        vertical_overload *= scale;
        lateral_overload *= scale;
        command_overload = std::hypot(vertical_overload, lateral_overload);
    }

    const double command_phase = std::atan2(lateral_overload, vertical_overload);
    Vector3 ballistic_acceleration{
        0.0, config.gravity * vertical_overload,
        config.gravity * lateral_overload};
    if (!is_finite(ballistic_acceleration)) {
        return invalid_output;
    }
    const double command_acceleration = vector_norm(ballistic_acceleration);
    if (!is_finite(command_acceleration)) {
        return invalid_output;
    }
    if (is_finite(config.maximum_command_acceleration) &&
        command_acceleration > config.maximum_command_acceleration) {
        const double scale =
            config.maximum_command_acceleration / command_acceleration;
        ballistic_acceleration.x *= scale;
        ballistic_acceleration.y *= scale;
        ballistic_acceleration.z *= scale;
        command_overload *= scale;
    }

    const double cosine_roll = std::cos(roll_rad);
    const double sine_roll = std::sin(roll_rad);
    const Vector3 body_acceleration{
        ballistic_acceleration.x,
        cosine_roll * ballistic_acceleration.y +
            sine_roll * ballistic_acceleration.z,
        -sine_roll * ballistic_acceleration.y +
            cosine_roll * ballistic_acceleration.z};
    const Vector3 body_overload{
        body_acceleration.x / config.gravity,
        body_acceleration.y / config.gravity,
        body_acceleration.z / config.gravity};
    if (!is_finite(body_overload) || !is_finite(command_phase)) {
        return invalid_output;
    }
    return {true, ballistic_acceleration, body_acceleration, body_overload,
            command_overload, command_phase};
}
