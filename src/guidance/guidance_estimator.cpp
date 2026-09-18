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
    return is_finite(config.process_noise) &&
           is_finite(config.measurement_noise) &&
           is_finite(config.initial_covariance) && config.process_noise >= 0.0 &&
           config.measurement_noise > 0.0 && config.initial_covariance >= 0.0;
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

LineOfSightRateEstimator::ScalarKalmanFilter::ScalarKalmanFilter(
    const LineOfSightRateFilterConfig& config)
    : config_(is_valid_rate_filter_config(config)
                  ? config
                  : LineOfSightRateFilterConfig{}) {
    reset();
}

void LineOfSightRateEstimator::ScalarKalmanFilter::reset() {
    rate_ = 0.0;
    rate_covariance_ = config_.initial_covariance;
}

void LineOfSightRateEstimator::ScalarKalmanFilter::initialize(double rate) {
    rate_ = rate;
    rate_covariance_ = config_.initial_covariance;
}

double LineOfSightRateEstimator::ScalarKalmanFilter::update(
    double measurement, double dt) {
    const double predicted_rate = rate_;
    const double predicted_covariance =
        rate_covariance_ + config_.process_noise * dt * dt;
    const double innovation = measurement - predicted_rate;
    const double innovation_covariance =
        predicted_covariance + config_.measurement_noise;
    if (!is_finite(predicted_rate) || !is_finite(predicted_covariance) ||
        !is_finite(innovation_covariance) || innovation_covariance <= 0.0) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const double gain = predicted_covariance / innovation_covariance;
    rate_ = predicted_rate + gain * innovation;
    rate_covariance_ = (1.0 - gain) * predicted_covariance;
    return rate_;
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
    const LineOfSight& line_of_sight, const Vector3& body_angular_velocity,
    double dt) {
    if (!is_valid_line_of_sight(line_of_sight) ||
        !is_finite(body_angular_velocity) || !is_finite(dt) || dt <= 0.0) {
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
    double rotation_q_y = 0.0;
    double rotation_q_z = 0.0;
    if (!is_finite(observed_q_y) || !is_finite(observed_q_z) ||
        !body_rotation_rate(line_of_sight, body_angular_velocity,
                            &rotation_q_y, &rotation_q_z)) {
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
    sample_.line_of_sight = line_of_sight;
    sample_.raw_angular_velocity = compensated_rate;

    if (!filter_enabled_) {
        angular_velocity_ = compensated_rate;
        sample_.filtered_angular_velocity = angular_velocity_;
        return angular_velocity_;
    }

    if (!has_rate_) {
        q_y_filter_.initialize(compensated_rate.q_y);
        q_z_filter_.initialize(compensated_rate.q_z);
        has_rate_ = true;
        angular_velocity_ = compensated_rate;
        sample_.filtered_angular_velocity = angular_velocity_;
        return angular_velocity_;
    }

    const double filtered_q_y = q_y_filter_.update(compensated_rate.q_y, dt);
    const double filtered_q_z = q_z_filter_.update(compensated_rate.q_z, dt);
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
    const LineOfSightAngularVelocity& angular_velocity, double speed_mps,
    double pitch_rad, const PngGuidanceConfig& config) {
    const PngGuidanceOutput invalid_output{
        false, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, 0.0,
        0.0};
    if (!is_valid_line_of_sight(line_of_sight) || !angular_velocity.valid ||
        !is_finite(angular_velocity.q_y) ||
        !is_finite(angular_velocity.q_z) || !is_finite(speed_mps) ||
        speed_mps <= kEpsilon || !is_finite(pitch_rad) ||
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
    Vector3 body_acceleration{
        0.0, config.gravity * vertical_overload,
        config.gravity * lateral_overload};
    if (!is_finite(body_acceleration)) {
        return invalid_output;
    }
    const double command_acceleration = vector_norm(body_acceleration);
    if (!is_finite(command_acceleration)) {
        return invalid_output;
    }
    if (is_finite(config.maximum_command_acceleration) &&
        command_acceleration > config.maximum_command_acceleration) {
        const double scale =
            config.maximum_command_acceleration / command_acceleration;
        body_acceleration.x *= scale;
        body_acceleration.y *= scale;
        body_acceleration.z *= scale;
        command_overload *= scale;
    }

    const Vector3 body_overload{
        body_acceleration.x / config.gravity,
        body_acceleration.y / config.gravity,
        body_acceleration.z / config.gravity};
    if (!is_finite(body_overload) || !is_finite(command_phase)) {
        return invalid_output;
    }
    return {true, body_acceleration, body_acceleration, body_overload,
            command_overload, command_phase};
}
