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

bool is_valid_png_config(const PngGuidanceConfig& config) {
    const bool maximum_acceleration_valid =
        (is_finite(config.maximum_command_acceleration) &&
         config.maximum_command_acceleration >= 0.0) ||
        (std::isinf(config.maximum_command_acceleration) &&
         config.maximum_command_acceleration > 0.0);
    return mosas::guidance::valid_control_channel(config.control_channel) &&
           is_finite(config.navigation_constant_y) &&
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

/**
 * @brief 将目标像素中心反投影为归一化相机方向。
 * @param result 含目标中心像素坐标的视觉结果。
 * @param intrinsics 相机焦距和主点，单位为像素。
 * @param direction 非空输出指针；成功时写入相机系单位方向向量。
 * @return 目标、内参和输出均有效时为 true。
 * @note 图像 v 轴向下，因此计算相机 +y 分量时反转像素纵坐标偏差。
 */
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

/**
 * @brief 将相机/镖体系方向向量转换为俯仰和偏航 LOS 角。
 * @param direction 坐标系中 +x 向前、+y 向上、+z 向右的方向向量。
 * @return 以 rad 表示的 q_y/q_z；零向量或非有限输入返回无效 LOS。
 */
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

}  // namespace

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
    const LineOfSightRateFilterConfig& /*config*/)
    : sample_{} {
    reset();
}

void LineOfSightRateEstimator::reset() {
    has_previous_ = false;
    previous_q_y_ = 0.0;
    previous_q_z_ = 0.0;
    angular_velocity_ = {false, 0.0, 0.0};
    sample_ = {};
}

LineOfSightAngularVelocity LineOfSightRateEstimator::update(
    const LineOfSight& line_of_sight, const Vector3& body_angular_velocity,
    double dt) {
    (void)body_angular_velocity;
    if (!is_finite(dt) || dt <= 0.0 || !is_valid_line_of_sight(line_of_sight)) {
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

    angular_velocity_ = {true, observed_q_y, observed_q_z};
    sample_.line_of_sight = line_of_sight;
    sample_.raw_angular_velocity = angular_velocity_;
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
    const RotationMatrix3& body_to_navigation, const PngGuidanceConfig& config,
    PngCalculationTrace* trace) {
    if (trace) *trace = {};
    const PngGuidanceOutput invalid_output{
        false, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, 0.0,
        0.0};
    if (!is_valid_line_of_sight(line_of_sight) || !angular_velocity.valid ||
        !is_finite(angular_velocity.q_y) ||
        !is_finite(angular_velocity.q_z) || !is_finite(speed_mps) ||
        speed_mps <= kEpsilon || !is_valid_rotation(body_to_navigation) ||
        !is_valid_png_config(config)) {
        return invalid_output;
    }

    const Vector3 inertial_direction = multiply(
        body_to_navigation, line_of_sight_unit(line_of_sight));
    const double inertial_elevation = std::atan2(
        inertial_direction.y, std::hypot(inertial_direction.x, inertial_direction.z));
    // 先在角速度域扣除自然抛物线基准，不在输出过载中再次补偿重力。
    const double parabolic_elevation_rate =
        -(0.5 * config.gravity / speed_mps) * std::cos(inertial_elevation);
    const double compensated_elevation_rate =
        angular_velocity.q_y - parabolic_elevation_rate;
    const double pitch_rad = std::atan2(
        body_to_navigation.m10,
        std::hypot(body_to_navigation.m00, body_to_navigation.m20));
    const double roll_rad = std::atan2(-body_to_navigation.m12, body_to_navigation.m11);
    const double cos_pitch = std::cos(pitch_rad);
    const double speed_over_gravity = speed_mps / config.gravity;
    const double vertical_overload =
        config.navigation_constant_y * compensated_elevation_rate * speed_over_gravity;
    const double lateral_overload =
        config.navigation_constant_z * angular_velocity.q_z *
        speed_over_gravity * cos_pitch;
    const double raw_command_overload = std::hypot(vertical_overload, lateral_overload);
    if (!is_finite(vertical_overload) || !is_finite(lateral_overload) ||
        !is_finite(raw_command_overload)) {
        return invalid_output;
    }
    if (trace) {
        trace->inertial_los_elevation_rad = inertial_elevation;
        trace->measured_elevation_rate_rad_s = angular_velocity.q_y;
        trace->parabolic_elevation_rate_rad_s = parabolic_elevation_rate;
        trace->compensated_elevation_rate_rad_s = compensated_elevation_rate;
        trace->raw_navigation_overload = {0.0, vertical_overload, lateral_overload};
        trace->raw_command_overload = raw_command_overload;
    }

    const double cosine_roll = std::cos(roll_rad);
    const double sine_roll = std::sin(roll_rad);
    Vector3 body_overload{
        0.0,
        cosine_roll * vertical_overload + sine_roll * lateral_overload,
        -sine_roll * vertical_overload + cosine_roll * lateral_overload};
    if (!is_finite(body_overload)) {
        return invalid_output;
    }
    // 通道属于镖体系；必须先完成 roll 转换，再选轴，避免关闭轴挤占限幅额度。
    mosas::guidance::select_control_channel(
        config.control_channel, body_overload.y, body_overload.z);
    double command_overload = std::hypot(body_overload.y, body_overload.z);
    if (!is_finite(command_overload)) {
        return invalid_output;
    }
    if (command_overload > config.maximum_command_overload) {
        if (trace) trace->overload_limited = true;
        const double scale = config.maximum_command_overload / command_overload;
        body_overload.y *= scale;
        body_overload.z *= scale;
        command_overload = std::hypot(body_overload.y, body_overload.z);
    }

    // 弹道系字段只反映所选通道的实际指令，不再参与控制计算。
    const Vector3 ballistic_overload{
        0.0,
        cosine_roll * body_overload.y - sine_roll * body_overload.z,
        sine_roll * body_overload.y + cosine_roll * body_overload.z};
    if (trace) trace->after_overload_limit = ballistic_overload;
    const double command_phase = command_overload == 0.0
        ? 0.0 : std::atan2(ballistic_overload.z, ballistic_overload.y);
    Vector3 body_acceleration{
        0.0, config.gravity * body_overload.y, config.gravity * body_overload.z};
    if (!is_finite(body_acceleration)) {
        return invalid_output;
    }
    const double command_acceleration = vector_norm(body_acceleration);
    if (!is_finite(command_acceleration)) {
        return invalid_output;
    }
    if (is_finite(config.maximum_command_acceleration) &&
        command_acceleration > config.maximum_command_acceleration) {
        if (trace) trace->acceleration_limited = true;
        const double scale = config.maximum_command_acceleration / command_acceleration;
        body_acceleration.y *= scale;
        body_acceleration.z *= scale;
        command_overload *= scale;
    }
    body_overload = {0.0, body_acceleration.y / config.gravity,
                    body_acceleration.z / config.gravity};
    const Vector3 ballistic_acceleration{
        0.0,
        cosine_roll * body_acceleration.y - sine_roll * body_acceleration.z,
        sine_roll * body_acceleration.y + cosine_roll * body_acceleration.z};
    if (!is_finite(body_overload) || !is_finite(ballistic_acceleration) ||
        !is_finite(command_phase)) {
        return invalid_output;
    }
    if (trace) {
        trace->navigation_acceleration = ballistic_acceleration;
        trace->body_overload = body_overload;
        trace->valid = true;
    }
    return {true, ballistic_acceleration, body_acceleration, body_overload,
            command_overload, command_phase};
}
