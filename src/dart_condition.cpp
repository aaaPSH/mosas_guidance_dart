#include "dart_condition.hpp"

#include <algorithm>
#include <cmath>

namespace {

constexpr double kEpsilon = 1e-12;
constexpr double kGravityVectorEpsilon = 1e-9;
constexpr double kGravityAcceleration = 9.80665;
constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kEulerSingularityTolerance = 1e-12;

bool is_finite_vector(const Vector3& value) {
    return std::isfinite(value.x) && std::isfinite(value.y) &&
           std::isfinite(value.z);
}

double vector_norm(const Vector3& value) {
    return std::hypot(std::hypot(value.x, value.y), value.z);
}

double normalize_angle(double angle) {
    angle = std::fmod(angle + kPi, kTwoPi);
    if (angle < 0.0) {
        angle += kTwoPi;
    }
    return angle - kPi;
}

double clamp_unit(double value) {
    return std::max(-1.0, std::min(1.0, value));
}

struct Quaternion {
    double w;
    double x;
    double y;
    double z;
};

struct QuaternionMath {
    static Quaternion multiply(const Quaternion& lhs, const Quaternion& rhs) {
        return {
            lhs.w * rhs.w - lhs.x * rhs.x - lhs.y * rhs.y - lhs.z * rhs.z,
            lhs.w * rhs.x + lhs.x * rhs.w + lhs.y * rhs.z - lhs.z * rhs.y,
            lhs.w * rhs.y - lhs.x * rhs.z + lhs.y * rhs.w + lhs.z * rhs.x,
            lhs.w * rhs.z + lhs.x * rhs.y - lhs.y * rhs.x + lhs.z * rhs.w,
        };
    }

    static Quaternion normalized(const Quaternion& value) {
        const double norm = std::hypot(
            std::hypot(value.w, value.x), std::hypot(value.y, value.z));
        if (!std::isfinite(norm) || norm < kEpsilon) {
            return {1.0, 0.0, 0.0, 0.0};
        }
        return {value.w / norm, value.x / norm, value.y / norm,
                value.z / norm};
    }

    static Quaternion conjugate(const Quaternion& value) {
        return {value.w, -value.x, -value.y, -value.z};
    }

    static Vector3 rotate_vector(const Quaternion& orientation,
                                 const Vector3& vector) {
        const Quaternion vector_quaternion{0.0, vector.x, vector.y, vector.z};
        const Quaternion rotated = multiply(
            multiply(orientation, vector_quaternion), conjugate(orientation));
        return {rotated.x, rotated.y, rotated.z};
    }

    static bool is_finite(const Quaternion& value) {
        return std::isfinite(value.w) && std::isfinite(value.x) &&
               std::isfinite(value.y) && std::isfinite(value.z);
    }

    static Quaternion from_euler(const EulerAngles& angles) {
        const double half_roll = angles.roll * 0.5;
        const double half_pitch = angles.pitch * 0.5;
        const double half_yaw = angles.yaw * 0.5;
        const Quaternion roll_rotation{
            std::cos(half_roll), std::sin(half_roll), 0.0, 0.0};
        const Quaternion pitch_rotation{
            std::cos(half_pitch), 0.0, 0.0, std::sin(half_pitch)};
        const Quaternion yaw_rotation{
            std::cos(half_yaw), 0.0, std::sin(half_yaw), 0.0};

        return normalized(multiply(
            multiply(yaw_rotation, pitch_rotation), roll_rotation));
    }

    static bool from_angular_velocity(const Vector3& angular_velocity,
                                      double dt, Quaternion* result) {
        const Vector3 delta{
            angular_velocity.x * dt,
            angular_velocity.y * dt,
            angular_velocity.z * dt,
        };
        if (!is_finite_vector(delta)) {
            return false;
        }
        const double angle = vector_norm(delta);
        if (!std::isfinite(angle)) {
            return false;
        }
        if (angle < kEpsilon) {
            *result = normalized({1.0, delta.x * 0.5, delta.y * 0.5,
                                  delta.z * 0.5});
            return true;
        }

        const double half_angle = angle * 0.5;
        const double scale = std::sin(half_angle) / angle;
        *result = {std::cos(half_angle), delta.x * scale,
                   delta.y * scale, delta.z * scale};
        return is_finite(*result);
    }

    static EulerAngles to_euler(const Quaternion& value) {
        const double matrix_r00 =
            1.0 - 2.0 * (value.y * value.y + value.z * value.z);
        const double matrix_r10 =
            2.0 * (value.x * value.y + value.w * value.z);
        const double matrix_r11 =
            1.0 - 2.0 * (value.x * value.x + value.z * value.z);
        const double matrix_r12 =
            2.0 * (value.y * value.z - value.w * value.x);
        const double matrix_r02 =
            2.0 * (value.x * value.z + value.w * value.y);
        const double matrix_r22 =
            1.0 - 2.0 * (value.x * value.x + value.y * value.y);
        const double matrix_r20 =
            2.0 * (value.x * value.z - value.w * value.y);
        const double pitch = std::asin(clamp_unit(matrix_r10));
        if (std::abs(std::abs(matrix_r10) - 1.0) <=
            kEulerSingularityTolerance) {
            // pitch 为 ±90 度时 yaw 和 roll 不可分别观测，固定 yaw 为零，
            // 使用两者的可观测组合角保持旋转姿态连续。
            const double singular_roll =
                matrix_r10 > 0.0
                    ? std::atan2(matrix_r02, matrix_r22)
                    : std::atan2(-matrix_r02, matrix_r22);
            return {pitch, 0.0, normalize_angle(singular_roll)};
        }
        const double roll = std::atan2(-matrix_r12, matrix_r11);
        const double yaw = std::atan2(
            -matrix_r20, matrix_r00);
        return {pitch, normalize_angle(yaw), normalize_angle(roll)};
    }
};

}  // namespace

DartCondition::DartCondition(double launch_speed)
    : launch_speed_(std::isfinite(launch_speed) && launch_speed >= 0.0
                        ? launch_speed
                        : 0.0) {}

bool DartCondition::initialize(const Vector3& acceleration) {
    if (state_ == State::Flying || !is_finite_vector(acceleration) ||
        vector_norm(acceleration) < kGravityVectorEpsilon) {
        return false;
    }

    const EulerAngles initial_angles{
        std::atan2(acceleration.x,
                   std::hypot(acceleration.y, acceleration.z)),
        0.0,
        std::atan2(-acceleration.z, acceleration.y),
    };
    const Quaternion initial_orientation = QuaternionMath::normalized(
        QuaternionMath::from_euler(initial_angles));
    orientation_w_ = initial_orientation.w;
    orientation_x_ = initial_orientation.x;
    orientation_y_ = initial_orientation.y;
    orientation_z_ = initial_orientation.z;
    angles_ = QuaternionMath::to_euler(
        {orientation_w_, orientation_x_, orientation_y_, orientation_z_});
    velocity_ = {0.0, 0.0, 0.0};
    state_ = State::Ready;
    return true;
}

void DartCondition::launch() {
    if (state_ == State::Ready) {
        const Quaternion orientation{
            orientation_w_, orientation_x_, orientation_y_, orientation_z_};
        // 将机体 +x 轴转换为地面坐标系的发射方向。
        velocity_ = QuaternionMath::rotate_vector(
            orientation, {1.0, 0.0, 0.0});
        velocity_.x *= launch_speed_;
        velocity_.y *= launch_speed_;
        velocity_.z *= launch_speed_;
        state_ = State::Flying;
    }
}

void DartCondition::reset() {
    orientation_w_ = 1.0;
    orientation_x_ = 0.0;
    orientation_y_ = 0.0;
    orientation_z_ = 0.0;
    angles_ = {0.0, 0.0, 0.0};
    velocity_ = {0.0, 0.0, 0.0};
    state_ = State::Uninitialized;
}

EulerAngles DartCondition::update(const Vector3& acceleration,
                                  const Vector3& angular_velocity, double dt) {
    if (state_ != State::Flying || !is_finite_vector(acceleration) ||
        !is_finite_vector(angular_velocity) ||
        !std::isfinite(dt) || dt <= 0.0) {
        return angles_;
    }

    // 将机体坐标系加速度转换到世界坐标系，并扣除世界坐标系重力。
    const Quaternion current{
        orientation_w_, orientation_x_, orientation_y_, orientation_z_};
    Vector3 world_acceleration =
        QuaternionMath::rotate_vector(current, acceleration);
    if (!is_finite_vector(world_acceleration)) {
        return angles_;
    }
    world_acceleration.y -= kGravityAcceleration;
    const Vector3 updated_velocity{
        velocity_.x + world_acceleration.x * dt,
        velocity_.y + world_acceleration.y * dt,
        velocity_.z + world_acceleration.z * dt,
    };
    if (!is_finite_vector(updated_velocity)) {
        return angles_;
    }

    Quaternion delta{};
    if (!QuaternionMath::from_angular_velocity(angular_velocity, dt, &delta)) {
        return angles_;
    }
    const Quaternion updated = QuaternionMath::multiply(current, delta);
    if (!QuaternionMath::is_finite(updated)) {
        return angles_;
    }
    const Quaternion normalized_updated = QuaternionMath::normalized(updated);
    if (!QuaternionMath::is_finite(normalized_updated)) {
        return angles_;
    }
    velocity_ = updated_velocity;
    orientation_w_ = normalized_updated.w;
    orientation_x_ = normalized_updated.x;
    orientation_y_ = normalized_updated.y;
    orientation_z_ = normalized_updated.z;
    angles_ = QuaternionMath::to_euler(
        {orientation_w_, orientation_x_, orientation_y_, orientation_z_});
    return angles_;
}

EulerAngles DartCondition::attitude() const {
    return angles_;
}

Vector3 DartCondition::velocity() const {
    return velocity_;
}

bool DartCondition::initialized() const {
    return state_ != State::Uninitialized;
}

bool DartCondition::flying() const {
    return state_ == State::Flying;
}
