#include "dart_condition.hpp"

#include <algorithm>
#include <cmath>

namespace {

constexpr double kEpsilon = 1e-12;
constexpr double kGravityVectorEpsilon = 1e-9;
constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

bool is_finite(const Vector3& value) {
    return std::isfinite(value.x) && std::isfinite(value.y) &&
           std::isfinite(value.z);
}

double vector_norm(const Vector3& value) {
    return std::sqrt(value.x * value.x + value.y * value.y +
                     value.z * value.z);
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
        const double norm = std::sqrt(value.w * value.w + value.x * value.x +
                                      value.y * value.y + value.z * value.z);
        if (!std::isfinite(norm) || norm < kEpsilon) {
            return {1.0, 0.0, 0.0, 0.0};
        }
        return {value.w / norm, value.x / norm, value.y / norm,
                value.z / norm};
    }

    static Quaternion from_euler(const EulerAngles& angles) {
        const double half_roll = angles.roll * 0.5;
        const double half_pitch = angles.pitch * 0.5;
        const double half_yaw = angles.yaw * 0.5;
        const double cr = std::cos(half_roll);
        const double sr = std::sin(half_roll);
        const double cp = std::cos(half_pitch);
        const double sp = std::sin(half_pitch);
        const double cy = std::cos(half_yaw);
        const double sy = std::sin(half_yaw);

        return {
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        };
    }

    static Quaternion from_angular_velocity(const Vector3& angular_velocity,
                                             double dt) {
        const Vector3 delta{
            angular_velocity.x * dt,
            angular_velocity.y * dt,
            angular_velocity.z * dt,
        };
        const double angle = vector_norm(delta);
        if (angle < kEpsilon) {
            return normalized({1.0, delta.x * 0.5, delta.y * 0.5,
                               delta.z * 0.5});
        }

        const double half_angle = angle * 0.5;
        const double scale = std::sin(half_angle) / angle;
        return {std::cos(half_angle), delta.x * scale, delta.y * scale,
                delta.z * scale};
    }

    static EulerAngles to_euler(const Quaternion& value) {
        const double roll = std::atan2(
            2.0 * (value.w * value.x + value.y * value.z),
            1.0 - 2.0 * (value.x * value.x + value.y * value.y));
        const double pitch = std::asin(clamp_unit(
            2.0 * (value.w * value.y - value.z * value.x)));
        const double yaw = std::atan2(
            2.0 * (value.w * value.z + value.x * value.y),
            1.0 - 2.0 * (value.y * value.y + value.z * value.z));
        return {normalize_angle(roll), pitch, normalize_angle(yaw)};
    }
};

}  // namespace

bool DartCondition::initialize(const Vector3& acceleration) {
    if (state_ == State::Flying || !is_finite(acceleration) ||
        vector_norm(acceleration) < kGravityVectorEpsilon) {
        return false;
    }

    const EulerAngles initial_angles{
        std::atan2(acceleration.y, acceleration.z),
        std::atan2(-acceleration.x,
                   std::sqrt(acceleration.y * acceleration.y +
                             acceleration.z * acceleration.z)),
        0.0,
    };
    const Quaternion initial_orientation = QuaternionMath::normalized(
        QuaternionMath::from_euler(initial_angles));
    orientation_w_ = initial_orientation.w;
    orientation_x_ = initial_orientation.x;
    orientation_y_ = initial_orientation.y;
    orientation_z_ = initial_orientation.z;
    angles_ = QuaternionMath::to_euler(
        {orientation_w_, orientation_x_, orientation_y_, orientation_z_});
    state_ = State::Ready;
    return true;
}

void DartCondition::launch() {
    if (state_ == State::Ready) {
        state_ = State::Flying;
    }
}

void DartCondition::reset() {
    orientation_w_ = 1.0;
    orientation_x_ = 0.0;
    orientation_y_ = 0.0;
    orientation_z_ = 0.0;
    angles_ = {0.0, 0.0, 0.0};
    state_ = State::Uninitialized;
}

EulerAngles DartCondition::update(const Vector3& acceleration,
                                  const Vector3& angular_velocity, double dt) {
    // 飞行过程中处于失重状态，加速度计不用于姿态校正。
    (void)acceleration;
    if (state_ != State::Flying || !is_finite(angular_velocity) ||
        !std::isfinite(dt) || dt <= 0.0) {
        return angles_;
    }

    const Quaternion delta =
        QuaternionMath::from_angular_velocity(angular_velocity, dt);
    const Quaternion current{
        orientation_w_, orientation_x_, orientation_y_, orientation_z_};
    const Quaternion updated =
        QuaternionMath::normalized(QuaternionMath::multiply(current, delta));
    orientation_w_ = updated.w;
    orientation_x_ = updated.x;
    orientation_y_ = updated.y;
    orientation_z_ = updated.z;
    angles_ = QuaternionMath::to_euler(
        {orientation_w_, orientation_x_, orientation_y_, orientation_z_});
    return angles_;
}

EulerAngles DartCondition::attitude() const {
    return angles_;
}

bool DartCondition::initialized() const {
    return state_ != State::Uninitialized;
}

bool DartCondition::flying() const {
    return state_ == State::Flying;
}
