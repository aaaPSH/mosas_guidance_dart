#ifndef MOSAS_RUNTIME_ATTITUDE_ESTIMATOR_HPP
#define MOSAS_RUNTIME_ATTITUDE_ESTIMATOR_HPP

#include <mosas/guidance/guidance_estimator.hpp>
#include <mosas/runtime/imu_angular_velocity.hpp>

#include <cstdint>

namespace mosas::runtime {

struct AttitudeEstimatorConfig {
    double gravity_mps2 = 9.80665;
    // 取 1.0 时直接使用当前加速度，不进行低通滤波。
    double acceleration_filter_alpha = 1.0;
    double launch_ax_threshold_mps2 = 5.0;
    TimestampNs launch_confirm_ns = 20'000'000;
    bool accelerometer_specific_force = true;
};

struct AttitudeEstimate {
    TimestampNs timestamp_ns = -1;
    RotationMatrix3 body_to_navigation{
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0};
    double pitch_rad = 0.0;
    bool launched = false;
    bool valid = false;
};

class AttitudeEstimator final {
public:
    explicit AttitudeEstimator(const AttitudeEstimatorConfig& config = {});

    AttitudeEstimator(const AttitudeEstimator&) = delete;
    AttitudeEstimator& operator=(const AttitudeEstimator&) = delete;

    AttitudeEstimate update(const ImuAngularVelocitySample& sample) noexcept;

    void reset() noexcept;

private:
    struct Quaternion {
        double w = 1.0;
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    static bool valid_config(const AttitudeEstimatorConfig& config) noexcept;
    static bool finite_vector(const Vector3& value) noexcept;
    static Quaternion multiply(const Quaternion& lhs,
                               const Quaternion& rhs) noexcept;
    static Quaternion normalize(const Quaternion& value) noexcept;
    static Quaternion from_pitch(double pitch_rad) noexcept;
    static Quaternion delta_from_body_rate(const Vector3& angular_velocity,
                                           double dt_seconds) noexcept;
    static RotationMatrix3 to_matrix(const Quaternion& value) noexcept;
    static double pitch_from_matrix(const RotationMatrix3& matrix) noexcept;

    AttitudeEstimate make_estimate(TimestampNs timestamp_ns,
                                   bool valid) const noexcept;
    bool update_prelaunch_attitude(const Vector3& acceleration) noexcept;
    void update_launch_state(TimestampNs timestamp_ns) noexcept;

    AttitudeEstimatorConfig config_;
    Quaternion orientation_{};
    Vector3 filtered_acceleration_{0.0, 0.0, 0.0};
    double launch_baseline_x_ = 0.0;
    TimestampNs launch_candidate_start_ns_ = -1;
    TimestampNs last_timestamp_ns_ = -1;
    bool has_filtered_acceleration_ = false;
    bool has_launch_baseline_ = false;
    bool has_timestamp_ = false;
    bool launched_ = false;
    AttitudeEstimate last_estimate_{};
};

}  // namespace mosas::runtime

#endif  // MOSAS_RUNTIME_ATTITUDE_ESTIMATOR_HPP
