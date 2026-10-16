#ifndef MOSAS_RUNTIME_ATTITUDE_ESTIMATOR_HPP
#define MOSAS_RUNTIME_ATTITUDE_ESTIMATOR_HPP

#include <mosas/runtime/imu_angular_velocity.hpp>
#include <mosas/runtime/rotation_matrix3.hpp>

#include <cstdint>

namespace mosas::runtime {

struct AttitudeEstimatorConfig {
    double gravity_mps2 = 9.80665;
    /** @brief 标定的发射速度初值，单位 m/s；必须为有限正数。 */
    double launch_speed_mps = 100.0;
    // 静止判定和起飞前低比力保护使用的加速度模长相对重力最大偏差，单位 m/s²。
    double stationary_acceleration_tolerance_mps2 = 0.3;
    // 静止判定时角速度模长的最大值，单位 rad/s。
    double gyro_stationary_threshold_rad_s = 0.2;
    // 静止阶段陀螺仪零偏 EMA 系数。
    double gyro_bias_filter_alpha = 0.02;
    // 相对启动基线的 X 轴加速度增量阈值，单位 m/s²（1.5g）。
    double launch_ax_threshold_mps2 = 14.709975;
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
    double roll_rad = 0.0;
    /** @brief 与当前姿态同一 IMU 时刻的纵向速度估计，单位 m/s。 */
    double speed_mps = 0.0;
    bool launched = false;
    bool valid = false;
    TimestampNs launch_timestamp_ns = -1;
    // 扣除静止阶段估计零偏后的角速度，供姿态和制导使用。
    Vector3 corrected_angular_velocity{0.0, 0.0, 0.0};
};

class AttitudeEstimator final {
public:
    /**
     * @brief 创建姿态、起飞状态和纵向速度估计器。
     * @param config 估计参数；无效配置整体回退为默认配置。
     */
    explicit AttitudeEstimator(const AttitudeEstimatorConfig& config = {});

    AttitudeEstimator(const AttitudeEstimator&) = delete;
    AttitudeEstimator& operator=(const AttitudeEstimator&) = delete;

    /**
     * @brief 更新姿态，并在确认起飞后的连续样本上积分纵向速度。
     * @param sample 镖体系 IMU 样本；加速度单位 m/s²，角速度单位 rad/s。
     * @return 同步状态；无效或非递增样本返回上一状态。
     * @note 起飞确认帧保持标定初速；随后使用当前帧俯仰角计算
     * V += (ax - gravity * sin(pitch)) * dt，跨断流区间不积分。
     */
    AttitudeEstimate update(const ImuAngularVelocitySample& sample) noexcept;

    /** @brief 清除姿态、零偏和起飞状态，并将速度恢复为标定发射速度。 */
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
    static double vector_norm(const Vector3& value) noexcept;
    static Quaternion multiply(const Quaternion& lhs,
                               const Quaternion& rhs) noexcept;
    static Quaternion normalize(const Quaternion& value) noexcept;
    static Quaternion from_roll_pitch(double roll_rad,
                                      double pitch_rad) noexcept;
    static Quaternion delta_from_body_rate(const Vector3& angular_velocity,
                                           double dt_seconds) noexcept;
    static RotationMatrix3 to_matrix(const Quaternion& value) noexcept;
    static double pitch_from_matrix(const RotationMatrix3& matrix) noexcept;
    static double roll_from_matrix(const RotationMatrix3& matrix) noexcept;

    AttitudeEstimate make_estimate(TimestampNs timestamp_ns,
                                   bool valid) const noexcept;
    bool is_stationary(const Vector3& acceleration,
                       const Vector3& angular_velocity) const noexcept;
    void update_gyro_bias(const Vector3& angular_velocity) noexcept;
    bool update_prelaunch_attitude(const Vector3& acceleration) noexcept;
    void update_launch_state(double current_ax,
                             TimestampNs timestamp_ns) noexcept;

    AttitudeEstimatorConfig config_;
    Quaternion orientation_{};
    Vector3 gyro_bias_{0.0, 0.0, 0.0};
    Vector3 corrected_angular_velocity_{0.0, 0.0, 0.0};
    double speed_mps_ = 0.0;
    double launch_baseline_x_ = 0.0;
    TimestampNs launch_candidate_start_ns_ = -1;
    TimestampNs last_timestamp_ns_ = -1;
    bool has_launch_baseline_ = false;
    bool has_gyro_bias_ = false;
    bool has_timestamp_ = false;
    bool launched_ = false;
    TimestampNs launch_timestamp_ns_ = -1;
    AttitudeEstimate last_estimate_{};
};

}  // namespace mosas::runtime

#endif  // MOSAS_RUNTIME_ATTITUDE_ESTIMATOR_HPP
