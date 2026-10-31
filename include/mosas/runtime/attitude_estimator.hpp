#ifndef MOSAS_RUNTIME_ATTITUDE_ESTIMATOR_HPP
#define MOSAS_RUNTIME_ATTITUDE_ESTIMATOR_HPP

#include <mosas/runtime/imu_angular_velocity.hpp>
#include <mosas/runtime/rotation_matrix3.hpp>

#include <cstdint>

namespace mosas::runtime {

/** @brief 发射触发判据；实例构造时选择，运行中不可热切换。 */
enum class LaunchDetectionMode {
    acceleration, ///< 持续加速达到确认时长即触发，不等待低比力离架。
    two_stage,    ///< 加速确认后还须连续低比力确认离架。
};

struct AttitudeEstimatorConfig {
    /** @brief 默认保留单阶段加速触发；两种模式共用标定、异常保护和积分流程。 */
    LaunchDetectionMode launch_detection_mode = LaunchDetectionMode::acceleration;
    double gravity_mps2 = 9.80665;
    /** @brief 触发确认帧使用的标定速度初值，单位 m/s；必须为有限正数。 */
    double launch_speed_mps = 100.0;
    // 静止校准允许的比力模长相对重力最大偏差，单位 m/s²。
    double stationary_acceleration_tolerance_mps2 = 0.3;
    /** @brief 静止判定的原始角速度模长上限，单位 rad/s，默认约 2.86°/s。 */
    double gyro_stationary_threshold_rad_s = 0.05;
    // 静止阶段陀螺仪零偏 EMA 系数。
    double gyro_bias_filter_alpha = 0.02;
    /** @brief 连续静止段内重力矢量的 EMA 系数，必须位于 (0, 1]。 */
    double accel_filter_alpha = 0.02;
    /** @brief 静止姿态确认时长，单位 ns；不得低于 200 ms，扰动后重新计时。 */
    TimestampNs stationary_confirm_ns = 200'000'000;
    /** @brief 相对静止基线的 X 轴加速度增量阈值，单位 m/s²；两种模式共用。 */
    double launch_ax_threshold_mps2 = 14.709975;
    /** @brief 架上加速的连续确认时长，单位 ns。 */
    TimestampNs launch_confirm_ns = 20'000'000;
    /** @brief two_stage 的离架比力模长上限，单位 m/s²；须低于静止校准区间。 */
    double launch_release_acceleration_max_mps2 = 3.0;
    /** @brief two_stage 的低比力连续确认时长，单位 ns。 */
    TimestampNs launch_release_confirm_ns = 10'000'000;
    /** @brief two_stage 等待离架的最长时长，单位 ns；须大于离架确认时长，超时重新校准。 */
    TimestampNs launch_release_timeout_ns = 500'000'000;
    /** @brief IMU 连续性的最大样本间隔，单位 ns；断流取消候选且不跨段积分。 */
    TimestampNs maximum_sample_gap_ns = 20'000'000;
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
    /** @brief 所选发射判据已确认；acceleration 不代表已实际离架。 */
    bool launched = false;
    bool valid = false;
    /** @brief 所选判据的确认帧时刻，单位 ns；不是候选起点。 */
    TimestampNs launch_timestamp_ns = -1;
    // 扣除静止阶段估计零偏后的角速度，供姿态和制导使用。
    Vector3 corrected_angular_velocity{0.0, 0.0, 0.0};
};

class AttitudeEstimator final {
public:
    /**
     * @brief 创建姿态、发射触发状态和纵向速度估计器；模式在构造时固定。
     * @param config 估计参数；无效配置整体回退为默认配置。
     */
    explicit AttitudeEstimator(const AttitudeEstimatorConfig& config = {});

    AttitudeEstimator(const AttitudeEstimator&) = delete;
    AttitudeEstimator& operator=(const AttitudeEstimator&) = delete;

    /**
     * @brief 校验模式及全部姿态/触发参数，包括当前模式未使用的离架参数。
     * @param config 待校验参数，时间单位 ns，加速度单位 m/s²。
     * @return 模式有效、参数有限、范围及离架阈值/确认超时关系均有效时为 true。
     */
    static bool valid_config(const AttitudeEstimatorConfig& config) noexcept;

    /**
     * @brief 连续静止确认后用 EMA 重力矢量校准姿态，触发后才积分姿态和速度。
     * @param sample 镖体系 IMU 样本；加速度为比力，单位 m/s²，角速度单位 rad/s。
     * @return 同步状态；无效或非递增样本返回上一状态。
     * @note 首次静止确认前姿态无效且禁止触发；两种模式均冻结触发前的非静止姿态。
     * 扰动清除静止计时和 EMA 候选，重新连续静止确认前保持上次锁定姿态。
     * 确认帧保持静止校准姿态和标定初速。
     * 下一连续有效帧起执行陀螺传播和 V += (ax - gravity * sin(pitch)) * dt。
     * 新的无效样本或断流取消待发射证据并中断积分；非负的重复/回退时间戳优先忽略。
     */
    AttitudeEstimate update(const ImuAngularVelocitySample& sample) noexcept;

    /** @brief 清除姿态、零偏和触发状态，速度恢复为标定初值；保留所选模式。 */
    void reset() noexcept;

private:
    enum class LaunchStage { awaiting_acceleration, awaiting_release, triggered };

    struct Quaternion {
        double w = 1.0;
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

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
    /**
     * @brief 从已确认静止段的平滑重力矢量更新滚转和俯仰，不估计航向。
     * @param acceleration EMA 比力或重力矢量，符号由配置决定，单位 m/s²。
     * @return 矢量非退化且角度有限时为 true，并更新内部姿态。
     */
    bool update_prelaunch_attitude(const Vector3& acceleration) noexcept;
    /**
     * @brief 累计连续证据；按所选模式在加速确认或随后离架确认时触发。
     * @param current_ax 当前 X 轴比力，单位 m/s²。
     * @param acceleration_norm 当前比力模长，单位 m/s²。
     * @param timestamp_ns 当前连续有效样本的时间，单位 ns。
     */
    void update_launch_state(double current_ax, double acceleration_norm,
                             TimestampNs timestamp_ns) noexcept;
    /** @brief 清除触发证据与静止候选并要求重校准；不改变已锁姿态和零偏。 */
    void reset_launch_detection() noexcept;

    AttitudeEstimatorConfig config_;
    Quaternion orientation_{};
    Vector3 gyro_bias_{0.0, 0.0, 0.0};
    Vector3 corrected_angular_velocity_{0.0, 0.0, 0.0};
    double speed_mps_ = 0.0;
    double launch_baseline_x_ = 0.0;
    TimestampNs launch_candidate_start_ns_ = -1;
    TimestampNs acceleration_confirmed_ns_ = -1;
    TimestampNs last_timestamp_ns_ = -1;
    TimestampNs stationary_start_ns_ = -1;
    Vector3 filtered_acceleration_{0.0, 0.0, 0.0};
    bool has_launch_baseline_ = false;
    bool has_gyro_bias_ = false;
    // 无效样本打断积分连续性，但 last_timestamp_ns_ 仍保留用于拒绝时间回退。
    bool has_timestamp_ = false;
    LaunchStage launch_stage_ = LaunchStage::awaiting_acceleration;
    TimestampNs launch_timestamp_ns_ = -1;
    AttitudeEstimate last_estimate_{};
};

}  // namespace mosas::runtime

#endif  // MOSAS_RUNTIME_ATTITUDE_ESTIMATOR_HPP
