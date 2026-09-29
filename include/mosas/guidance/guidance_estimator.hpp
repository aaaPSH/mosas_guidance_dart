#ifndef GUIDANCE_ESTIMATOR_HPP
#define GUIDANCE_ESTIMATOR_HPP

#include <mosas/runtime/vector3.hpp>
#include <mosas/vision/vision_types.hpp>

#include <limits>

struct CameraIntrinsics {
    double fx;
    double fy;
    double cx;
    double cy;
};

// 三维旋转矩阵，表示列向量坐标变换：目标向量 = R * 源向量。
struct RotationMatrix3 {
    double m00;
    double m01;
    double m02;
    double m10;
    double m11;
    double m12;
    double m20;
    double m21;
    double m22;
};

struct LineOfSight {
    bool valid;
    double q_y;
    double q_z;
};

struct LineOfSightAngularVelocity {
    bool valid;
    double q_y;
    double q_z;
};

// PNG 律参数。镖体系 Y/Z 方向导航系数独立配置且无量纲，重力加速度单位为
// m/s²；
// maximum_command_overload 是弹道系 Y/Z 合成过载上限（单位 g），
// maximum_command_acceleration 是镖体系指令加速度模长上限（单位 m/s²）。
struct PngGuidanceConfig {
    double navigation_constant_y = 3.0;
    double navigation_constant_z = 3.0;
    double gravity = 9.80665;
    double maximum_command_overload = 1.0;
    double maximum_command_acceleration =
        std::numeric_limits<double>::infinity();
};

// 二维解耦 PNG 律输出。过载先在弹道系计算，再按 roll 转到镖体系；
// navigation_acceleration 保留转换前的弹道系加速度，body 字段为转换结果。
struct PngGuidanceOutput {
    bool valid;
    Vector3 navigation_acceleration;
    Vector3 body_acceleration;
    Vector3 body_overload;
    double command_overload;  // 弹道系 y-z 平面合成指令过载 Γc，单位为 g。
    double command_phase;     // 弹道系 y-z 平面指令相位 φc，单位为弧度。
};

// 保存一次视线估计的原始角度、未滤波角速度和最终输出，便于记录和对比。
struct LineOfSightRateSample {
    LineOfSight line_of_sight{false, 0.0, 0.0};
    LineOfSightAngularVelocity raw_angular_velocity{false, 0.0, 0.0};
    LineOfSightAngularVelocity filtered_angular_velocity{false, 0.0, 0.0};
};

// Kalman 滤波参数：角加速度过程噪声标准差 sigma_a 单位为 rad/s^2，
// 视线角测量噪声标准差 sigma_q 单位为 rad，初始角速度协方差单位为 (rad/s)^2。
// 可选支持俯仰 (pitch) 与偏航 (yaw) 通道过程噪声独立配置，<= 0 时回退使用 sigma_a。
struct LineOfSightRateFilterConfig {
    double sigma_a = 8.0;            // 默认综合角加速度过程噪声标准差 (rad/s^2)
    double sigma_a_pitch = 0.0;      // 俯仰角加速度过程噪声标准差 (<=0 时使用 sigma_a)
    double sigma_a_yaw = 0.0;        // 偏航角加速度过程噪声标准差 (<=0 时使用 sigma_a)
    double sigma_q = 0.0035;         // 视线角测量噪声标准差 (rad)
    double initial_covariance = 1.0; // 初始角速度协方差 ((rad/s)^2)
    bool enable_filter = true;       // 是否启用 Kalman 滤波

    LineOfSightRateFilterConfig() = default;

    LineOfSightRateFilterConfig(double sa, double sq, double init_cov = 1.0,
                                bool enable = true, double sa_pitch = 0.0,
                                double sa_yaw = 0.0)
        : sigma_a(sa),
          sigma_a_pitch(sa_pitch),
          sigma_a_yaw(sa_yaw),
          sigma_q(sq),
          initial_covariance(init_cov),
          enable_filter(enable) {}
};

class GuidanceEstimator {
public:
    // 根据绿色引导灯形心和相机内参计算相机视线俯仰角与偏航角。
    // 相机光轴对应弹体系 +x，图像上方对应 +y，图像右方对应 +z。
    // 目标上仰时 q_y 为正，目标右偏时 q_z 为正，角度单位为弧度。
    static LineOfSight calculate(const VisionResult& result,
                                 const CameraIntrinsics& intrinsics);

    // 只使用固定安装矩阵将相机视线转换到镖体系，不使用姿态状态。
    static LineOfSight calculate_body(
        const VisionResult& result, const CameraIntrinsics& intrinsics,
        const RotationMatrix3& camera_to_body);
};

// 根据连续视线角估计角速度，并用同步的镖体角速度补偿弹体转动影响。
class LineOfSightRateEstimator {
public:
    explicit LineOfSightRateEstimator(
        const LineOfSightRateFilterConfig& config = {});

    void reset();

    // 第一次有效视线角仅建立差分/滤波基准，不产生有效角速度；
    // 启用二状态滤波时，直接将相机角度作为测量更新，IMU 旋转补偿作为前馈已知输入；
    // 丢帧时执行纯预测外推，保持角速度连续输出。
    LineOfSightAngularVelocity update(const LineOfSight& line_of_sight,
                                       const Vector3& body_angular_velocity,
                                       double dt);

    LineOfSightAngularVelocity angular_velocity() const;

    // 返回最近一次更新的原始视线角、未滤波角速度和最终角速度。
    LineOfSightRateSample sample() const;

private:
    class TwoStateLosKalmanFilter {
    public:
        TwoStateLosKalmanFilter(double sigma_a, double sigma_q,
                                double initial_covariance);

        void reset();
        void initialize(double q_b, double initial_rate = 0.0);
        double update(bool valid, double z_qb, double c_omega, double dt);
        bool is_initialized() const { return initialized_; }
        double estimated_angle() const { return q_b_; }
        double estimated_rate() const { return q_dot_i_; }

    private:
        double sigma_a_ = 8.0;
        double r_ = 0.0035 * 0.0035;
        double initial_covariance_ = 1.0;
        bool initialized_ = false;
        double q_b_ = 0.0;
        double q_dot_i_ = 0.0;
        double p00_ = 0.0;
        double p01_ = 0.0;
        double p10_ = 0.0;
        double p11_ = 1.0;
    };

    bool has_previous_ = false;
    bool filter_enabled_ = true;
    double previous_q_y_ = 0.0;
    double previous_q_z_ = 0.0;
    TwoStateLosKalmanFilter q_y_filter_;
    TwoStateLosKalmanFilter q_z_filter_;
    LineOfSightAngularVelocity angular_velocity_{false, 0.0, 0.0};
    LineOfSightRateSample sample_{};
};

// 使用二维解耦 PNG 律在弹道系计算过载，再按维护的 roll 角转到镖体系。
class PngGuidance {
public:
    // 速度方向固定为镖体系 +x；角度使用相机时刻插值得到的姿态估计。
    static PngGuidanceOutput calculate(
        const LineOfSight& line_of_sight,
        const LineOfSightAngularVelocity& angular_velocity,
        double speed_mps, double pitch_rad, double roll_rad,
        const PngGuidanceConfig& config = {});
};

#endif  // GUIDANCE_ESTIMATOR_HPP
