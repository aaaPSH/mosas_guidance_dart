#ifndef GUIDANCE_ESTIMATOR_HPP
#define GUIDANCE_ESTIMATOR_HPP

#include <mosas/runtime/rotation_matrix3.hpp>
#include <mosas/guidance/png_calculation_trace.hpp>
#include <mosas/runtime/vector3.hpp>
#include <mosas/vision/vision_types.hpp>

#include <limits>

struct CameraIntrinsics {
    double fx;
    double fy;
    double cx;
    double cy;
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
    /**
     * @brief 根据图像目标中心和相机内参计算相机坐标视线角。
     * @param result 视觉识别结果及目标中心像素坐标。
     * @param intrinsics 相机焦距和主点，单位为像素。
     * @return 相机轴约定下的俯仰/偏航视线角（rad）；目标或参数无效时 valid=false。
     * @note 相机光轴为 +x，图像上方为 +y、右方为 +z；目标上仰/右偏分别产生正 q_y/q_z。
     */
    static LineOfSight calculate(const VisionResult& result,
                                 const CameraIntrinsics& intrinsics);

    /**
     * @brief 将像素目标视线通过固定安装矩阵转换到镖体系。
     * @param result 视觉识别结果及目标中心像素坐标。
     * @param intrinsics 相机内参，单位为像素。
     * @param camera_to_body 从相机系到镖体系的旋转矩阵。
     * @return 镖体系视线角（rad）；不使用姿态状态，输入无效时 valid=false。
     */
    static LineOfSight calculate_body(
        const VisionResult& result, const CameraIntrinsics& intrinsics,
        const RotationMatrix3& camera_to_body);
};

/**
 * @brief 根据连续视线角估计角速度，并补偿同步的镖体转动。
 * @note 有状态实例应在单一制导处理线程调用；时间间隔单位为秒，角度单位为 rad。
 */
class LineOfSightRateEstimator {
public:
    /**
     * @brief 使用指定过程噪声和测量噪声配置初始化滤波器。
     * @param config Kalman 噪声、初始协方差和滤波开关配置。
     */
    explicit LineOfSightRateEstimator(
        const LineOfSightRateFilterConfig& config = {});

    /**
     * @brief 清除上一段连续处理的差分和滤波状态。
     */
    void reset();

    /**
     * @brief 更新视线角速度估计并记录原始、未滤波及滤波结果。
     * @param line_of_sight 当前视线角；无效值会按滤波器策略处理。
     * @param body_angular_velocity 与当前视线同步的镖体系角速度（rad/s）。
     * @param dt 相邻帧时间间隔（s），须为有限正数。
     * @return 视线角速度（rad/s）；建立首个有效基准时可能暂不可用。
     * @note 丢帧时滤波器可执行预测；时间或数值异常会使滤波器重置。
     */
    LineOfSightAngularVelocity update(const LineOfSight& line_of_sight,
                                       const Vector3& body_angular_velocity,
                                       double dt);

    /**
     * @brief 返回最近一次 update 的视线角速度输出。
     * @return 当前角速度估计及其有效标志。
     */
    LineOfSightAngularVelocity angular_velocity() const;

    /**
     * @brief 获取最近一次更新的原始视线、差分速率和最终滤波速率。
     * @return 视线速率诊断样本。
     */
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

/// 使用二维解耦比例导航律在弹道系计算过载，再转换到镖体系。
class PngGuidance {
public:
    /**
     * @brief 计算 PNG 过载、加速度及制导诊断量。
     * @param line_of_sight 有效目标视线角（rad）。
     * @param angular_velocity 与视线对应的角速度（rad/s）。
     * @param speed_mps 镖体速度大小（m/s）；速度方向按镖体系 +x 处理。
     * @param pitch_rad 相机帧时刻插值得到的俯仰姿态角（rad）。
     * @param roll_rad 相机帧时刻插值得到的滚转角（rad）。
     * @param config PNG 导航系数、重力和输出限幅配置。
     * @param trace 可选只读诊断快照输出；只复制既有中间量，不改变控制计算。
     * @return 制导计算结果；输入无效或数值异常时 valid=false。
     * @note 先在弹道系计算过载，再用 roll 转到镖体系；过载单位为 g，加速度单位为 m/s^2。
     */
    static PngGuidanceOutput calculate(
        const LineOfSight& line_of_sight,
        const LineOfSightAngularVelocity& angular_velocity,
        double speed_mps, double pitch_rad, double roll_rad,
        const PngGuidanceConfig& config = {},
        PngCalculationTrace* trace = nullptr);
};

#endif  // GUIDANCE_ESTIMATOR_HPP
