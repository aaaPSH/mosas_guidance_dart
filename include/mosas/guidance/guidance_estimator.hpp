#ifndef GUIDANCE_ESTIMATOR_HPP
#define GUIDANCE_ESTIMATOR_HPP

#include <mosas/dart/dart_condition.hpp>
#include <mosas/vision/vision_types.hpp>

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

// PNG 律参数。Y/Z 方向导航系数无量纲，重力加速度单位为 m/s²。
struct PngGuidanceConfig {
    double navigation_constant_y = 3.0;
    double navigation_constant_z = 3.0;
    double gravity = 9.80665;
};

// 二维解耦 PNG 律输出。加速度在导航系和弹体系中均保留，过载为弹体系分量除以重力加速度。
struct PngGuidanceOutput {
    bool valid;
    Vector3 navigation_acceleration;
    Vector3 body_acceleration;
    Vector3 body_overload;
    double command_overload;  // 导航系 y-z 平面指令过载 Γc，单位为 g。
    double command_phase;     // 导航系 y-z 平面指令相位 φc，单位为弧度。
};

// 保存一次视线估计的原始角度、未滤波角速度和最终输出，便于记录和对比。
struct LineOfSightRateSample {
    LineOfSight line_of_sight{false, 0.0, 0.0};
    LineOfSightAngularVelocity raw_angular_velocity{false, 0.0, 0.0};
    LineOfSightAngularVelocity filtered_angular_velocity{false, 0.0, 0.0};
};

// Kalman 参数中的噪声均表示方差：角加速度过程噪声单位为
// (rad/s^2)^2，视线角测量噪声单位为 rad^2，初始角速度协方差单位为
// (rad/s)^2。
struct LineOfSightRateFilterConfig {
    double process_noise = 0.01;
    double measurement_noise = 0.25;
    double initial_covariance = 1.0;
    bool enable_filter = true;  // 是否启用 Kalman 滤波。
};

class GuidanceEstimator {
public:
    // 根据绿色引导灯形心和相机内参计算相机/弹体系视线俯仰角与偏航角。
    // 相机光轴对应弹体系 +x，图像上方对应 +y，图像右方对应 +z。
    // 目标上仰时 q_y 为正，目标右偏时 q_z 为正，角度单位为弧度。
    static LineOfSight calculate(const VisionResult& result,
                                 const CameraIntrinsics& intrinsics);

    // 使用飞镖姿态将相机视线补偿到地面坐标系。
    // attitude 的字段顺序为 pitch、yaw、roll，角度单位为弧度，旋转顺序与
    // DartCondition 一致（YZX），表示弹体坐标系到地面坐标系的姿态。
    // 调用方需保证姿态估计和视觉模块的轴定义一致。
    // 默认相机坐标系与弹体坐标系重合。
    static LineOfSight calculate_compensated(
        const VisionResult& result, const CameraIntrinsics& intrinsics,
        const EulerAngles& attitude);

    // camera_to_body 表示相机坐标系到弹体坐标系的固定安装旋转。
    static LineOfSight calculate_compensated(
        const VisionResult& result, const CameraIntrinsics& intrinsics,
        const EulerAngles& attitude, const RotationMatrix3& camera_to_body);
};

// 根据连续视线角估计俯仰/偏航角速度，并使用两个独立的二状态 Kalman
// 滤波器直接处理视线角观测，避免先差分再滤波造成的噪声放大。
class LineOfSightRateEstimator {
public:
    explicit LineOfSightRateEstimator(
        const LineOfSightRateFilterConfig& config = {});

    void reset();

    // 第一次有效视线角仅建立差分基准，不产生有效角速度。
    // 视线无效或 dt 非法时清空时序状态并返回无效结果。
    LineOfSightAngularVelocity update(const LineOfSight& line_of_sight,
                                       double dt);

    LineOfSightAngularVelocity angular_velocity() const;

    // 返回最近一次更新的原始视线角、未滤波角速度和最终角速度。
    LineOfSightRateSample sample() const;

private:
    class ScalarKalmanFilter {
    public:
        explicit ScalarKalmanFilter(
            const LineOfSightRateFilterConfig& config);

        void reset();
        void initialize(double angle, double rate);
        double update(double measurement, double dt);

    private:
        LineOfSightRateFilterConfig config_;
        double angle_ = 0.0;
        double rate_ = 0.0;
        double angle_covariance_ = 0.0;
        double angle_rate_covariance_ = 0.0;
        double rate_covariance_ = 0.0;
    };

    bool has_previous_ = false;
    bool has_rate_ = false;
    bool filter_enabled_ = true;
    double previous_q_y_ = 0.0;
    double previous_q_z_ = 0.0;
    ScalarKalmanFilter q_y_filter_;
    ScalarKalmanFilter q_z_filter_;
    LineOfSightAngularVelocity angular_velocity_{false, 0.0, 0.0};
    LineOfSightRateSample sample_{};
};

// 使用二维解耦 PNG 律，根据视线角速度和飞镖速度计算纵向/横向过载。
class PngGuidance {
public:
    // line_of_sight、angular_velocity 和 dart_velocity 必须在同一导航坐标系中。
    // 输出 body_overload 的 x/y/z 分量为弹体系过载倍数。
    // 速度模长为零时返回无效输出。
    static PngGuidanceOutput calculate(
        const LineOfSight& line_of_sight,
        const LineOfSightAngularVelocity& angular_velocity,
        const Vector3& dart_velocity, const EulerAngles& attitude,
        const PngGuidanceConfig& config = {});
};

#endif  // GUIDANCE_ESTIMATOR_HPP
