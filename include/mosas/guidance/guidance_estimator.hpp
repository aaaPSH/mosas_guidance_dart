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

// PNG 律参数。镖体系 Y/Z 方向导航系数无量纲，重力加速度单位为 m/s²；
// maximum_command_overload 是镖体系 Y/Z 合成过载上限（单位 g），
// maximum_command_acceleration 是镖体系指令加速度模长上限（单位 m/s²）。
struct PngGuidanceConfig {
    double navigation_constant_y = 3.0;
    double navigation_constant_z = 3.0;
    double gravity = 9.80665;
    double maximum_command_overload = 1.0;
    double maximum_command_acceleration =
        std::numeric_limits<double>::infinity();
};

// 二维解耦 PNG 律输出。过载先在弹道系计算，再转换到导航系和弹体系；
// 输出保留转换后的加速度，过载为弹体系分量除以重力加速度。
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

    // 第一次有效视线角仅建立差分基准，不产生有效角速度；第二帧开始将
    // body_angular_velocity 对 LOS 的转动项加入观测角速度，再进行滤波。
    LineOfSightAngularVelocity update(const LineOfSight& line_of_sight,
                                       const Vector3& body_angular_velocity,
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
        void initialize(double rate);
        double update(double measurement, double dt);

    private:
        LineOfSightRateFilterConfig config_;
        double rate_ = 0.0;
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

// 使用二维解耦 PNG 律，在恒速度、小攻角假设下直接在镖体系计算过载。
class PngGuidance {
public:
    // 速度方向固定为镖体系 +x；输出的 navigation/body 字段均为镖体系数值，
    // 不进行姿态或速度向量旋转。
    static PngGuidanceOutput calculate(
        const LineOfSight& line_of_sight,
        const LineOfSightAngularVelocity& angular_velocity,
        double speed_mps,
        const PngGuidanceConfig& config = {});
};

#endif  // GUIDANCE_ESTIMATOR_HPP
