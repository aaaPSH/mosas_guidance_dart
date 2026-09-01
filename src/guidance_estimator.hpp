#ifndef GUIDANCE_ESTIMATOR_HPP
#define GUIDANCE_ESTIMATOR_HPP

#include "dart_condition.hpp"
#include "vision_types.hpp"

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

#endif  // GUIDANCE_ESTIMATOR_HPP
