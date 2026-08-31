#ifndef GUIDANCE_ESTIMATOR_HPP
#define GUIDANCE_ESTIMATOR_HPP

#include "vision_types.hpp"

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

class GuidanceEstimator {
public:
    // 根据绿色引导灯形心和相机内参计算镖体系视线仰角与方位角。
    // 相机光轴对应镖体系 +x，图像上方对应 +y，图像右方对应 +z。
    // 目标上仰时 q_y 为正，目标右偏时 q_z 为正，角度单位为弧度。
    static LineOfSight calculate(const VisionResult& result,
                                 const CameraIntrinsics& intrinsics);
};

#endif  // GUIDANCE_ESTIMATOR_HPP
