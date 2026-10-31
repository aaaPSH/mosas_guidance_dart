#ifndef MOSAS_RUNTIME_ROTATION_MATRIX3_HPP
#define MOSAS_RUNTIME_ROTATION_MATRIX3_HPP

// 行优先三维旋转矩阵，列向量变换语义为目标向量 = R * 源向量。
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

#endif  // MOSAS_RUNTIME_ROTATION_MATRIX3_HPP
