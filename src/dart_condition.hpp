#ifndef DART_CONDITION_HPP
#define DART_CONDITION_HPP

struct Vector3 {
    double x;
    double y;
    double z;
};

struct EulerAngles {
    double roll;
    double pitch;
    double yaw;
};

class DartCondition {
public:
    // 使用发射前的重力方向初始化 roll 和 pitch，yaw 从零开始。
    bool initialize(const Vector3& acceleration);

    // 进入飞行状态；未初始化时调用无效。
    void launch();

    // 清除当前姿态和生命周期状态。
    void reset();

    // 更新姿态。飞行阶段保留加速度参数以兼容六轴 IMU 数据流，但不使用它。
    EulerAngles update(const Vector3& acceleration,
                       const Vector3& angular_velocity,
                       double dt);

    EulerAngles attitude() const;
    bool initialized() const;
    bool flying() const;

private:
    enum class State {
        Uninitialized,
        Ready,
        Flying,
    };

    double orientation_w_ = 1.0;
    double orientation_x_ = 0.0;
    double orientation_y_ = 0.0;
    double orientation_z_ = 0.0;
    EulerAngles angles_{0.0, 0.0, 0.0};
    State state_ = State::Uninitialized;
};

#endif  // DART_CONDITION_HPP
