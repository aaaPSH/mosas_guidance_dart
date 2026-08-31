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
    // launch_speed 为固定的初始发射速度，单位为 m/s。
    explicit DartCondition(double launch_speed = 0.0);

    // 使用发射前的重力方向初始化 roll 和 pitch，yaw 从零开始。
    bool initialize(const Vector3& acceleration);

    // 进入飞行状态，并使用初始姿态推导机体 +x 轴的发射方向。
    // 未初始化时调用无效。
    void launch();

    // 清除当前姿态和生命周期状态。
    void reset();

    // 更新姿态和世界坐标系速度。加速度单位为 m/s²，且包含重力分量。
    EulerAngles update(const Vector3& acceleration,
                       const Vector3& angular_velocity,
                       double dt);

    EulerAngles attitude() const;
    Vector3 velocity() const;
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
    double launch_speed_ = 0.0;
    Vector3 velocity_{0.0, 0.0, 0.0};
    State state_ = State::Uninitialized;
};

#endif  // DART_CONDITION_HPP
