#ifndef MOSAS_GUIDANCE_CONTROL_CHANNEL_HPP
#define MOSAS_GUIDANCE_CONTROL_CHANNEL_HPP

namespace mosas::guidance {

/** @brief 镖体系控制通道；pitch 对应 body-Y，yaw 对应 body-Z。 */
enum class ControlChannel { both, pitch, yaw };

/**
 * @brief 校验控制通道枚举。
 * @param channel 待校验通道。
 * @return 为 both、pitch 或 yaw 时返回 true。
 */
constexpr bool valid_control_channel(ControlChannel channel) noexcept {
    return channel == ControlChannel::both || channel == ControlChannel::pitch ||
           channel == ControlChannel::yaw;
}

/**
 * @brief 在限幅前清零未启用的镖体系控制分量。
 * @param channel 已校验的控制通道；both 保留两轴。
 * @param pitch 输入/输出的 body-Y 分量，单位由调用方保持一致。
 * @param yaw 输入/输出的 body-Z 分量，单位与 pitch 相同。
 * @note 必须在坐标转换之后、合成限幅之前调用；不改变姿态或 LOS 估计。
 */
inline void select_control_channel(ControlChannel channel, double& pitch,
                                   double& yaw) noexcept {
    if (channel == ControlChannel::pitch) {
        yaw = 0.0;
    } else if (channel == ControlChannel::yaw) {
        pitch = 0.0;
    }
}

}  // namespace mosas::guidance

#endif  // MOSAS_GUIDANCE_CONTROL_CHANNEL_HPP
