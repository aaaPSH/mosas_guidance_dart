#ifndef MOSAS_GUIDANCE_COMMAND_OUTPUT_MAPPER_HPP
#define MOSAS_GUIDANCE_COMMAND_OUTPUT_MAPPER_HPP

#include <mosas/guidance/control_channel.hpp>
#include <mosas/ports/command_output_mapper.hpp>

#include <cmath>
#include <cstdint>
#include <mutex>
#include <optional>

namespace mosas::guidance {

/** @brief 调试模式触发类型。 */
enum class GuidanceDebugMode {
    disabled,         ///< 禁用调试模式（正常闭环制导）
    launch_delay,     ///< 发射确认后延迟固定时间触发
    target_detected,  ///< 首次识别到目标后触发
};

/** @brief 调试模式运行参数。 */
struct GuidanceDebugConfig {
    GuidanceDebugMode mode = GuidanceDebugMode::disabled;
    int trigger_delay_ms = 0;      ///< launch_delay 触发延迟时间，单位 ms
    double yaw_overload_g = 0.0;   ///< 镖体系 yaw 轴固定过载，单位 g
    double pitch_overload_g = 0.0; ///< 镖体系 pitch 轴固定过载，单位 g
    int duration_ms = 0;           ///< 指令持续时间，单位 ms（0 表示持续至结束）
};

/**
 * @brief 校验调试模式配置是否合法。
 * @param config 调试模式配置。
 * @return 模式合法、延迟与持续时间非负、过载值为有限数值时返回 true。
 */
inline bool valid_debug_config(const GuidanceDebugConfig& config) noexcept {
    return (config.mode == GuidanceDebugMode::disabled ||
            config.mode == GuidanceDebugMode::launch_delay ||
            config.mode == GuidanceDebugMode::target_detected) &&
           config.trigger_delay_ms >= 0 && config.duration_ms >= 0 &&
           std::isfinite(config.yaw_overload_g) &&
           std::isfinite(config.pitch_overload_g);
}

/** @brief 最终输出的通道和物理限制；重力由装配层统一注入。 */
struct CommandMappingConfig {
    double gravity = 9.80665;
    double maximum_command_overload = 10;
    double maximum_command_acceleration = 100;
    ControlChannel control_channel = ControlChannel::both;
    GuidanceDebugConfig debug{};
};

/** @brief 唯一的输出安全检查、Roll 映射、body 选轴、双阶段限幅及调试指令输出实现。 */
class CommandOutputMapper final : public ports::ICommandOutputMapper {
public:
    /** @param config 输出配置；非法值保留为不可用状态，不静默回退默认值。 */
    explicit CommandOutputMapper(const CommandMappingConfig& config = {}) : config_(config) {}

    /** @return 上限非负、重力正且有限、通道合法且调试配置有效时为 true；仅加速度上限允许 +inf。 */
    static bool valid_config(const CommandMappingConfig& config) noexcept;

    bool valid() const noexcept override { return valid_config(config_); }
    /** @brief Clear target-trigger timing and cached generation; retains all configured limits. */
    void reset() noexcept override;

    /**
     * @param request 弹道系 Y/Z 请求。
     * @param context 本次发送时刻状态；launch_delay 使用当前代际的发射状态及有效发射时间，不要求视觉请求。
     * @return 精确限幅后的 body 命令；正常模式保护拒绝时为零，调试模式下按触发条件发送固定指令。
     */
    domain::CommandMappingResult map(const domain::BallisticOverloadG& request,
        const domain::CommandOutputContext& context) const noexcept override;

private:
    /** @brief Clear shared session/generation state; debug_mutex_ must already be held. */
    void reset_debug_state_locked() const noexcept;
    CommandMappingConfig config_;
    mutable std::mutex debug_mutex_;
    mutable std::optional<std::uint64_t> last_generation_;
    mutable std::optional<std::int64_t> first_target_detected_time_ns_;
};

}  // namespace mosas::guidance

#endif  // MOSAS_GUIDANCE_COMMAND_OUTPUT_MAPPER_HPP
