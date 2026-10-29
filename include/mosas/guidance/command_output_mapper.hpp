#ifndef MOSAS_GUIDANCE_COMMAND_OUTPUT_MAPPER_HPP
#define MOSAS_GUIDANCE_COMMAND_OUTPUT_MAPPER_HPP
#include <mosas/guidance/control_channel.hpp>
#include <mosas/ports/command_output_mapper.hpp>
namespace mosas::guidance {
/** @brief 最终输出的通道和物理限制；重力由装配层统一注入。 */
struct CommandMappingConfig {
    double gravity = 9.80665;
    double maximum_command_overload = 10;
    double maximum_command_acceleration = 100;
    ControlChannel control_channel = ControlChannel::both;
};
/** @brief 唯一的输出安全检查、Roll 映射、body 选轴和双阶段限幅实现。 */
class CommandOutputMapper final : public ports::ICommandOutputMapper {
public:
    /** @param config 输出配置；非法值保留为不可用状态，不静默回退默认值。 */
    explicit CommandOutputMapper(const CommandMappingConfig& config = {}) : config_(config) {}
    /** @return 上限非负、重力正且有限、通道合法时为 true；仅加速度上限允许 +inf。 */
    static bool valid_config(const CommandMappingConfig& config) noexcept;
    bool valid() const noexcept override { return valid_config(config_); }
    /** @param request 弹道系 Y/Z 请求。 @param context 本次发送时刻状态。
     * @return 精确限幅后的 body 命令；保护拒绝时为零，不保存任何历史状态。
     */
    domain::CommandMappingResult map(const domain::BallisticOverloadG& request,
        const domain::CommandOutputContext& context) const noexcept override;
private:
    CommandMappingConfig config_;
};
}
#endif
