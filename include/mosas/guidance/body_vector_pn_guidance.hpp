#ifndef MOSAS_BODY_VECTOR_PN_GUIDANCE_HPP
#define MOSAS_BODY_VECTOR_PN_GUIDANCE_HPP

#include <mosas/ports/guidance_law.hpp>

namespace mosas::guidance {

/** @brief 单系数镖体系 PN 参数；限幅作用于抛物线速率补偿后的最终指令比力。 */
struct BodyVectorPnConfig {
    double navigation_constant = 3.0;
    double gravity = 9.80665;
    double maximum_command_overload = 1.0;
    double maximum_command_acceleration = 9.80665;
};

/** @brief 镖体系矢量纯比例导航，小攻角/侧滑假设速度沿镖体 +X。 */
class BodyVectorPnGuidance final : public ports::IGuidanceLaw {
  public:
    /** @brief 保存参数；非法参数在 calculate 时返回无效。 @param config PN 参数。 */
    explicit BodyVectorPnGuidance(const BodyVectorPnConfig &config = {});

    /** @brief 检查参数。 @param config 待校验参数。 @return 参数有效时为 true。 */
    static bool valid_config(const BodyVectorPnConfig &config) noexcept;

    /**
     * @brief 使用矢量惯性 LOS 状态计算镖体系比力过载，并分解为 Y/Z 指令。
     * @param input 同步矢量 LOS、完整姿态和速度；不依赖标量角速率诊断。
     * @return 扣除自然抛物线速率基准并合成限幅后的过载/g；非法输入明确返回无效。
     * @note 基准按同步速度在导航系构造，经完整姿态转到镖体系后扣除；不改原 LOS 状态。
     *       PN 项不再进行 roll 变换或输出端重力叠加；仅诊断量转到弹道系，X 命令为零。
     */
    domain::GuidanceOutput calculate(const domain::GuidanceInput &input) override;

  private:
    BodyVectorPnConfig config_;
};

} // namespace mosas::guidance
#endif
