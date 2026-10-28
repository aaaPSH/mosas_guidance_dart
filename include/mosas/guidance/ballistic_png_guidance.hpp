#ifndef MOSAS_GUIDANCE_BALLISTIC_PNG_GUIDANCE_HPP
#define MOSAS_GUIDANCE_BALLISTIC_PNG_GUIDANCE_HPP

#include <mosas/domain/algorithm_results.hpp>
#include <mosas/guidance/control_channel.hpp>
#include <mosas/guidance/png_calculation_trace.hpp>
#include <mosas/ports/guidance_law.hpp>

namespace mosas::guidance {

struct BallisticPngConfig {
    double navigation_constant_y = 3.0;
    double navigation_constant_z = 3.0;
    double gravity = 9.80665;
    double maximum_command_overload = 10.0;         ///< 最大指令过载 (g)
    double maximum_command_acceleration = 100.0;    ///< 最大指令加速度 (m/s²)
    ControlChannel control_channel = ControlChannel::both;
};

/**
 * @brief 弹道系比例导引律、自然重力抛物线补偿与免三角函数 Roll 补偿实现。
 */
class BallisticPngGuidance final : public ports::IGuidanceLaw {
public:
    explicit BallisticPngGuidance(const BallisticPngConfig& config = {},
                                 PngTraceObserver observer = nullptr);

    static bool valid_config(const BallisticPngConfig& config) noexcept;

    domain::GuidanceOutput calculate(
        const domain::GuidanceInput& input) override;

private:
    BallisticPngConfig config_;
    PngTraceObserver observer_;
};

}  // namespace mosas::guidance

#endif  // MOSAS_GUIDANCE_BALLISTIC_PNG_GUIDANCE_HPP
