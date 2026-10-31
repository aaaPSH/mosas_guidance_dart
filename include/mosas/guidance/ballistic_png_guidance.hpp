#ifndef MOSAS_GUIDANCE_BALLISTIC_PNG_GUIDANCE_HPP
#define MOSAS_GUIDANCE_BALLISTIC_PNG_GUIDANCE_HPP
#include <mosas/guidance/png_calculation_trace.hpp>
#include <mosas/ports/guidance_law.hpp>
namespace mosas::guidance {
/** @brief 弹道导引参数；输出通道和限制属于发送端映射配置。 */
struct BallisticPngConfig {
    double navigation_constant_y = 3;
    double navigation_constant_z = 3;
    double gravity = 9.80665;
};
/** @brief 从曝光同步的惯性 LOS 生成原始弹道请求的唯一导引端口。 */
class BallisticPngGuidance final : public ports::IGuidanceLaw {
public:
    /** @param config 正导航常数和共用重力。 @param observer 可选只读计算观察器。
     * @throws std::invalid_argument 配置非法时拒绝构造，不静默回退。
     */
    explicit BallisticPngGuidance(const BallisticPngConfig& config = {}, PngTraceObserver observer = {});
    /** @return 所有常数有限且严格为正时为 true。 */
    static bool valid_config(const BallisticPngConfig& config) noexcept;
    /** @param input 导航系视线、曝光时刻速度及来源。 @return 未限幅弹道请求或无效零请求。 */
    domain::GuidanceOutput calculate(const domain::GuidanceInput& input) override;
private:
    BallisticPngConfig config_;
    PngTraceObserver observer_;
};
}
#endif
