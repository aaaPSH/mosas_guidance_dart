#ifndef MOSAS_PORTS_LOS_RATE_FILTER_HPP
#define MOSAS_PORTS_LOS_RATE_FILTER_HPP

#include <mosas/domain/algorithm_results.hpp>
#include <mosas/domain/geometry.hpp>

namespace mosas::ports {

/// 在制导处理线程内调用的有状态视线角速度估计端口。
class ILosRateFilter {
public:
    virtual ~ILosRateFilter() = default;

    /**
     * @brief 结合视线测量和镖体角速度更新视线角速度估计。
     * @param input LOS、同步姿态/校正角速度、帧间隔和起飞代际。
     * @return 共同有效性、角速率诊断及可选矢量估计；不提供的表示保持无效。
     * @note 首帧只建立基准；矢量惯性导数仍以镖体系表达，不能当作导航系分量。
     */
    virtual domain::LosRateEstimate update(
        const domain::LosRateFilterInput& input) = 0;

    /**
     * @brief 返回最近一次更新的原始、未滤波和最终视线速率诊断值。
     * @return 最近一次处理对应的速率样本。
     */
    virtual domain::LineOfSightRateSample sample() const = 0;

    /**
     * @brief 清除滤波器状态；时间异常或运行代际切换后应调用。
     */
    virtual void reset() noexcept = 0;
};

}  // namespace mosas::ports

#endif  // MOSAS_PORTS_LOS_RATE_FILTER_HPP
