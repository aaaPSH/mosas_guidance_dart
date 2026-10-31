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
     * @brief 使用镖体系视线向量和曝光姿态更新导航系惯性视线估计。
     * @param input 镖体系单位 LOS、曝光同步姿态与速度、帧间隔和起飞代际。
     * @return 共同有效性、导航系角速率诊断及导航系矢量估计。
     * @note 首帧只建立基准；方向和惯性导数均以导航系表达，不能重复补偿机体转动。
     *       默认实现先补偿向量，再滤波仰角/方位角；q_z 诊断为 cos(q_y)*dq_z/dt。
     */
    virtual domain::LosRateEstimate update(
        const domain::LosRateFilterInput& input) = 0;

    /**
     * @brief 读取最近观测、补偿后的惯性角速率及实际补偿距离，不推进时间。
     * @return 最近一次处理对应的诊断快照；距离有效位独立于角速率，关闭补偿或重置后无效。
     */
    virtual domain::LineOfSightRateSample sample() const = 0;

    /**
     * @brief 清除滤波器状态；时间异常或运行代际切换后应调用。
     */
    virtual void reset() noexcept = 0;
};

}  // namespace mosas::ports

#endif  // MOSAS_PORTS_LOS_RATE_FILTER_HPP
