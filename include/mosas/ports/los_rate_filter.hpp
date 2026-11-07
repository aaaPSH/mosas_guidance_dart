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
     * @param input 镖体系单位 LOS、带曝光时间的同步姿态与速度、LOS 帧间隔和起飞代际。
     * @return 共同有效性、导航系角速率诊断及导航系矢量估计。
     * @note 首帧只建立基准；方向和惯性导数均以导航系表达，不能重复补偿机体转动。
     *       默认实现先补偿向量，再滤波仰角/方位角；q_z 诊断为 cos(q_y)*dq_z/dt。
     *       无效输入中断 LOS 连续段并清空当前诊断，但保留距离积分状态；距离跨段使用实际曝光间隔。
     */
    virtual domain::LosRateEstimate update(
        const domain::LosRateFilterInput& input) = 0;

    /**
     * @brief 读取最近观测、补偿后的惯性角速率及实际补偿距离，不推进时间。
     * @return 最近一次处理对应的诊断快照；距离有效位独立于角速率，关闭补偿或重置后无效。
     */
    virtual domain::LineOfSightRateSample sample() const = 0;

    /**
     * @brief 新会话时清除全部滤波和距离积分状态；发射代际切换由 update 处理。
     */
    virtual void reset() noexcept = 0;
};

}  // namespace mosas::ports

#endif  // MOSAS_PORTS_LOS_RATE_FILTER_HPP
