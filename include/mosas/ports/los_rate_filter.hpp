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
     * @param line_of_sight 当前帧的视线角；无效测量由 valid 标志表达。
     * @param body_angular_velocity 同步到当前帧时刻的镖体系角速度（rad/s）。
     * @param dt_seconds 相邻处理时刻间隔（s），必须为有效正值。
     * @return 视线角速度（rad/s）；不可用时通过结果的 valid 状态表示。
     */
    virtual domain::LineOfSightAngularVelocity update(
        const domain::LineOfSight& line_of_sight,
        const domain::AngularVelocityDartRadPerSec& body_angular_velocity,
        double dt_seconds) = 0;

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
