#ifndef MOSAS_PORTS_FLIGHT_STATE_ESTIMATOR_HPP
#define MOSAS_PORTS_FLIGHT_STATE_ESTIMATOR_HPP

#include <mosas/domain/contracts.hpp>

namespace mosas::ports {

/**
 * @brief 将单个 IMU 样本转换为一致的姿态、纵向速度与起飞状态快照。
 * @note 一个样本只调用一次 update，以避免姿态与起飞估计器重复消费样本；实例由单一 IMU 工作线程调用。
 * launched 表示所选发射判据已确认；触发前非静止阶段不积分姿态或速度。
 */
class IFlightStateEstimator {
public:
    virtual ~IFlightStateEstimator() = default;

    /**
     * @brief 更新飞行状态估计。
     * @param sample 带单调时间戳的 IMU 运动样本。
     * @return 与该样本对应的姿态、纵向速度（m/s）、起飞状态及有效性信息。
     */
    virtual domain::FlightStateSnapshot update(
        const domain::ImuKinematicSample& sample) = 0;

    /**
     * @brief 清除历史估计状态，为新一代运行状态重新初始化。
     */
    virtual void reset() noexcept = 0;
};

}  // namespace mosas::ports

#endif  // MOSAS_PORTS_FLIGHT_STATE_ESTIMATOR_HPP
