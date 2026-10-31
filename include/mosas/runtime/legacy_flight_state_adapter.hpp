#ifndef MOSAS_RUNTIME_LEGACY_FLIGHT_STATE_ADAPTER_HPP
#define MOSAS_RUNTIME_LEGACY_FLIGHT_STATE_ADAPTER_HPP

#include <mosas/ports/flight_state_estimator.hpp>
#include <mosas/runtime/attitude_estimator.hpp>

namespace mosas::runtime {

/** @brief 将姿态、纵向速度和起飞估计转换为领域快照，单次消费每个 IMU 样本。 */
class LegacyFlightStateEstimatorAdapter final
    : public ports::IFlightStateEstimator {
public:
    /** @brief 创建领域适配器。 @param config 姿态、起飞判据与标定初速参数。 */
    explicit LegacyFlightStateEstimatorAdapter(
        const AttitudeEstimatorConfig& config = {});

    /**
     * @brief 更新并返回同一 IMU 时刻的飞行状态。
     * @param sample 含单调时间、镖体系加速度与角速度的 IMU 样本。
     * @return 同步姿态、纵向速度和起飞状态。
     */
    domain::FlightStateSnapshot update(
        const domain::ImuKinematicSample& sample) override;
    /** @brief 清空估计状态，将速度恢复为标定发射速度。 */
    void reset() noexcept override;

private:
    AttitudeEstimator estimator_;
};

}  // namespace mosas::runtime

#endif  // MOSAS_RUNTIME_LEGACY_FLIGHT_STATE_ADAPTER_HPP
