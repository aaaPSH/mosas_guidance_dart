#ifndef MOSAS_GUIDANCE_DEFAULT_ALGORITHM_ADAPTERS_HPP
#define MOSAS_GUIDANCE_DEFAULT_ALGORITHM_ADAPTERS_HPP

#include <mosas/guidance/guidance_estimator.hpp>
#include <mosas/ports/guidance_law.hpp>
#include <mosas/ports/los_estimator.hpp>
#include <mosas/ports/los_rate_filter.hpp>

namespace mosas::guidance {

/// 将现有相机视线计算适配到领域 LOS 端口。
class ExistingLosEstimatorAdapter final : public ports::ILosEstimator {
public:
    /**
     * @brief 调用既有视线几何算法并转换到领域类型。
     * @param observation 当前帧的视觉检测结果。
     * @param intrinsics 相机内参。
     * @param dart_from_camera 相机系到镖体系的固定外参。
     * @return 镖体系视线角及有效性。
     */
    domain::LineOfSight estimate(
        const domain::VisionDetection& observation,
        const domain::CameraIntrinsics& intrinsics,
        const domain::Rotation3<domain::DartFrameTag,
                                domain::CameraFrameTag>& dart_from_camera)
        override;
};

/// 将既有有状态视线角速度估计器适配到领域算法端口。
class ExistingLosRateFilterAdapter final : public ports::ILosRateFilter {
public:
    /** @brief 使用给定滤波配置创建适配器。 @param config LOS 角速率滤波配置。 */
    explicit ExistingLosRateFilterAdapter(
        const LineOfSightRateFilterConfig& config = {});

    /**
     * @brief 更新视线速率估计。
     * @param line_of_sight 当前帧 LOS 角和有效标志。
     * @param body_angular_velocity 同步的镖体系角速度（rad/s）。
     * @param dt_seconds 帧间时间（s）。
     * @return 当前 LOS 角速度（rad/s）及有效标志。
     */
    domain::LineOfSightAngularVelocity update(
        const domain::LineOfSight& line_of_sight,
        const domain::AngularVelocityDartRadPerSec& body_angular_velocity,
        double dt_seconds) override;
    /** @brief 返回最近更新的原始及滤波速率诊断。 @return 最近一次 LOS 速率样本。 */
    domain::LineOfSightRateSample sample() const override;
    /** @brief 清除内部差分和滤波器历史。 */
    void reset() noexcept override;

private:
    LineOfSightRateEstimator estimator_;
};

/// 将既有 PNG 制导算法适配到领域制导律端口。
class ExistingPngGuidanceLawAdapter final : public ports::IGuidanceLaw {
public:
    /** @brief 使用给定参数创建 PNG 适配器。 @param config PNG 参数和限幅配置。 */
    explicit ExistingPngGuidanceLawAdapter(
        const PngGuidanceConfig& config = {});

    /**
     * @brief 根据领域制导输入计算 PNG 命令和诊断量。
     * @param input 当前帧的视线、速率、姿态、时间和速度状态。
     * @return 领域制导输出；拒绝或无效输入通过有效标志及原因返回。
     */
    domain::GuidanceOutput calculate(
        const domain::GuidanceInput& input) override;

private:
    PngGuidanceConfig config_;
};

}  // namespace mosas::guidance

#endif  // MOSAS_GUIDANCE_DEFAULT_ALGORITHM_ADAPTERS_HPP
