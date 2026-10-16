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
     * @param input 同步 LOS、姿态中的校正角速度、帧间隔和代际。
     * @return 原算法的标量 LOS 角速度；矢量估计保持无效。
     */
    domain::LosRateEstimate update(
        const domain::LosRateFilterInput& input) override;
    /** @brief 返回最近更新的原始及滤波速率诊断。 @return 最近一次 LOS 速率样本。 */
    domain::LineOfSightRateSample sample() const override;
    /** @brief 清除内部差分和滤波器历史。 */
    void reset() noexcept override;

private:
    LineOfSightRateEstimator estimator_;
};

/// 将默认 PNG 的抛物线 LOS 速率补偿及过载计算适配到领域制导律端口。
class ExistingPngGuidanceLawAdapter final : public ports::IGuidanceLaw {
public:
    /** @brief 使用默认 PNG 并可选观察计算中间量。
     * @param config PNG 参数和限幅配置。 @param observer 默认关闭的只读诊断接收器。
     */
    explicit ExistingPngGuidanceLawAdapter(
        const PngGuidanceConfig& config = {}, PngTraceObserver observer = {});

    /**
     * @brief 将完整姿态和动态速度传入 PNG，先补偿 LOS 速率，再计算命令。
     * @param input 曝光时刻同步的视线、惯性速率、完整姿态、时间和动态速度。
     * @return 领域制导输出；拒绝或无效输入通过有效标志及原因返回。
     */
    domain::GuidanceOutput calculate(
        const domain::GuidanceInput& input) override;

private:
    PngGuidanceConfig config_;
    PngTraceObserver observer_;
};

}  // namespace mosas::guidance

#endif  // MOSAS_GUIDANCE_DEFAULT_ALGORITHM_ADAPTERS_HPP
