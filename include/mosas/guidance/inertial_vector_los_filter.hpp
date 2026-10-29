#ifndef MOSAS_GUIDANCE_INERTIAL_VECTOR_LOS_FILTER_HPP
#define MOSAS_GUIDANCE_INERTIAL_VECTOR_LOS_FILTER_HPP

#include <mosas/domain/algorithm_results.hpp>
#include <mosas/ports/los_rate_filter.hpp>

#include <array>
#include <cstdint>

namespace mosas::guidance {

struct InertialVectorLosFilterConfig {
    double sigma_a = 5.0;                   ///< DWNA 加速度扰动标准差 (1/s²)
    double sigma_l = 0.0035;                ///< 视线角测量噪声标准差 (无量纲)
    double initial_rate_covariance = 1.0;   ///< 初始角速率方差 ((1/s)²)
    double maximum_gap_seconds = 0.05;      ///< 连续帧最大允许时间间隔 (s)
};

/** @brief 各向同性 LOS 协方差的只读诊断值。 */
struct LosFilterCovariance {
    double p00;
    double p01;
    double p11;
};

/**
 * @brief 在局部导航惯性系 N 下运行的 6 维视线矢量卡尔曼滤波器。
 *
 * 机制：
 * 1. 利用曝光时刻姿态将镖体系视线投影到局部导航惯性系；
 * 2. 惯性系下各向同性退化为 2x2 标量协方差更新；
 * 3. 显式流形正交投影保持单位模长与切向正交条件；
 * 4. 直接输出导航系视线角速率与矢量状态。
 */
class InertialVectorLosFilter final : public ports::ILosRateFilter {
public:
    /** @param config 有限正噪声、初始方差和最大间隔；非法配置抛出 invalid_argument。 */
    explicit InertialVectorLosFilter(
        const InertialVectorLosFilterConfig& config = {});

    /** @return 配置及噪声平方有限、正且可用于严格正定滤波时为 true。 */
    static bool valid_config(const InertialVectorLosFilterConfig& config) noexcept;

    /** @param input 曝光同步观测；首帧仅建基准。 @return 无效输入/数值异常重置并拒绝。 */
    domain::LosRateEstimate update(
        const domain::LosRateFilterInput& input) override;

    /** @return 最近观测及角速度诊断，不推进滤波时间。 */
    domain::LineOfSightRateSample sample() const override;

    /** @return 最近一次滤波的标量协方差快照，不允许修改内部状态。 */
    LosFilterCovariance covariance() const noexcept { return {p00_, p01_, p11_}; }

    /** @brief 清除跟踪段及速率基准，恢复初始协方差。 */
    void reset() noexcept override;

private:
    /** @return 单位化及切向截断成功时为 true，失败状态不能继续输出。 */
    bool project_to_manifold() noexcept;

    InertialVectorLosFilterConfig config_;
    std::array<double, 3> direction_{1.0, 0.0, 0.0}; ///< 惯性系单位视线向量 \hat{l}_N
    std::array<double, 3> rate_{0.0, 0.0, 0.0};      ///< 惯性系视线导数 \hat{\dot{l}}_N
    double p00_ = 0.0;                              ///< 2x2 标量协方差 P_00
    double p01_ = 0.0;                              ///< 2x2 标量协方差 P_01
    double p11_ = 0.0;                              ///< 2x2 标量协方差 P_11
    bool initialized_ = false;
    std::uint64_t generation_ = 0;
    domain::LineOfSightRateSample sample_{};
};

}  // namespace mosas::guidance

#endif  // MOSAS_GUIDANCE_INERTIAL_VECTOR_LOS_FILTER_HPP
