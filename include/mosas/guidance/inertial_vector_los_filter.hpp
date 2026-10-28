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

/**
 * @brief 在局部导航惯性系 N 下运行的 6 维视线矢量卡尔曼滤波器。
 * 
 * 机制：
 * 1. 利用曝光时刻姿态将镖体系视线投影到局部导航惯性系；
 * 2. 惯性系下各向同性退化为 2x2 标量协方差更新；
 * 3. 显式流形正交投影保持单位模长与切向正交条件；
 * 4. 解析输出惯性视线角速率与镖体系矢量状态。
 */
class InertialVectorLosFilter final : public ports::ILosRateFilter {
public:
    explicit InertialVectorLosFilter(
        const InertialVectorLosFilterConfig& config = {});

    static bool valid_config(const InertialVectorLosFilterConfig& config) noexcept;

    domain::LosRateEstimate update(
        const domain::LosRateFilterInput& input) override;

    domain::LineOfSightRateSample sample() const override;

    void reset() noexcept override;

private:
    void project_to_manifold() noexcept;

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
