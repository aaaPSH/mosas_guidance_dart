#ifndef MOSAS_BODY_VECTOR_LOS_FILTER_HPP
#define MOSAS_BODY_VECTOR_LOS_FILTER_HPP

#include <mosas/ports/los_rate_filter.hpp>

#include <array>
#include <cstdint>

namespace mosas::guidance {

/** @brief 各向同性矢量滤波参数，不使用独立 pitch/yaw 增益。 */
struct BodyVectorLosFilterConfig {
    double sigma_a = 8.0;    ///< 方向惯性导数的变化噪声标准差，1/s²。
    double sigma_l = 0.0035; ///< 单位 LOS 分量的测量噪声标准差，无量纲。
    double initial_rate_covariance = 1.0; ///< 初始惯性方向变化率方差，(1/s)²。
    double maximum_gap_seconds = 0.2;     ///< 超过此帧间隔后重建基准，s。
};

/** @brief 单线程六状态 LOS Kalman，状态与完整协方差随镖体坐标转动运输。 */
class BodyVectorLosFilter final : public ports::ILosRateFilter {
  public:
    /** @brief 保存滤波参数，非法参数不会被静默替换。 @param config 矢量滤波参数。 */
    explicit BodyVectorLosFilter(const BodyVectorLosFilterConfig &config = {});

    /** @brief 校验参数及方差的数值范围。 @param config 参数。 @return 有效时为 true。 */
    static bool valid_config(const BodyVectorLosFilterConfig &config) noexcept;

    /**
     * @brief 用视觉方向和完整同步姿态估计惯性 LOS 导数（镖体系表达）。
     * @param input 相机帧观测、姿态、真实间隔和代际；首帧不传播占位间隔。
     * @return 矢量估计及角度诊断；首帧、复位及无效输入时共同 valid=false。
     * @note 姿态差运输已实现转动补偿，不能再叠加 omega_body×LOS。
     */
    domain::LosRateEstimate update(const domain::LosRateFilterInput &input) override;

    /** @brief 返回测量角度和原始/滤波惯性角速率诊断。 @return 最近样本。 */
    domain::LineOfSightRateSample sample() const override;

    /** @brief 清除连续段状态、协方差和诊断；下一有效帧只建立基准。 */
    void reset() noexcept override;

  private:
    using Vec3 = std::array<double, 3>;
    using Covariance = std::array<std::array<double, 6>, 6>;

    /**
     * @brief 初始化单位方向及切向协方差。
     * @param measurement 单位方向测量。
     * @param input 当前同步姿态和代际。
     * @return 数值约束成功时为 true。
     */
    bool initialize(const Vec3 &measurement, const domain::LosRateFilterInput &input);
    /**
     * @brief 归一化状态，并用映射 Jacobian 变换完整协方差。
     * @return 状态有限、非退化且协方差数值检查通过时为 true。
     */
    bool constrain_state();

    BodyVectorLosFilterConfig config_;
    Vec3 direction_{};
    Vec3 inertial_rate_{};
    Covariance covariance_{};
    Vec3 previous_measurement_{};
    domain::AttitudeState previous_attitude_{};
    std::uint64_t generation_ = 0;
    bool initialized_ = false;
    domain::LineOfSightRateSample sample_{};
};

} // namespace mosas::guidance
#endif
