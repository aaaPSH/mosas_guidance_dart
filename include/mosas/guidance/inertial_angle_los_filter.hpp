#ifndef MOSAS_GUIDANCE_INERTIAL_ANGLE_LOS_FILTER_HPP
#define MOSAS_GUIDANCE_INERTIAL_ANGLE_LOS_FILTER_HPP

#include <mosas/ports/los_rate_filter.hpp>

#include <cstdint>

namespace mosas::guidance {

/** @brief 惯性系双通道角度 Kalman 配置；噪声参数为标准差而非方差。 */
struct InertialAngleLosFilterConfig {
    double sigma_a = 8.0;                  ///< 公共角加速度过程噪声标准差，rad/s²。
    double sigma_a_pitch = 10.0;           ///< 俯仰过程噪声标准差；0 表示使用 sigma_a。
    double sigma_a_yaw = 3.5;              ///< 方位过程噪声标准差；0 表示使用 sigma_a。
    double sigma_q = 0.0035;              ///< 视线角测量噪声标准差，rad。
    double initial_covariance = 1.0;       ///< 初始角速度方差，(rad/s)²，可为 0。
    bool enable_filter = true;            ///< false 时输出补偿后角度的帧间差分。
    double maximum_gap_seconds = 0.05;    ///< 超过此帧间隔重新建立基准，s。
    bool enable_lever_arm_compensation = true; ///< false 时仅做姿态转换，不进行杆臂补偿。
    double camera_lever_arm_x = 0.0;       ///< 相机相对质心的镖体系 X 轴杆臂，m。
    double estimated_target_distance = 15.0; ///< 连续段首帧弹目距离初值，m；后续速度积分递减，最低 1 m。
};

/**
 * @brief 先完成曝光姿态转换和可选杆臂补偿，再独立滤波惯性系仰角与方位角。
 * @note 两通道状态均为 [角度, 角速度]，不再叠加机体角速度前馈。
 *       下游向量及其切向导数由同一角度状态解析生成；方位极点拒绝输出。
 *       有状态实例仅在同一制导处理线程内调用。
 */
class InertialAngleLosFilter final : public ports::ILosRateFilter {
public:
    /** @param config 角度噪声、时序保护及视差补偿配置；非法时抛出 invalid_argument。 */
    explicit InertialAngleLosFilter(const InertialAngleLosFilterConfig& config = {});

    /** @param config 待检查配置。 @return 噪声方差可表示且其他参数合法时为 true。 */
    static bool valid_config(const InertialAngleLosFilterConfig& config) noexcept;

    /**
     * @brief 更新补偿后的惯性系角度和角速度。
     * @param input 镖体系方向、曝光姿态与速度、帧间隔和代际；方向会防御性单位化。
     * @return 导航系状态及角速率；首帧、极点和非法输入返回无效结果。
     * @note 首帧不传播占位 dt；启用补偿后按相邻曝光速度梯形积分距离，最低 1 m。
     *       断流或代际变化重建零速率及初始距离基准，数值异常清空状态。
     */
    domain::LosRateEstimate update(const domain::LosRateFilterInput& input) override;

    /**
     * @brief 读取最近观测、惯性角速率和实际杆臂补偿距离，不推进时间或积分。
     * @return 同帧诊断；首帧距离可有效而速率无效，关闭补偿或重置后距离无效。
     */
    domain::LineOfSightRateSample sample() const override;

    /** @brief 清除角度、差分和速度积分基准，恢复初始距离并等待新连续段。 */
    void reset() noexcept override;

private:
    /** @brief 单个角度通道的常角速度、白噪声角加速度 Kalman 状态。 */
    struct AngleState {
        double angle = 0.0;
        double rate = 0.0;
        double p00 = 0.0;
        double p01 = 0.0;
        double p11 = 0.0;

        /** @param measurement 首帧角度，rad。 @param config 初始测量及速率方差。 */
        void initialize(double measurement, const InertialAngleLosFilterConfig& config) noexcept;

        /**
         * @brief 以角度观测执行二状态 Kalman 预测和校正。
         * @param measurement 当前角度，rad。
         * @param dt 有限正帧间隔，s。
         * @param sigma_a 本通道角加速度噪声标准差，rad/s²。
         * @param measurement_variance 角度测量方差，rad²。
         * @param wrap 方位通道为 true，创新和状态使用最短圆周差。
         * @return 更新状态有限且协方差正定时为 true。
         */
        bool update(double measurement, double dt, double sigma_a,
                    double measurement_variance, bool wrap) noexcept;
    };

    InertialAngleLosFilterConfig config_;
    AngleState pitch_{};
    AngleState yaw_{};
    bool initialized_ = false;
    std::uint64_t generation_ = 0;
    double previous_pitch_ = 0.0;
    double previous_yaw_ = 0.0;
    double estimated_target_distance_ = 0.0;
    double previous_speed_mps_ = 0.0;
    domain::LineOfSightRateSample sample_{};
};

}  // namespace mosas::guidance

#endif  // MOSAS_GUIDANCE_INERTIAL_ANGLE_LOS_FILTER_HPP
