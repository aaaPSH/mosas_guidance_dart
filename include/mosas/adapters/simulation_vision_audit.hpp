#pragma once
#include <mosas/adapters/simulation_audit.hpp>
#include <mosas/ports/vision_detector.hpp>
namespace mosas::adapters {
/** @brief 只观察原检测结果的审计装饰器；绝不生成或提交控制请求。 */
class SimulationVisionAudit final : public ports::IVisionDetector {
public:
    /** @param detector 原检测器。 @param audit 会话审计。 @param state 原运行时状态查询器。 */
    SimulationVisionAudit(std::unique_ptr<ports::IVisionDetector> detector,
        std::shared_ptr<SimulationAudit> audit,
        std::function<std::optional<runtime::ImuStateSnapshot>()> state);
    /** @param image 原图像。 @param frame 来源帧。 @param time 曝光时间。
     * @return 未修改的原检测；审计拒绝时抛异常使运行时停止。
     */
    domain::VisionDetection detect(const domain::ImageView& image, std::uint64_t frame,
                                  const domain::SensorTime& time) override;
    void reset() noexcept override;
private:
    std::unique_ptr<ports::IVisionDetector> detector_;
    std::shared_ptr<SimulationAudit> audit_;
    std::function<std::optional<runtime::ImuStateSnapshot>()> state_;
};
}
