#include <mosas/adapters/simulation_vision_audit.hpp>
#include <stdexcept>
namespace mosas::adapters {
SimulationVisionAudit::SimulationVisionAudit(std::unique_ptr<ports::IVisionDetector> detector,
    std::shared_ptr<SimulationAudit> audit, std::function<std::optional<runtime::ImuStateSnapshot>()> state)
    : detector_(std::move(detector)),audit_(std::move(audit)),state_(std::move(state)) {
    if(!detector_||!audit_||!state_)throw std::invalid_argument("视觉审计依赖不完整");
}
domain::VisionDetection SimulationVisionAudit::detect(const domain::ImageView& image,
    std::uint64_t frame,const domain::SensorTime& time) {
    auto result=detector_->detect(image,frame,time);
    const auto state=state_();
    if(!audit_->record_vision(frame,time,state?std::optional<std::uint64_t>(state->launch.generation):std::nullopt,
        result.observation.valid,static_cast<std::uint32_t>(result.observation.reason)))
        throw std::runtime_error("simulation vision audit failed");
    return result;
}
void SimulationVisionAudit::reset() noexcept {detector_->reset();}
}
