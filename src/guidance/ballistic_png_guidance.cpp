#include <mosas/guidance/ballistic_png_guidance.hpp>
#include <cmath>
#include <stdexcept>
#include <utility>
namespace mosas::guidance {
BallisticPngGuidance::BallisticPngGuidance(const BallisticPngConfig& config, PngTraceObserver observer)
    : config_(config), observer_(std::move(observer)) {
    if (!valid_config(config)) throw std::invalid_argument("弹道 PNG 配置无效");
}
bool BallisticPngGuidance::valid_config(const BallisticPngConfig& c) noexcept {
    return std::isfinite(c.navigation_constant_y) && c.navigation_constant_y > 0 &&
           std::isfinite(c.navigation_constant_z) && c.navigation_constant_z > 0 &&
           std::isfinite(c.gravity) && c.gravity > 0;
}
domain::GuidanceOutput BallisticPngGuidance::calculate(const domain::GuidanceInput& input) {
    PngCalculationTrace trace;
    if (input.vector_los.valid) {
        trace = evaluate_ballistic_png(input.vector_los.direction, input.vector_los.inertial_direction_rate,
            input.speed_mps.value, config_.navigation_constant_y, config_.navigation_constant_z, config_.gravity);
    } else trace.invalid_reason = domain::GuidanceInvalidReason::invalid_los;
    domain::GuidanceOutput result;
    result.valid = result.command.valid = trace.valid;
    result.invalid_reason = result.command.invalid_reason = trace.invalid_reason;
    result.command.source_frame_id = input.source_frame_id;
    result.command.source_time = input.time;
    result.command.overload_g = result.diagnostics.requested_overload = trace.requested_overload;
    if (observer_) observer_(input, trace);
    return result;
}
}
