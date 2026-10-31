#include <mosas/output/output_mapping.hpp>

#include <algorithm>
#include <cmath>

namespace mosas::output::detail {

AttitudeAngles make_attitude_angles(const domain::AttitudeState& attitude) noexcept {
    const auto& m = attitude.navigation_from_dart.matrix();
    if (!attitude.valid ||
        !std::all_of(m.begin(), m.end(), [](double value) { return std::isfinite(value); })) {
        return {};
    }
    const double horizontal = std::hypot(m[0], m[6]);
    AttitudeAngles result;
    result.valid = true;
    result.pitch_rad = std::atan2(m[3], horizontal);
    if (horizontal < 1e-12) {
        // 极点的 roll/yaw 不独立，选 yaw=0 并保留同一旋转的组合滚转角。
        result.roll_rad = std::atan2(m[7], m[8]);
    } else {
        result.roll_rad = std::atan2(-m[5], m[4]);
        result.yaw_rad = std::atan2(m[6], m[0]);
    }
    return result;
}

VisionResult make_legacy_vision_result(
    const domain::VisionDetection& detection) {
    VisionResult result{};
    result.found = detection.observation.valid;
    result.blob.center_x = detection.observation.center.u;
    result.blob.center_y = detection.observation.center.v;
    result.blob.x = detection.diagnostics.bounding_box.x;
    result.blob.y = detection.diagnostics.bounding_box.y;
    result.blob.width = detection.diagnostics.bounding_box.width;
    result.blob.height = detection.diagnostics.bounding_box.height;
    result.blob.area = detection.diagnostics.area_pixels;
    result.next_roi = {detection.diagnostics.next_search_region.x,
                       detection.diagnostics.next_search_region.y,
                       detection.diagnostics.next_search_region.width,
                       detection.diagnostics.next_search_region.height};
    return result;
}

VisionOverlayData make_legacy_overlay(
    const domain::GuidanceOverlay& overlay,
    const domain::FlightStateSnapshot& state) {
    VisionOverlayData result{};
    result.line_of_sight_valid = overlay.line_of_sight_valid;
    result.line_of_sight_q_y_rad = overlay.line_of_sight_q_y.value;
    result.line_of_sight_q_z_rad = overlay.line_of_sight_q_z.value;
    result.line_of_sight_rate_valid = overlay.line_of_sight_rate_valid;
    result.line_of_sight_rate_q_y_rad_s =
        overlay.line_of_sight_rate_q_y.value;
    result.line_of_sight_rate_q_z_rad_s =
        overlay.line_of_sight_rate_q_z.value;
    result.lever_arm_distance_valid = overlay.lever_arm_distance_valid;
    result.lever_arm_distance_m = overlay.lever_arm_distance_m;
    result.body_overload_valid = overlay.body_overload_valid;
    result.body_overload_x_g = overlay.body_overload.x;
    result.body_overload_y_g = overlay.body_overload.y;
    result.body_overload_z_g = overlay.body_overload.z;
    const auto attitude = make_attitude_angles(state.attitude);
    result.attitude_valid = attitude.valid;
    result.attitude_pitch_rad = attitude.pitch_rad;
    result.attitude_roll_rad = attitude.roll_rad;
    result.attitude_yaw_rad = attitude.yaw_rad;
    result.output_attitude_valid = overlay.output_attitude_valid;
    result.output_attitude_pitch_rad = overlay.output_attitude_pitch.value;
    result.output_attitude_roll_rad = overlay.output_attitude_roll.value;
    result.launched = state.launch.phase == domain::LaunchPhase::launched;
    result.processing_fps = overlay.processing_fps;
    return result;
}

}  // namespace mosas::output::detail
