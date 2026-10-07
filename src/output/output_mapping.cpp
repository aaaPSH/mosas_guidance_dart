#include <mosas/output/output_mapping.hpp>

namespace mosas::output::detail {

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
    result.minimum_blob_area =
        detection.diagnostics.minimum_blob_area_pixels;
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
    result.body_overload_valid = overlay.body_overload_valid;
    result.body_overload_x_g = overlay.body_overload.x;
    result.body_overload_y_g = overlay.body_overload.y;
    result.body_overload_z_g = overlay.body_overload.z;
    result.attitude_valid = state.attitude.valid;
    result.attitude_pitch_rad = state.attitude.pitch_rad.value;
    result.attitude_roll_rad = state.attitude.roll_rad.value;
    result.launched = state.launch.phase == domain::LaunchPhase::launched;
    result.processing_fps = overlay.processing_fps;
    return result;
}

}  // namespace mosas::output::detail
