#include <mosas/output/output_mapping.hpp>
#include <mosas/vision/vision_visualizer.hpp>

#include "test_support.hpp"

#include <opencv2/core.hpp>

int main() {
    using namespace mosas;

    domain::VisionDetection detection{};
    detection.observation.valid = true;
    detection.observation.center = {71.5, 42.25};
    detection.diagnostics.bounding_box = {65, 36, 13, 12};
    detection.diagnostics.next_search_region = {40, 20, 80, 60};
    detection.diagnostics.area_pixels = 93;
    detection.diagnostics.minimum_blob_area_pixels = 17;

    const VisionResult mapped_vision =
        output::detail::make_legacy_vision_result(detection);
    CHECK(mapped_vision.found);
    CHECK(mapped_vision.blob.x == 65);
    CHECK(mapped_vision.blob.y == 36);
    CHECK(mapped_vision.blob.width == 13);
    CHECK(mapped_vision.blob.height == 12);
    CHECK(mapped_vision.blob.area == 93);
    CHECK_NEAR(mapped_vision.blob.center_x, 71.5, 1e-12);
    CHECK_NEAR(mapped_vision.blob.center_y, 42.25, 1e-12);
    CHECK(mapped_vision.next_roi.x == 40);
    CHECK(mapped_vision.next_roi.y == 20);
    CHECK(mapped_vision.next_roi.width == 80);
    CHECK(mapped_vision.next_roi.height == 60);
    CHECK(mapped_vision.minimum_blob_area == 17);

    domain::GuidanceOverlay overlay{};
    overlay.line_of_sight_valid = true;
    overlay.line_of_sight_q_y = {0.1};
    overlay.line_of_sight_q_z = {-0.2};
    overlay.line_of_sight_rate_valid = true;
    overlay.line_of_sight_rate_q_y = {0.3};
    overlay.line_of_sight_rate_q_z = {-0.4};
    overlay.body_overload_valid = true;
    overlay.body_overload = {1.0, 2.0, 3.0};
    overlay.attitude_valid = true;
    overlay.attitude_pitch = {0.25};
    overlay.attitude_roll = {-0.5};
    overlay.launch_phase = domain::LaunchPhase::launched;
    overlay.processing_fps = 123.0;

    domain::FlightStateSnapshot state{};
    state.attitude.valid = true;
    state.attitude.pitch_rad = {0.25};
    state.attitude.roll_rad = {-0.5};
    state.launch.phase = domain::LaunchPhase::launched;
    const VisionOverlayData mapped_overlay =
        output::detail::make_legacy_overlay(overlay, state);
    CHECK(mapped_overlay.line_of_sight_valid);
    CHECK_NEAR(mapped_overlay.line_of_sight_q_y_rad, 0.1, 1e-12);
    CHECK_NEAR(mapped_overlay.line_of_sight_q_z_rad, -0.2, 1e-12);
    CHECK(mapped_overlay.line_of_sight_rate_valid);
    CHECK_NEAR(mapped_overlay.line_of_sight_rate_q_y_rad_s, 0.3, 1e-12);
    CHECK_NEAR(mapped_overlay.line_of_sight_rate_q_z_rad_s, -0.4, 1e-12);
    CHECK(mapped_overlay.body_overload_valid);
    CHECK_NEAR(mapped_overlay.body_overload_x_g, 1.0, 1e-12);
    CHECK_NEAR(mapped_overlay.body_overload_y_g, 2.0, 1e-12);
    CHECK_NEAR(mapped_overlay.body_overload_z_g, 3.0, 1e-12);
    CHECK(mapped_overlay.attitude_valid);
    CHECK_NEAR(mapped_overlay.attitude_pitch_rad, 0.25, 1e-12);
    CHECK_NEAR(mapped_overlay.attitude_roll_rad, -0.5, 1e-12);
    CHECK(mapped_overlay.launched);
    CHECK_NEAR(mapped_overlay.processing_fps, 123.0, 1e-12);

    VisionResult expected_vision{};
    expected_vision.found = true;
    expected_vision.blob = {65, 36, 13, 12, 93, 71.5, 42.25};
    expected_vision.next_roi = {40, 20, 80, 60};
    expected_vision.minimum_blob_area = 17;
    VisionOverlayData expected_overlay{};
    expected_overlay.line_of_sight_valid = true;
    expected_overlay.line_of_sight_q_y_rad = 0.1;
    expected_overlay.line_of_sight_q_z_rad = -0.2;
    expected_overlay.line_of_sight_rate_valid = true;
    expected_overlay.line_of_sight_rate_q_y_rad_s = 0.3;
    expected_overlay.line_of_sight_rate_q_z_rad_s = -0.4;
    expected_overlay.body_overload_valid = true;
    expected_overlay.body_overload_x_g = 1.0;
    expected_overlay.body_overload_y_g = 2.0;
    expected_overlay.body_overload_z_g = 3.0;
    expected_overlay.attitude_valid = true;
    expected_overlay.attitude_pitch_rad = 0.25;
    expected_overlay.attitude_roll_rad = -0.5;
    expected_overlay.launched = true;
    expected_overlay.processing_fps = 123.0;

    cv::Mat mapped_frame(120, 160, CV_8UC3, cv::Scalar(25, 40, 60));
    cv::Mat expected_frame = mapped_frame.clone();
    VisionVisualizer::draw_result(mapped_frame, mapped_vision,
                                  cv::Point2d{80.0, 60.0});
    VisionVisualizer::draw_guidance_overlay(mapped_frame, mapped_overlay);
    VisionVisualizer::draw_result(expected_frame, expected_vision,
                                  cv::Point2d{80.0, 60.0});
    VisionVisualizer::draw_guidance_overlay(expected_frame, expected_overlay);
    CHECK(cv::norm(mapped_frame, expected_frame, cv::NORM_INF) == 0.0);

    detection.observation.valid = false;
    CHECK(!output::detail::make_legacy_vision_result(detection).found);
    return 0;
}
