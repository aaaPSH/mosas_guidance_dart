#include <mosas/output/output_mapping.hpp>
#include <mosas/runtime/attitude_history.hpp>
#include <mosas/vision/vision_visualizer.hpp>

#include "body_vector_test_support.hpp"
#include "test_support.hpp"

#include <array>
#include <cmath>
#include <limits>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace {
/** @return 杆臂补偿距离文字使用两位小数和 m，无效或非有限值显示 N/A 时为 0。 */
int verify_distance_overlay() {
    struct Case {
        bool valid;
        double distance;
        const char* text;
    };
    const std::array<Case, 5> cases{{
        {true, 12.346, "Lever-arm distance: 12.35 m"},
        {true, 1.0, "Lever-arm distance: 1.00 m"},
        {false, 12.346, "Lever-arm distance: N/A"},
        {true, std::numeric_limits<double>::quiet_NaN(), "Lever-arm distance: N/A"},
        {true, std::numeric_limits<double>::infinity(), "Lever-arm distance: N/A"},
    }};
    for (const auto& item : cases) {
        VisionOverlayData data{};
        data.lever_arm_distance_valid = item.valid;
        data.lever_arm_distance_m = item.distance;
        cv::Mat actual(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
        cv::Mat expected = actual.clone();
        VisionVisualizer::draw_guidance_overlay(actual, data);
        cv::putText(expected, item.text, {11, 57}, cv::FONT_HERSHEY_SIMPLEX,
                    0.38, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
        // 第三行只包含补偿距离，独立核对实际文字，避免仅比较同一个绘图实现。
        const cv::Rect row(0, 45, 640, 19);
        CHECK(cv::norm(actual(row), expected(row), cv::NORM_INF) == 0.0);
    }
    return 0;
}
}  // namespace

int main() {
    using namespace mosas;

    domain::VisionDetection detection{};
    detection.observation.valid = true;
    detection.observation.center = {71.5, 42.25};
    detection.diagnostics.bounding_box = {65, 36, 13, 12};
    detection.diagnostics.next_search_region = {40, 20, 80, 60};
    detection.diagnostics.area_pixels = 93;

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
    overlay.lever_arm_distance_valid = true;
    overlay.lever_arm_distance_m = 12.346;

    domain::FlightStateSnapshot state{};
    state.attitude = test::vector_test_attitude(0.25, -0.5, 0.35);
    // 缓存角和陀螺仪值故意不一致，观测必须从同一完整姿态矩阵提取三轴角。
    state.attitude.pitch_rad = {-1.0};
    state.attitude.roll_rad = {1.0};
    state.imu_sample.angular_velocity = {4.0, 5.0, 6.0};
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
    CHECK_NEAR(mapped_overlay.attitude_yaw_rad, 0.35, 1e-12);
    CHECK(mapped_overlay.launched);
    CHECK_NEAR(mapped_overlay.processing_fps, 123.0, 1e-12);
    CHECK(mapped_overlay.lever_arm_distance_valid);
    CHECK_NEAR(mapped_overlay.lever_arm_distance_m, 12.346, 1e-12);
    const auto empty_overlay = output::detail::make_legacy_overlay({}, state);
    CHECK(!empty_overlay.lever_arm_distance_valid);
    CHECK_NEAR(empty_overlay.lever_arm_distance_m, 0.0, 1e-12);

    VisionResult expected_vision{};
    expected_vision.found = true;
    expected_vision.blob = {65, 36, 13, 12, 93, 71.5, 42.25};
    expected_vision.next_roi = {40, 20, 80, 60};
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
    expected_overlay.attitude_yaw_rad = 0.35;
    expected_overlay.launched = true;
    expected_overlay.processing_fps = 123.0;
    expected_overlay.lever_arm_distance_valid = true;
    expected_overlay.lever_arm_distance_m = 12.346;

    cv::Mat mapped_frame(120, 160, CV_8UC3, cv::Scalar(25, 40, 60));
    cv::Mat expected_frame = mapped_frame.clone();
    VisionVisualizer::draw_result(mapped_frame, mapped_vision);
    VisionVisualizer::draw_guidance_overlay(mapped_frame, mapped_overlay);
    VisionVisualizer::draw_result(expected_frame, expected_vision);
    VisionVisualizer::draw_guidance_overlay(expected_frame, expected_overlay);
    CHECK(cv::norm(mapped_frame, expected_frame, cv::NORM_INF) == 0.0);

    // 删除中心红色标记和参考圆后，对应区域保留原像素，ROI/目标框不受影响。
    const cv::Scalar background(25, 40, 60);
    cv::Mat marker_frame(480, 640, CV_8UC3, background);
    VisionResult marker_result{};
    marker_result.found = true;
    marker_result.blob = {220, 180, 41, 41, 1681, 240.0, 200.0};
    marker_result.next_roi = {200, 160, 100, 100};
    VisionVisualizer::draw_result(marker_frame, marker_result);
    for (const auto& region : {cv::Rect(236, 196, 9, 9), cv::Rect(496, 296, 9, 9)}) {
        CHECK(cv::norm(marker_frame(region), cv::Mat(region.size(), CV_8UC3, background),
                       cv::NORM_INF) == 0.0);
    }
    CHECK(marker_frame.at<cv::Vec3b>(160, 200) == cv::Vec3b(0, 0, 255));
    CHECK(marker_frame.at<cv::Vec3b>(180, 220) == cv::Vec3b(255, 0, 0));
    VisionOverlayData roll_overlay{};
    roll_overlay.attitude_valid = true;
    VisionVisualizer::draw_guidance_overlay(marker_frame, roll_overlay);
    CHECK(marker_frame.at<cv::Vec3b>(240, 320) == cv::Vec3b(0, 255, 255));

    cv::Mat yaw_frame(140, 640, CV_8UC3, cv::Scalar(25, 40, 60));
    cv::Mat changed_yaw_frame = yaw_frame.clone();
    auto changed_yaw = mapped_overlay;
    changed_yaw.attitude_yaw_rad = -0.35;
    VisionVisualizer::draw_guidance_overlay(yaw_frame, mapped_overlay);
    VisionVisualizer::draw_guidance_overlay(changed_yaw_frame, changed_yaw);
    CHECK(cv::norm(yaw_frame, changed_yaw_frame, cv::NORM_INF) > 0.0);

    // 黑底上的白色抗锯齿文字必须三通道相等；禁用姿态十字以单独检查左上角文字。
    cv::Mat panel_frame(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
    VisionOverlayData panel_data{};
    VisionVisualizer::draw_guidance_overlay(panel_frame, panel_data);
    std::vector<cv::Mat> panel_channels;
    cv::split(panel_frame, panel_channels);
    CHECK(cv::countNonZero(panel_channels[0]) > 0);
    CHECK(cv::norm(panel_channels[0], panel_channels[1], cv::NORM_INF) == 0.0);
    CHECK(cv::norm(panel_channels[1], panel_channels[2], cv::NORM_INF) == 0.0);

    // 底部目标状态在检出和未检出时均使用白色。
    for (const bool found : {false, true}) {
        cv::Mat status_frame(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
        VisionResult status_result{};
        status_result.found = found;
        VisionVisualizer::draw_result(status_frame, status_result);
        VisionVisualizer::draw_guidance_overlay(status_frame, panel_data);
        std::vector<cv::Mat> status_channels;
        cv::split(status_frame(cv::Rect(0, 450, 640, 30)), status_channels);
        CHECK(cv::countNonZero(status_channels[0]) > 0);
        CHECK(cv::norm(status_channels[0], status_channels[1], cv::NORM_INF) == 0.0);
        CHECK(cv::norm(status_channels[1], status_channels[2], cv::NORM_INF) == 0.0);
    }

    detection.observation.valid = false;
    CHECK(!output::detail::make_legacy_vision_result(detection).found);

    runtime::AttitudeHistory history(2);
    CHECK(history.push({1'000, 0.0, 0.0, 16.0,
                        test::runtime_test_rotation(0.0, 0.0, -0.2)}));
    CHECK(history.push({3'000, 0.0, 0.0, 16.0,
                        test::runtime_test_rotation(0.0, 0.0, 0.6)}));
    const auto exposure = history.find_interpolated(2'000);
    CHECK(exposure && exposure->interpolated);
    const auto& r = exposure->body_to_navigation;
    state.attitude.navigation_from_dart =
        domain::Rotation3<domain::NavigationFrameTag, domain::DartFrameTag>(
            {r.m00, r.m01, r.m02, r.m10, r.m11, r.m12, r.m20, r.m21, r.m22});
    const auto interpolated_overlay = output::detail::make_legacy_overlay(overlay, state);
    CHECK_NEAR(interpolated_overlay.attitude_yaw_rad, 0.2, 1e-12);
    CHECK_NEAR(interpolated_overlay.attitude_pitch_rad, 0.0, 1e-12);
    CHECK_NEAR(interpolated_overlay.attitude_roll_rad, 0.0, 1e-12);

    // 俯仰极点采用 yaw=0 的等价姿态约定，不输出不定的 atan2(0,0)。
    for (const double pitch : {-std::acos(-1.0) / 2.0, std::acos(-1.0) / 2.0}) {
        const auto attitude = test::vector_test_attitude(pitch, 0.4, -0.3);
        const auto angles = output::detail::make_attitude_angles(attitude);
        CHECK(angles.valid);
        CHECK_NEAR(angles.pitch_rad, pitch, 1e-12);
        CHECK_NEAR(angles.yaw_rad, 0.0, 1e-12);
        const auto reconstructed = test::vector_test_attitude(
            angles.pitch_rad, angles.roll_rad, angles.yaw_rad);
        for (std::size_t i = 0; i < 9; ++i)
            CHECK_NEAR(reconstructed.navigation_from_dart.matrix()[i],
                       attitude.navigation_from_dart.matrix()[i], 1e-12);
    }
    state.attitude.valid = false;
    CHECK(!output::detail::make_attitude_angles(state.attitude).valid);
    CHECK(!output::detail::make_legacy_overlay(overlay, state).attitude_valid);
    state.attitude.valid = true;
    auto invalid_matrix = state.attitude.navigation_from_dart.matrix();
    invalid_matrix[0] = std::numeric_limits<double>::quiet_NaN();
    state.attitude.navigation_from_dart =
        domain::Rotation3<domain::NavigationFrameTag, domain::DartFrameTag>(invalid_matrix);
    CHECK(!output::detail::make_attitude_angles(state.attitude).valid);
    CHECK(verify_distance_overlay() == 0);
    return 0;
}
