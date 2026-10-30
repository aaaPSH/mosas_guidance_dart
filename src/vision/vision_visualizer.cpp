#include <mosas/vision/vision_visualizer.hpp>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace {

constexpr double kRadiansToDegrees = 57.29577951308232;
const cv::Scalar kTextColor{255, 255, 255};

bool is_valid_frame(const cv::Mat& frame) {
    return !frame.empty() && frame.rows > 0 && frame.cols > 0 &&
           frame.type() == CV_8UC3;
}

VisionRoi clip_roi(const VisionRoi& roi, int width, int height) {
    const int left = std::max(0, std::min(roi.x, width));
    const int top = std::max(0, std::min(roi.y, height));
    const int right = std::max(left, std::min(roi.x + roi.width, width));
    const int bottom = std::max(top, std::min(roi.y + roi.height, height));
    return {left, top, right - left, bottom - top};
}

void draw_rectangle(cv::Mat& frame, const VisionRoi& raw_roi,
                    const cv::Scalar& color) {
    const VisionRoi roi = clip_roi(raw_roi, frame.cols, frame.rows);
    if (roi.width <= 0 || roi.height <= 0) {
        return;
    }
    cv::rectangle(frame, cv::Rect(roi.x, roi.y, roi.width, roi.height), color,
                  1);
}

void draw_roll_cross(cv::Mat& frame, double roll_rad) {
    if (!std::isfinite(roll_rad)) {
        return;
    }

    const double scale = static_cast<double>(std::min(frame.cols, frame.rows));
    const double horizontal_half_length = scale * 0.16;
    const double vertical_half_length = scale * 0.10;
    const double cosine = std::cos(roll_rad);
    const double sine = std::sin(roll_rad);
    const double center_x = static_cast<double>(frame.cols) / 2.0;
    const double center_y = static_cast<double>(frame.rows) / 2.0;
    const auto rotate_to_frame = [&](double x, double y) {
        return cv::Point(cvRound(center_x + x * cosine - y * sine),
                         cvRound(center_y + x * sine + y * cosine));
    };

    const cv::Point horizontal_start =
        rotate_to_frame(-horizontal_half_length, 0.0);
    const cv::Point horizontal_end =
        rotate_to_frame(horizontal_half_length, 0.0);
    const cv::Point vertical_start =
        rotate_to_frame(0.0, -vertical_half_length);
    const cv::Point vertical_end =
        rotate_to_frame(0.0, vertical_half_length);

    // 深色描边让中心十字在明暗背景上都清晰可见。
    cv::line(frame, horizontal_start, horizontal_end, {0, 0, 0}, 6,
             cv::LINE_AA);
    cv::line(frame, vertical_start, vertical_end, {0, 0, 0}, 6,
             cv::LINE_AA);
    cv::line(frame, horizontal_start, horizontal_end, {0, 255, 255}, 3,
             cv::LINE_AA);
    cv::line(frame, vertical_start, vertical_end, {0, 255, 255}, 3,
             cv::LINE_AA);
}

std::string format_value(bool valid, double value, double scale,
                         const char* unit, int precision = 2) {
    if (!valid || !std::isfinite(value)) {
        return "N/A";
    }

    std::ostringstream stream;
    stream << std::fixed << std::setprecision(precision) << value * scale << unit;
    return stream.str();
}

std::string format_fps(double value) {
    if (!std::isfinite(value) || value < 0.0) {
        return "N/A";
    }

    std::ostringstream stream;
    stream << std::fixed << std::setprecision(2) << value;
    return stream.str();
}

void draw_guidance_overlay_impl(cv::Mat& frame,
                                const VisionOverlayData& data) {
    if (!is_valid_frame(frame)) {
        return;
    }

    const std::vector<std::string> lines{
        "Inertial LOS(deg): qy " +
            format_value(data.line_of_sight_valid,
                         data.line_of_sight_q_y_rad, kRadiansToDegrees, "") +
            " qz " +
            format_value(data.line_of_sight_valid,
                         data.line_of_sight_q_z_rad, kRadiansToDegrees, ""),
        "Inertial LOS rate(deg/s): qy " +
            format_value(data.line_of_sight_rate_valid,
                         data.line_of_sight_rate_q_y_rad_s,
                         kRadiansToDegrees, "") +
            " qz " +
            format_value(data.line_of_sight_rate_valid,
                         data.line_of_sight_rate_q_z_rad_s,
                         kRadiansToDegrees, ""),
        "Body overload(g): x " +
            format_value(data.body_overload_valid, data.body_overload_x_g,
                         1.0, "") +
            " y " +
            format_value(data.body_overload_valid, data.body_overload_y_g,
                         1.0, "") +
            " z " +
            format_value(data.body_overload_valid, data.body_overload_z_g,
                         1.0, ""),
        "Exposure attitude(deg): roll " +
            format_value(data.attitude_valid, data.attitude_roll_rad,
                         kRadiansToDegrees, "") +
            " pitch " +
            format_value(data.attitude_valid, data.attitude_pitch_rad,
                         kRadiansToDegrees, "") +
            " yaw " +
            format_value(data.attitude_valid, data.attitude_yaw_rad,
                         kRadiansToDegrees, "") +
            " phase " +
            (data.attitude_valid ? (data.launched ? "LAUNCHED" : "PRELAUNCH")
                                 : "N/A"),
        "Output attitude(deg): roll " +
            format_value(data.output_attitude_valid, data.output_attitude_roll_rad, kRadiansToDegrees, "") +
            " pitch " + format_value(data.output_attitude_valid, data.output_attitude_pitch_rad, kRadiansToDegrees, ""),
        "Processing FPS: " + format_fps(data.processing_fps),
    };

    constexpr int kPanel_x = 4;
    constexpr int kPanel_y = 4;
    constexpr int kPanel_padding = 7;
    constexpr int kLine_height = 17;
    const int panel_height =
        std::min(kPanel_padding * 2 +
                     static_cast<int>(lines.size()) * kLine_height,
                 frame.rows - kPanel_y - 1);
    if (panel_height <= 0) {
        return;
    }

    try {
        // 仅绘制文字，保持原始画面可见，不使用背景矩形遮挡画面。
        for (std::size_t index = 0; index < lines.size(); ++index) {
            const int baseline = kPanel_y + kPanel_padding + 12 +
                                 static_cast<int>(index) * kLine_height;
            if (baseline >= frame.rows) {
                break;
            }
            cv::putText(frame, lines[index], {kPanel_x + kPanel_padding,
                                              baseline},
                        cv::FONT_HERSHEY_SIMPLEX, 0.38, kTextColor, 1,
                        cv::LINE_AA);
        }
        if (data.attitude_valid) {
            draw_roll_cross(frame, data.attitude_roll_rad);
        }
    } catch (const cv::Exception&) {
        return;
    }
}

}  // namespace

void VisionVisualizer::draw_result(cv::Mat& frame, const VisionResult& result) {
    if (!is_valid_frame(frame)) {
        return;
    }

    try {
        draw_rectangle(frame, result.next_roi, {0, 0, 255});
        const std::string detection_status =
            result.found ? "Target: DETECTED" : "Target: NOT DETECTED";
        cv::putText(frame, detection_status, {6, std::max(18, frame.rows - 8)},
                    cv::FONT_HERSHEY_SIMPLEX, 0.45, kTextColor, 1,
                    cv::LINE_AA);
        if (!result.found) {
            return;
        }

        draw_rectangle(frame,
                       {result.blob.x, result.blob.y, result.blob.width,
                        result.blob.height},
                       {255, 0, 0});
    } catch (const cv::Exception&) {
        return;
    }
}

void VisionVisualizer::draw_guidance_overlay(
    cv::Mat& frame, const VisionOverlayData& data) {
    draw_guidance_overlay_impl(frame, data);
}
