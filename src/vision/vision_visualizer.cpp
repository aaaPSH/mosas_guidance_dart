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
constexpr double kPi = 3.14159265358979323846;
const cv::Scalar kOverlayColor{0, 255, 0};
const cv::Scalar kDebugColor{0, 255, 255};

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

void draw_cross(cv::Mat& frame, const cv::Point2d& raw_center,
                const cv::Scalar& color) {
    if (!std::isfinite(raw_center.x) || !std::isfinite(raw_center.y)) {
        return;
    }

    const cv::Point center(cvRound(raw_center.x), cvRound(raw_center.y));
    cv::line(frame, {center.x - 2, center.y}, {center.x + 2, center.y}, color,
             1);
    cv::line(frame, {center.x, center.y - 2}, {center.x, center.y + 2}, color,
             1);
}

void draw_reference_circle(cv::Mat& frame, const cv::Point2d& raw_center,
                           int minimum_blob_area,
                           const cv::Scalar& color) {
    if (!std::isfinite(raw_center.x) || !std::isfinite(raw_center.y) ||
        minimum_blob_area <= 0) {
        return;
    }

    const double radius = std::sqrt(
        static_cast<double>(minimum_blob_area) / kPi);
    const int radius_pixels = std::max(1, cvRound(radius));
    cv::circle(frame,
               {cvRound(raw_center.x), cvRound(raw_center.y)},
               radius_pixels, color, 1, cv::LINE_8);
}

void draw_result_impl(cv::Mat& frame, const VisionResult& result,
                      const cv::Point2d* line_of_sight_reference_point) {
    if (!is_valid_frame(frame)) {
        return;
    }

    try {
        // 使用 OpenCV BGR 存储约定：基准圆为绿色，目标框为蓝色，目标中心十字为红色。
        draw_rectangle(frame, result.next_roi, {0, 0, 255});
        if (line_of_sight_reference_point != nullptr) {
            draw_reference_circle(frame, *line_of_sight_reference_point,
                                  result.minimum_blob_area, {0, 255, 0});
        }
        if (!result.found) {
            return;
        }

        draw_rectangle(frame,
                       {result.blob.x, result.blob.y, result.blob.width,
                        result.blob.height},
                       {255, 0, 0});
        draw_cross(frame,
                   {result.blob.center_x, result.blob.center_y},
                   {0, 0, 255});
    } catch (const cv::Exception&) {
        return;
    }
}

std::string format_value(bool valid, double value, double scale,
                         const char* unit) {
    if (!valid || !std::isfinite(value)) {
        return "N/A";
    }

    std::ostringstream stream;
    stream << std::fixed << std::setprecision(1) << value * scale << unit;
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

    const double speed = std::hypot(
        data.velocity_x_mps,
        std::hypot(data.velocity_y_mps, data.velocity_z_mps));
    const std::vector<std::string> lines{
        "LOS(deg): qy " +
            format_value(data.line_of_sight_valid,
                         data.line_of_sight_q_y_rad, kRadiansToDegrees, "") +
            " qz " +
            format_value(data.line_of_sight_valid,
                         data.line_of_sight_q_z_rad, kRadiansToDegrees, ""),
        "LOS rate(deg/s): qy " +
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
        "Attitude(deg): P " +
            format_value(data.attitude_valid, data.attitude_pitch_rad,
                         kRadiansToDegrees, "") +
            " Y " +
            format_value(data.attitude_valid, data.attitude_yaw_rad,
                         kRadiansToDegrees, "") +
            " R " +
            format_value(data.attitude_valid, data.attitude_roll_rad,
                         kRadiansToDegrees, ""),
        "Velocity(m/s): x " +
            format_value(data.velocity_valid, data.velocity_x_mps, 1.0, "") +
            " y " +
            format_value(data.velocity_valid, data.velocity_y_mps, 1.0, "") +
            " z " +
            format_value(data.velocity_valid, data.velocity_z_mps, 1.0, ""),
        "Speed(m/s): " +
            format_value(data.velocity_valid, speed, 1.0, ""),
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
                        cv::FONT_HERSHEY_SIMPLEX, 0.38, kOverlayColor, 1,
                        cv::LINE_AA);
        }
    } catch (const cv::Exception&) {
        return;
    }
}

const char* debug_stage_name(VisionDebugStage stage) {
    switch (stage) {
        case VisionDebugStage::not_processed:
            return "NOT_PROCESSED";
        case VisionDebugStage::invalid_frame:
            return "INVALID_FRAME";
        case VisionDebugStage::invalid_roi:
            return "INVALID_ROI";
        case VisionDebugStage::mask_empty:
            return "MASK_EMPTY";
        case VisionDebugStage::candidate_rejected:
            return "CANDIDATE_REJECTED";
        case VisionDebugStage::found:
            return "FOUND";
        case VisionDebugStage::processing_error:
            return "PROCESSING_ERROR";
    }
    return "UNKNOWN";
}

void draw_vision_debug_impl(cv::Mat& frame, const VisionResult& result) {
    if (!is_valid_frame(frame)) {
        return;
    }

    try {
        std::ostringstream statistics;
        statistics << "Vision: " << debug_stage_name(result.debug_stage)
                   << " mask=" << result.mask_pixel_count
                   << " components=" << result.component_count
                   << " candidates=" << result.candidate_count;
        constexpr int kTextX = 6;
        const int baseline = std::max(18, frame.rows - 8);
        cv::putText(frame, statistics.str(), {kTextX, baseline},
                    cv::FONT_HERSHEY_SIMPLEX, 0.45, kDebugColor, 1,
                    cv::LINE_AA);

        if (!result.found && result.debug_blob_valid) {
            const VisionRoi suspicious_blob{
                result.debug_blob.x, result.debug_blob.y,
                result.debug_blob.width, result.debug_blob.height};
            draw_rectangle(frame, suspicious_blob, kDebugColor);
            std::ostringstream blob_text;
            blob_text << "suspicious area=" << result.debug_blob.area;
            const int text_y = std::max(
                16, std::min(frame.rows - 8,
                             result.debug_blob.y - 4));
            cv::putText(frame, blob_text.str(),
                        {std::max(2, result.debug_blob.x), text_y},
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, kDebugColor, 1,
                        cv::LINE_AA);
        }
    } catch (const cv::Exception&) {
        return;
    }
}

}  // namespace

void VisionVisualizer::draw_result(cv::Mat& frame, const VisionResult& result) {
    draw_result_impl(frame, result, nullptr);
}

void VisionVisualizer::draw_result(
    cv::Mat& frame, const VisionResult& result,
    const cv::Point2d& line_of_sight_reference_point) {
    draw_result_impl(frame, result, &line_of_sight_reference_point);
}

void VisionVisualizer::draw_guidance_overlay(
    cv::Mat& frame, const VisionOverlayData& data) {
    draw_guidance_overlay_impl(frame, data);
}

void VisionVisualizer::draw_vision_debug(cv::Mat& frame,
                                         const VisionResult& result) {
    draw_vision_debug_impl(frame, result);
}
