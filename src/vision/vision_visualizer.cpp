#include <mosas/vision/vision_visualizer.hpp>

#include <algorithm>

#include <opencv2/imgproc.hpp>

namespace {

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

}  // namespace

void VisionVisualizer::draw_result(cv::Mat& frame, const VisionResult& result) {
    if (!is_valid_frame(frame)) {
        return;
    }

    try {
        // RGB 存储下的颜色顺序与图像通道顺序一致。
        draw_rectangle(frame, result.next_roi, {0, 0, 255});
        if (!result.found) {
            return;
        }

        draw_rectangle(frame,
                       {result.blob.x, result.blob.y, result.blob.width,
                        result.blob.height},
                       {255, 0, 0});
        const cv::Point center(result.blob.x + result.blob.width / 2,
                               result.blob.y + result.blob.height / 2);
        cv::line(frame, {center.x - 2, center.y},
                 {center.x + 2, center.y}, {0, 255, 0}, 1);
        cv::line(frame, {center.x, center.y - 2},
                 {center.x, center.y + 2}, {0, 255, 0}, 1);
    } catch (const cv::Exception&) {
        return;
    }
}
