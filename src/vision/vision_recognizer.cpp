#include <mosas/vision/vision_recognizer.hpp>

#include <algorithm>
#include <cmath>

#include <opencv2/imgproc.hpp>

namespace {

constexpr VisionConfig kDefaultConfig{
    {0, 0, 320, 240},
    {35, 85, 80, 255, 100, 255},
    40,
    40,
    8,
    0.5,
    0.35,
};

bool is_valid_frame(const cv::Mat& frame) {
    return !frame.empty() && frame.rows > 0 && frame.cols > 0 &&
           frame.type() == CV_8UC3;
}

bool is_valid_roi(const VisionRoi& roi) {
    return roi.width > 0 && roi.height > 0;
}

VisionRoi clip_roi(const VisionRoi& roi, int width, int height) {
    const int left = std::max(0, std::min(roi.x, width));
    const int top = std::max(0, std::min(roi.y, height));
    const int right = std::max(left, std::min(roi.x + roi.width, width));
    const int bottom = std::max(top, std::min(roi.y + roi.height, height));
    return {left, top, right - left, bottom - top};
}

bool is_candidate(const VisionBlob& blob, const VisionConfig& config) {
    if (blob.area < config.min_blob_area || blob.width <= 0 ||
        blob.height <= 0) {
        return false;
    }

    const double width = static_cast<double>(blob.width);
    const double height = static_cast<double>(blob.height);
    const double aspect_ratio = std::min(width / height, height / width);
    const double fill_ratio = static_cast<double>(blob.area) / (width * height);
    return aspect_ratio >= config.min_aspect_ratio &&
           fill_ratio >= config.min_fill_ratio;
}

VisionRoi expanded_roi(const VisionBlob& blob, int range_x, int range_y,
                       int frame_width, int frame_height) {
    return clip_roi({blob.x - range_x, blob.y - range_y,
                     blob.width + 2 * range_x, blob.height + 2 * range_y},
                    frame_width, frame_height);
}

}  // namespace

VisionRecognizer::VisionRecognizer()
    : config_(kDefaultConfig), current_roi_(kDefaultConfig.initial_roi) {}

VisionRecognizer::VisionRecognizer(const VisionConfig& config)
    : config_(config), current_roi_(config.initial_roi) {}

void VisionRecognizer::reset() {
    current_roi_ = config_.initial_roi;
}

VisionResult VisionRecognizer::process(const cv::Mat& frame) {
    VisionResult result{false, {0, 0, 0, 0, 0, 0.0, 0.0}, current_roi_};
    if (!is_valid_frame(frame)) {
        return result;
    }

    const VisionRoi roi = clip_roi(current_roi_, frame.cols, frame.rows);
    if (!is_valid_roi(roi)) {
        current_roi_ = config_.initial_roi;
        result.next_roi = current_roi_;
        return result;
    }

    try {
        const cv::Mat roi_view =
            frame(cv::Rect(roi.x, roi.y, roi.width, roi.height));
        cv::cvtColor(roi_view, hsv_frame_, cv::COLOR_RGB2HSV);
        cv::inRange(
            hsv_frame_,
            cv::Scalar(config_.threshold.h_min, config_.threshold.s_min,
                       config_.threshold.v_min),
            cv::Scalar(config_.threshold.h_max, config_.threshold.s_max,
                       config_.threshold.v_max),
            mask_);

        const int component_count = cv::connectedComponentsWithStats(
            mask_, labels_, stats_, centroids_, 8, CV_32S);
        VisionBlob best_blob{0, 0, 0, 0, 0, 0.0, 0.0};
        for (int label = 1; label < component_count; ++label) {
            const VisionBlob candidate{
                stats_.at<int>(label, cv::CC_STAT_LEFT) + roi.x,
                stats_.at<int>(label, cv::CC_STAT_TOP) + roi.y,
                stats_.at<int>(label, cv::CC_STAT_WIDTH),
                stats_.at<int>(label, cv::CC_STAT_HEIGHT),
                stats_.at<int>(label, cv::CC_STAT_AREA),
                centroids_.at<double>(label, 0) + roi.x,
                centroids_.at<double>(label, 1) + roi.y,
            };
            if (is_candidate(candidate, config_) &&
                candidate.area > best_blob.area) {
                best_blob = candidate;
            }
        }

        if (best_blob.area > 0) {
            result.found = true;
            result.blob = best_blob;
            current_roi_ = expanded_roi(best_blob, config_.found_range_x,
                                         config_.found_range_y, frame.cols,
                                         frame.rows);
        } else {
            current_roi_ = clip_roi(config_.initial_roi, frame.cols, frame.rows);
        }
        result.next_roi = current_roi_;
    } catch (const cv::Exception&) {
        return result;
    }
    return result;
}

VisionRoi VisionRecognizer::current_roi() const {
    return current_roi_;
}
