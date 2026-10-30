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
    : config_(kDefaultConfig),
      current_roi_(kDefaultConfig.initial_roi),
      is_tracking_(false) {}

VisionRecognizer::VisionRecognizer(const VisionConfig& config)
    : config_(config),
      current_roi_(config.initial_roi),
      is_tracking_(false) {}

void VisionRecognizer::reset() {
    current_roi_ = config_.initial_roi;
    is_tracking_ = false;
}

bool VisionRecognizer::is_tracking() const {
    return is_tracking_;
}

bool VisionRecognizer::scan_roi(const cv::Mat& frame, const VisionRoi& roi,
                               VisionBlob& out_blob) {
    const VisionRoi clipped = clip_roi(roi, frame.cols, frame.rows);
    if (!is_valid_roi(clipped)) {
        return false;
    }

    const cv::Mat roi_view =
        frame(cv::Rect(clipped.x, clipped.y, clipped.width, clipped.height));
    // 尺寸不变时 create() 复用已有存储，尺寸变化时才重新分配。
    hsv_frame_.create(roi_view.size(), CV_8UC3);
    mask_.create(roi_view.size(), CV_8UC1);
    cv::cvtColor(roi_view, hsv_frame_, cv::COLOR_BGR2HSV);
    cv::inRange(
        hsv_frame_,
        cv::Scalar(config_.threshold.h_min, config_.threshold.s_min,
                   config_.threshold.v_min),
        cv::Scalar(config_.threshold.h_max, config_.threshold.s_max,
                   config_.threshold.v_max),
        mask_);
    if (cv::countNonZero(mask_) == 0) {
        return false;
    }

    const int label_count = cv::connectedComponentsWithStats(
        mask_, labels_, stats_, centroids_, 8, CV_32S);
    VisionBlob best_blob{0, 0, 0, 0, 0, 0.0, 0.0};
    for (int label = 1; label < label_count; ++label) {
        const VisionBlob candidate{
            stats_.at<int>(label, cv::CC_STAT_LEFT) + clipped.x,
            stats_.at<int>(label, cv::CC_STAT_TOP) + clipped.y,
            stats_.at<int>(label, cv::CC_STAT_WIDTH),
            stats_.at<int>(label, cv::CC_STAT_HEIGHT),
            stats_.at<int>(label, cv::CC_STAT_AREA),
            centroids_.at<double>(label, 0) + clipped.x,
            centroids_.at<double>(label, 1) + clipped.y,
        };
        if (is_candidate(candidate, config_) &&
            candidate.area > best_blob.area) {
            best_blob = candidate;
        }
    }

    if (best_blob.area > 0) {
        out_blob = best_blob;
        return true;
    }
    return false;
}

bool VisionRecognizer::scan_coarse_pooled(const cv::Mat& frame,
                                         VisionRoi& out_coarse_roi) {
    if (frame.cols < 2 || frame.rows < 2) {
        return false;
    }

    // 采用宽高各减半（像素总量为 1/4）进行整图池化粗筛
    const int pooled_width = frame.cols / 2;
    const int pooled_height = frame.rows / 2;
    pooled_frame_.create(cv::Size(pooled_width, pooled_height), frame.type());
    cv::resize(frame, pooled_frame_, cv::Size(pooled_width, pooled_height), 0,
               0, cv::INTER_AREA);

    pooled_hsv_.create(pooled_frame_.size(), CV_8UC3);
    pooled_mask_.create(pooled_frame_.size(), CV_8UC1);
    cv::cvtColor(pooled_frame_, pooled_hsv_, cv::COLOR_BGR2HSV);
    cv::inRange(
        pooled_hsv_,
        cv::Scalar(config_.threshold.h_min, config_.threshold.s_min,
                   config_.threshold.v_min),
        cv::Scalar(config_.threshold.h_max, config_.threshold.s_max,
                   config_.threshold.v_max),
        pooled_mask_);

    if (cv::countNonZero(pooled_mask_) == 0) {
        return false;
    }

    const int label_count = cv::connectedComponentsWithStats(
        pooled_mask_, pooled_labels_, pooled_stats_, pooled_centroids_, 8,
        CV_32S);

    // 粗筛面积阈值：像素总量降为 1/4，因此目标面积理论缩放至 1/4。
    const int coarse_min_area = std::max(1, config_.min_blob_area / 4);
    int best_label = -1;
    int best_area = 0;

    for (int label = 1; label < label_count; ++label) {
        const int area = pooled_stats_.at<int>(label, cv::CC_STAT_AREA);
        const int width = pooled_stats_.at<int>(label, cv::CC_STAT_WIDTH);
        const int height = pooled_stats_.at<int>(label, cv::CC_STAT_HEIGHT);
        if (area < coarse_min_area || width <= 0 || height <= 0) {
            continue;
        }

        const double w_d = static_cast<double>(width);
        const double h_d = static_cast<double>(height);
        const double aspect_ratio = std::min(w_d / h_d, h_d / w_d);
        const double fill_ratio = static_cast<double>(area) / (w_d * h_d);

        // 粗筛阶段放宽离散化带来的几何形变误差（0.7 倍），由原画质 ROI 精检阶段严格校验
        if (aspect_ratio >= config_.min_aspect_ratio * 0.7 &&
            fill_ratio >= config_.min_fill_ratio * 0.7) {
            if (area > best_area) {
                best_area = area;
                best_label = label;
            }
        }
    }

    if (best_label < 0) {
        return false;
    }

    const int c_x = pooled_stats_.at<int>(best_label, cv::CC_STAT_LEFT);
    const int c_y = pooled_stats_.at<int>(best_label, cv::CC_STAT_TOP);
    const int c_w = pooled_stats_.at<int>(best_label, cv::CC_STAT_WIDTH);
    const int c_h = pooled_stats_.at<int>(best_label, cv::CC_STAT_HEIGHT);

    // 映射回原画质全分辨率坐标，并外扩 found_range_x/y（至少外扩 2 像素以吸收奇数像素舍入误差）
    const int full_x = c_x * 2;
    const int full_y = c_y * 2;
    const int full_w = c_w * 2;
    const int full_h = c_h * 2;
    const int margin_x = std::max(config_.found_range_x, 2);
    const int margin_y = std::max(config_.found_range_y, 2);

    out_coarse_roi =
        clip_roi({full_x - margin_x, full_y - margin_y,
                  full_w + 2 * margin_x, full_h + 2 * margin_y},
                 frame.cols, frame.rows);
    return is_valid_roi(out_coarse_roi);
}

VisionResult VisionRecognizer::process(const cv::Mat& frame) {
    VisionResult result{};
    result.next_roi = current_roi_;
    if (!is_valid_frame(frame)) {
        return result;
    }

    try {
        VisionBlob detected_blob{0, 0, 0, 0, 0, 0.0, 0.0};
        bool found = false;

        if (is_tracking_) {
            // 局部跟踪态：直接对上一帧锁定的目标外扩 ROI 进行 1:1 原画质扫描
            const VisionRoi tracking_roi =
                clip_roi(current_roi_, frame.cols, frame.rows);
            if (is_valid_roi(tracking_roi) &&
                scan_roi(frame, tracking_roi, detected_blob)) {
                found = true;
            }
        } else {
            // 全局搜索态：
            // 1. 先用原画幅 1/4 像素量（宽高各 1/2）进行整图池化粗筛
            VisionRoi coarse_roi{};
            if (scan_coarse_pooled(frame, coarse_roi)) {
                // 2. 粗筛命中候选区域后，对该 ROI 区域进行 1:1 原画质精确扫描
                if (scan_roi(frame, coarse_roi, detected_blob)) {
                    found = true;
                }
            }
        }

        if (found) {
            result.found = true;
            result.blob = detected_blob;
            is_tracking_ = true;
            current_roi_ = expanded_roi(detected_blob, config_.found_range_x,
                                         config_.found_range_y, frame.cols,
                                         frame.rows);
        } else {
            result.found = false;
            is_tracking_ = false;
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
