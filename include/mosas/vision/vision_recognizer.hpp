#ifndef VISION_RECOGNIZER_HPP
#define VISION_RECOGNIZER_HPP

#include <mosas/vision/vision_types.hpp>

#include <opencv2/core/mat.hpp>

class VisionRecognizer {
public:
    VisionRecognizer();
    explicit VisionRecognizer(const VisionConfig& config);

    void reset();
    VisionResult process(const cv::Mat& frame);
    VisionRoi current_roi() const;
    bool is_tracking() const;

private:
    bool scan_roi(const cv::Mat& frame, const VisionRoi& roi,
                  VisionBlob& out_blob);
    bool scan_coarse_pooled(const cv::Mat& frame, VisionRoi& out_coarse_roi);

    VisionConfig config_;
    VisionRoi current_roi_;
    bool is_tracking_{false};

    // 原画质 ROI 扫描缓存
    cv::Mat hsv_frame_;
    cv::Mat mask_;
    cv::Mat labels_;
    cv::Mat stats_;
    cv::Mat centroids_;

    // 1/4 像素池化整图扫描缓存
    cv::Mat pooled_frame_;
    cv::Mat pooled_hsv_;
    cv::Mat pooled_mask_;
    cv::Mat pooled_labels_;
    cv::Mat pooled_stats_;
    cv::Mat pooled_centroids_;
};

#endif  // VISION_RECOGNIZER_HPP
