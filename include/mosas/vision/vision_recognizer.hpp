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

private:
    VisionConfig config_;
    VisionRoi current_roi_;
    cv::Mat hsv_frame_;
    cv::Mat mask_;
    cv::Mat labels_;
    cv::Mat stats_;
    cv::Mat centroids_;
};

#endif  // VISION_RECOGNIZER_HPP
