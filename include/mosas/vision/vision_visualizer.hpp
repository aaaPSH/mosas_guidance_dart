#ifndef VISION_VISUALIZER_HPP
#define VISION_VISUALIZER_HPP

#include <mosas/vision/vision_types.hpp>

#include <opencv2/core/mat.hpp>

class VisionVisualizer {
public:
    // 在 RGB888 帧上绘制搜索 ROI、目标框和目标中心十字。
    static void draw_result(cv::Mat& frame, const VisionResult& result);
};

#endif  // VISION_VISUALIZER_HPP
