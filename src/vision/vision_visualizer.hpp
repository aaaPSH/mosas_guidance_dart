#ifndef VISION_VISUALIZER_HPP
#define VISION_VISUALIZER_HPP

#include "vision_types.hpp"

class VisionVisualizer {
public:
    // 在 RGB888 帧上绘制搜索 ROI、目标框和目标中心十字。
    static void draw_result(Rgb888Frame& frame, const VisionResult& result);
};

#endif  // VISION_VISUALIZER_HPP
