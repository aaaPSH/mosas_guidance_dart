#ifndef VISION_VISUALIZER_HPP
#define VISION_VISUALIZER_HPP

#include <mosas/vision/vision_types.hpp>

#include <opencv2/core/mat.hpp>

class VisionVisualizer {
public:
    /**
     * @brief 在 BGR888 帧上绘制搜索 ROI、目标框和白色目标状态文字。
     * @param frame 原地绘制的 BGR 图像；无效图像保持不变。
     * @param result 当前帧的检测结果；不绘制参考圆或目标中心标记。
     */
    static void draw_result(cv::Mat& frame, const VisionResult& result);

    /**
     * @brief 在左上角绘制白色引导数据，并按 roll 在画面中心绘制姿态十字。
     * @param frame 原地绘制的 BGR 图像；无效图像保持不变。
     * @param data 当前帧的引导、姿态和杆臂补偿距离快照；距离单位 m，显示两位小数或 N/A。
     */
    static void draw_guidance_overlay(cv::Mat& frame,
                                      const VisionOverlayData& data);
};

#endif  // VISION_VISUALIZER_HPP
