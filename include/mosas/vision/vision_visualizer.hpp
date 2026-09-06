#ifndef VISION_VISUALIZER_HPP
#define VISION_VISUALIZER_HPP

#include <mosas/vision/vision_types.hpp>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

class VisionVisualizer {
public:
    // 在 BGR888 帧上绘制搜索 ROI、目标框和目标计算中心的红色十字。
    static void draw_result(cv::Mat& frame, const VisionResult& result);

    // 额外绘制用于计算视线角的相机主点基准十字。
    static void draw_result(cv::Mat& frame, const VisionResult& result,
                            const cv::Point2d& line_of_sight_reference_point);

    // 在画面左上角以透明背景绘制绿色引导关键数据。
    static void draw_guidance_overlay(cv::Mat& frame,
                                      const VisionOverlayData& data);

    // 在显示画面上绘制视觉识别阶段、统计信息和可疑目标框。
    static void draw_vision_debug(cv::Mat& frame, const VisionResult& result);
};

#endif  // VISION_VISUALIZER_HPP
