#ifndef VISION_RECOGNIZER_HPP
#define VISION_RECOGNIZER_HPP

#include <mosas/vision/vision_types.hpp>

#include <opencv2/core/mat.hpp>

/// 基于 HSV 阈值和连通区域的目标识别器，维护连续帧跟踪 ROI。
class VisionRecognizer {
public:
    /** @brief 使用内置默认阈值和初始 ROI 创建识别器。 */
    VisionRecognizer();

    /**
     * @brief 使用给定的识别阈值、尺寸约束和初始 ROI 创建识别器。
     * @param config 目标颜色、连通区域筛选及跟踪搜索范围配置。
     */
    explicit VisionRecognizer(const VisionConfig& config);

    /**
     * @brief 清除目标跟踪状态，使下一帧从初始搜索区域开始。
     */
    void reset();

    /**
     * @brief 在单帧 BGR 图像中识别目标并更新跟踪区域。
     * @param frame 输入图像；必须是非空、有效的 OpenCV 图像。
     * @return 目标识别结果；未找到目标时由结果状态表示。
     * @note 实例有跨帧状态，应由单一处理线程串行调用。
     */
    VisionResult process(const cv::Mat& frame);

    /**
     * @brief 获取当前跟踪器用于局部搜索的 ROI。
     * @return 当前搜索区域，单位为像素。
     */
    VisionRoi current_roi() const;

    /**
     * @brief 查询跟踪器是否已锁定目标。
     * @return 已建立跟踪状态时为 true。
     */
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
