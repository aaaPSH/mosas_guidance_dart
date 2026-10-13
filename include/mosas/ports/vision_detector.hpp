#ifndef MOSAS_PORTS_VISION_DETECTOR_HPP
#define MOSAS_PORTS_VISION_DETECTOR_HPP

#include <mosas/domain/algorithm_results.hpp>
#include <mosas/domain/image_frame.hpp>

#include <cstdint>

namespace mosas::ports {

/// 单帧视觉识别端口；只产生目标观测，不提交控制命令或访问输出设备。
class IVisionDetector {
public:
    virtual ~IVisionDetector() = default;

    /**
     * @brief 从一帧图像中检测目标并返回观测及视觉诊断信息。
     * @param image 借用的图像视图；调用期间其像素存储必须保持有效。
     * @param source_frame_id 与输入帧关联的主机帧编号。
     * @param time 输入帧的采集时间及其来源/质量信息。
     * @return 检测结果；未找到目标时由结果中的状态和原因表达，不以零坐标表示有效目标。
     */
    virtual domain::VisionDetection detect(
        const domain::ImageView& image, std::uint64_t source_frame_id,
        const domain::SensorTime& time) = 0;

    /**
     * @brief 清除跟踪器的跨帧状态。
     * @note 由调用方在识别器所属的处理线程调用。
     */
    virtual void reset() noexcept = 0;
};

}  // namespace mosas::ports

#endif  // MOSAS_PORTS_VISION_DETECTOR_HPP
