#pragma once

#include <mosas/runtime/runtime_interfaces.hpp>
#include <mosas/wireless/rtsp_streamer.hpp>

#include <memory>
#include <string>

#include <opencv2/core/types.hpp>

namespace mosas::wireless {

struct RtspFrameSinkConfig {
    // 关闭时不建立 RTSP 连接，也不发送任何帧。
    bool enable_wireless_stream = false;
    // 是否在无线输出前绘制识别框和引导数据面板。
    bool draw_visualization = true;
    RtspStreamConfig stream{};
};

class RtspFrameSink final : public mosas::runtime::FrameSink {
public:
    RtspFrameSink(RtspFrameSinkConfig config,
                  cv::Point2d line_of_sight_reference_point);
    ~RtspFrameSink() override;

    RtspFrameSink(const RtspFrameSink&) = delete;
    RtspFrameSink& operator=(const RtspFrameSink&) = delete;

    bool publish(const mosas::runtime::CameraFrame& frame,
                 const VisionResult& vision_result,
                 const mosas::runtime::ImuStateSnapshot& imu_state,
                 const PngGuidanceOutput& guidance) override;

    bool publish(const mosas::runtime::CameraFrame& frame,
                 const VisionResult& vision_result,
                 const mosas::runtime::ImuStateSnapshot& imu_state,
                 const PngGuidanceOutput& guidance,
                 const VisionOverlayData& overlay) override;

    void stop() noexcept;
    bool running() const noexcept;
    std::string last_error() const;

private:
    bool publish_impl(const mosas::runtime::CameraFrame& frame,
                      const VisionResult& vision_result,
                      const VisionOverlayData* overlay);

    RtspFrameSinkConfig config_;
    cv::Point2d line_of_sight_reference_point_;
    std::unique_ptr<RtspStreamer> streamer_;
    std::string last_error_;
};

}  // namespace mosas::wireless
