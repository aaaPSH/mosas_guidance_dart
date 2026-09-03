#pragma once

#include <mosas/runtime/runtime_interfaces.hpp>
#include <mosas/wireless/rtsp_streamer.hpp>

#include <atomic>
#include <memory>
#include <mutex>
#include <string>

#include <opencv2/core/types.hpp>
#include <opencv2/videoio.hpp>

namespace mosas::wireless {

struct RtspFrameSinkConfig {
    // 关闭时不建立 RTSP 连接，也不发送任何帧。
    bool enable_wireless_stream = false;
    // 是否启用本地录像；关闭时不创建 VideoWriter。
    bool enable_recording = false;
    // 录像文件路径；启用录像时不能为空。
    std::string recording_path;
    double recording_fps = 30.0;
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

    void cancel() noexcept override;
    void reset() noexcept override;
    void stop() noexcept override;
    bool running() const noexcept;
    std::string last_error() const;

private:
    bool publish_impl(const mosas::runtime::CameraFrame& frame,
                      const VisionResult& vision_result,
                      const VisionOverlayData* overlay);

    RtspFrameSinkConfig config_;
    cv::Point2d line_of_sight_reference_point_;
    // 仅保护 RTSP 对象的创建、取消和复位，不包住网络发送，避免取消被阻塞。
    mutable std::mutex streamer_mutex_;
    std::unique_ptr<RtspStreamer> streamer_;
    // 输出线程复用这些缓冲区，避免每帧重复申请图像头和像素内存。
    cv::Mat annotated_rgb_frame_;
    cv::Mat resized_rgb_frame_;
    cv::Mat bgr_frame_;
    cv::VideoWriter recording_writer_;
    std::atomic<bool> cancel_requested_{false};
    std::string last_error_;
};

}  // namespace mosas::wireless
