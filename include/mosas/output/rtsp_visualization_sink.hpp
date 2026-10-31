#ifndef MOSAS_OUTPUT_RTSP_VISUALIZATION_SINK_HPP
#define MOSAS_OUTPUT_RTSP_VISUALIZATION_SINK_HPP

#include <mosas/output/output_config.hpp>
#include <mosas/runtime/output_ports.hpp>
#include <mosas/wireless/rtsp_streamer.hpp>

#include <atomic>
#include <memory>
#include <mutex>
#include <string>

#include <opencv2/core.hpp>

namespace mosas::output {

/// RTSP 可视化 sink；渲染和网络 I/O 由 dispatcher 的独立可视化 worker 执行。
class RtspVisualizationSink final : public runtime::IVisualizationSink {
public:
    /**
     * @brief 创建 RTSP 可视化 sink；此时不启动网络流。
     * @param output_config 叠加绘制等输出配置。
     * @param stream_config RTSP 地址、编码、尺寸和帧率配置。
     */
    RtspVisualizationSink(OutputConfig output_config,
                          wireless::RtspStreamConfig stream_config);
    ~RtspVisualizationSink() override;

    bool start() override;
    void reset() noexcept override;
    /**
     * @brief 渲染并发送一帧 RTSP 图像。
     * @param frame 含 BGR8 图像、视觉结果和制导叠加数据的快照。
     * @return 成功发送或因取消而忽略时为 true；渲染、建流或发送失败时为 false。
     * @note streamer 在首帧发布时懒启动；本函数可能执行渲染、编码和网络 I/O。
     */
    bool publish(const runtime::ProcessedFrame& frame) override;
    void cancel() noexcept override;
    void stop() noexcept override;
    std::string last_error() const override;

private:
    wireless::RtspStreamer* get_streamer();
    bool fail(std::string message) noexcept;

    OutputConfig output_config_;
    wireless::RtspStreamConfig stream_config_;
    cv::Mat rendered_frame_;
    cv::Mat rtsp_frame_;
    std::mutex stream_mutex_;
    std::unique_ptr<wireless::RtspStreamer> streamer_;
    std::atomic<wireless::RtspStreamer*> streamer_ptr_{nullptr};
    std::atomic<bool> cancelled_{false};
    mutable std::mutex error_mutex_;
    std::string last_error_;
};

}  // namespace mosas::output

#endif  // MOSAS_OUTPUT_RTSP_VISUALIZATION_SINK_HPP
