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

// Rendering and network I/O run on the dispatcher's visualization worker.
class RtspVisualizationSink final : public runtime::IVisualizationSink {
public:
    RtspVisualizationSink(OutputConfig output_config,
                          wireless::RtspStreamConfig stream_config,
                          cv::Point2d line_of_sight_reference_point);
    ~RtspVisualizationSink() override;

    bool start() override;
    void reset() noexcept override;
    bool publish(const runtime::ProcessedFrame& frame) override;
    void cancel() noexcept override;
    void stop() noexcept override;
    std::string last_error() const override;

private:
    wireless::RtspStreamer* get_streamer();
    bool fail(std::string message) noexcept;

    OutputConfig output_config_;
    wireless::RtspStreamConfig stream_config_;
    cv::Point2d line_of_sight_reference_point_;
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
