#ifndef MOSAS_OUTPUT_OUTPUT_CONTROLLER_HPP
#define MOSAS_OUTPUT_OUTPUT_CONTROLLER_HPP

#include <mosas/output/local_recorder.hpp>
#include <mosas/output/imu_csv_recorder.hpp>
#include <mosas/runtime/runtime_interfaces.hpp>
#include <mosas/wireless/rtsp_streamer.hpp>

#include <atomic>
#include <memory>
#include <mutex>
#include <string>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

namespace mosas::output {

struct OutputControllerConfig {
    OutputConfig output{};
    wireless::RtspStreamConfig wireless{};
    int camera_width = 640;
    int camera_height = 480;
    int camera_fps = 120;
    cv::Point2d line_of_sight_reference_point{};
};

// 将普通可视化分发到 RTSP 或一种本地录像模式。
class OutputController final : public runtime::FrameSink {
public:
    explicit OutputController(OutputControllerConfig config);
    ~OutputController() override;

    OutputController(const OutputController&) = delete;
    OutputController& operator=(const OutputController&) = delete;

    bool publish(const runtime::CameraFrame& frame,
                 const domain::VisionDetection& vision_detection,
                 const domain::FlightStateSnapshot& flight_state,
                 const domain::GuidanceOutput& guidance,
                 const domain::GuidanceOverlay& overlay) override;

    bool try_record_imu_sample(
        const domain::ImuKinematicSample& sample) noexcept override;

    void cancel() noexcept override;
    void reset() noexcept override;
    void stop() noexcept override;
    bool take_rearm_request() noexcept override;
    std::string last_error() const override;

private:
    bool publish_impl(const runtime::CameraFrame& frame,
                      const domain::VisionDetection& vision_detection,
                      const domain::FlightStateSnapshot& flight_state,
                      const domain::GuidanceOutput& guidance,
                      const domain::GuidanceOverlay& overlay);
    bool publish_rtsp(const runtime::CameraFrame& frame,
                      const cv::Mat& rendered_frame);
    bool validate_config();

    OutputControllerConfig config_;
    std::unique_ptr<LocalRecorder> local_recorder_;
    std::unique_ptr<ImuCsvRecorder> imu_csv_recorder_;
    cv::Mat rendered_frame_;
    cv::Mat rtsp_frame_;
#if defined(MOSAS_OUTPUT_HAS_WIRELESS)
    mutable std::mutex rtsp_mutex_;
    std::unique_ptr<wireless::RtspStreamer> rtsp_streamer_;
#endif
    std::atomic<bool> cancelled_{false};
    std::string last_error_;
};

}  // namespace mosas::output

#endif  // MOSAS_OUTPUT_OUTPUT_CONTROLLER_HPP
