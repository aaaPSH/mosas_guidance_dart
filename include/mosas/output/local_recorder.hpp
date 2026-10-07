#ifndef MOSAS_OUTPUT_LOCAL_RECORDER_HPP
#define MOSAS_OUTPUT_LOCAL_RECORDER_HPP

#include <mosas/domain/algorithm_results.hpp>
#include <mosas/runtime/runtime_types.hpp>
#include <mosas/output/output_config.hpp>

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include <opencv2/videoio.hpp>

namespace mosas::output {

enum class LocalRecordingMode {
    continuous,
    launch_window,
};

struct LocalRecorderConfig {
    std::string recording_path;
    double fps = 120.0;
    int width = 640;
    int height = 480;
    LocalRecordingMode mode = LocalRecordingMode::continuous;
    std::int64_t window_duration_ns = 10'000'000'000LL;
    std::uint64_t max_buffer_bytes = kMatchRecordingMemoryLimitBytes;
};

struct LocalRecorderStats {
    std::size_t buffered_pixel_bytes = 0;
    std::size_t buffered_frame_count = 0;
    std::uint64_t frames_skipped_for_memory_limit = 0;
    bool launch_window_active = false;
};

class LocalRecorder {
public:
    explicit LocalRecorder(LocalRecorderConfig config);
    ~LocalRecorder();

    LocalRecorder(const LocalRecorder&) = delete;
    LocalRecorder& operator=(const LocalRecorder&) = delete;

    bool publish(const runtime::CameraFrame& frame,
                 const domain::VisionDetection& vision_detection,
                 const domain::FlightStateSnapshot& flight_state,
                 const domain::GuidanceOutput& guidance,
                 const domain::GuidanceOverlay& overlay);

    // 提前停止时保存当前已缓存的数据；部分窗口会在标准错误流中说明。
    void stop() noexcept;

    // 新一轮运行开始时清除状态，并允许从新的起飞周期重新录制。
    void reset() noexcept;

    // 每个完整窗口保存成功后最多返回一次 true。
    bool take_rearm_request() noexcept;

    std::string last_error() const;
    LocalRecorderStats stats() const noexcept;

private:
    struct BufferedFrame {
        cv::Mat image;
        std::string csv_row;
    };

    bool validate_config();
    bool publish_continuous(const runtime::CameraFrame& frame,
                           const domain::VisionDetection& vision_detection,
                           const domain::FlightStateSnapshot& flight_state,
                           const domain::GuidanceOutput& guidance,
                           const domain::GuidanceOverlay& overlay);
    bool publish_launch_window(const runtime::CameraFrame& frame,
                               const domain::VisionDetection& vision_detection,
                               const domain::FlightStateSnapshot& flight_state,
                               const domain::GuidanceOutput& guidance,
                               const domain::GuidanceOverlay& overlay);
    bool prepare_recording_frame(const runtime::CameraFrame& frame,
                                 cv::Mat* output);
    std::string make_csv_row(
        std::uint64_t frame_index, const runtime::CameraFrame& frame,
        const domain::VisionDetection& vision_detection,
        const domain::FlightStateSnapshot& flight_state,
        const domain::GuidanceOutput& guidance,
        const domain::GuidanceOverlay& overlay) const;
    bool open_recording();
    bool write_recording_frame(const cv::Mat& image,
                               const std::string& csv_row);
    bool finalize_recording(std::uint64_t expected_frame_count);
    bool persist_buffered_window(bool complete);
    void release_recording() noexcept;

    LocalRecorderConfig config_;
    std::string last_error_;
    std::string recording_video_path_;
    std::string flight_data_path_;
    cv::Mat recording_bgr_frame_;
    cv::VideoWriter recording_writer_;
    std::ofstream flight_data_writer_;
    std::vector<BufferedFrame> buffered_frames_;
    std::size_t buffered_pixel_bytes_ = 0;
    std::uint64_t recording_frame_index_ = 0;
    std::uint64_t frames_skipped_for_memory_limit_ = 0;
    std::int64_t launch_window_start_ns_ = -1;
    std::int64_t launch_window_end_ns_ = -1;
    std::uint64_t recording_launch_generation_ = 0;
    std::uint64_t completed_launch_generation_ = 0;
    bool has_completed_launch_generation_ = false;
    bool launch_window_active_ = false;
    bool rearm_request_pending_ = false;
    bool stopped_ = false;
    bool failed_ = false;
};

}  // namespace mosas::output

#endif  // MOSAS_OUTPUT_LOCAL_RECORDER_HPP
