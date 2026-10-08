#ifndef MOSAS_OUTPUT_LOCAL_VIDEO_RECORDER_HPP
#define MOSAS_OUTPUT_LOCAL_VIDEO_RECORDER_HPP

#include <mosas/output/local_recording_session.hpp>
#include <mosas/runtime/output_ports.hpp>

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <opencv2/core/types.hpp>
#include <opencv2/videoio.hpp>

namespace mosas::output {

struct LocalVideoRecorderOptions {
    bool draw_visualization = false;
    cv::Point2d line_of_sight_reference_point{};
};

class LocalVideoRecorder final : public runtime::IVideoRecorder {
public:
    explicit LocalVideoRecorder(
        std::shared_ptr<LocalRecordingSession> session,
        LocalVideoRecorderOptions options = {});
    ~LocalVideoRecorder() override;

    LocalVideoRecorder(const LocalVideoRecorder&) = delete;
    LocalVideoRecorder& operator=(const LocalVideoRecorder&) = delete;

    bool start() override;
    void reset() noexcept override;
    bool record(const runtime::ProcessedFrame& frame) override;
    bool take_rearm_request() noexcept override;
    void cancel() noexcept override;
    void stop() noexcept override;
    std::string last_error() const override;

    std::size_t buffered_pixel_bytes() const noexcept;
    std::size_t buffered_frame_count() const noexcept;
    std::uint64_t frames_skipped_for_memory_limit() const noexcept;
    bool launch_window_active() const noexcept;

private:
    bool record_continuous(const runtime::ProcessedFrame& frame);
    bool record_launch_window(const runtime::ProcessedFrame& frame);
    bool prepare_recording_frame(const runtime::ProcessedFrame& frame,
                                 cv::Mat* output);
    bool open_recording(std::uint64_t generation);
    bool write_recording_frame(const cv::Mat& image);
    bool finalize_recording(std::uint64_t expected_frame_count);
    bool persist_buffered_window(bool complete);
    void release_recording() noexcept;
    void set_error(std::string error) noexcept;

    std::shared_ptr<LocalRecordingSession> session_;
    LocalVideoRecorderOptions options_;
    cv::Mat recording_bgr_frame_;
    cv::VideoWriter recording_writer_;
    std::vector<cv::Mat> buffered_frames_;
    std::size_t buffered_pixel_bytes_ = 0;
    std::uint64_t recording_frame_index_ = 0;
    std::uint64_t frames_skipped_for_memory_limit_ = 0;
    std::uint64_t recording_generation_ = 0;
    bool launch_window_active_ = false;
    bool started_ = false;
    bool stopped_ = false;
    std::atomic<bool> cancelled_{false};
    bool failed_ = false;
    mutable std::mutex error_mutex_;
    std::string last_error_;
};

}  // namespace mosas::output

#endif  // MOSAS_OUTPUT_LOCAL_VIDEO_RECORDER_HPP
