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

/// 本地视频 sink；连续模式直接写帧，起飞窗口模式在内存上限内缓存并持久化。
class LocalVideoRecorder final : public runtime::IVideoRecorder {
public:
    /**
     * @brief 创建本地视频记录器。
     * @param session 与遥测 CSV sink 共用的路径及窗口协调会话。
     * @param options 可视化叠加和 LOS 参考点配置。
     */
    explicit LocalVideoRecorder(
        std::shared_ptr<LocalRecordingSession> session,
        LocalVideoRecorderOptions options = {});
    ~LocalVideoRecorder() override;

    LocalVideoRecorder(const LocalVideoRecorder&) = delete;
    LocalVideoRecorder& operator=(const LocalVideoRecorder&) = delete;

    bool start() override;
    void reset() noexcept override;
    /**
     * @brief 将处理帧写入连续录像或当前起飞窗口缓存。
     * @param frame 含拥有型 BGR 图像、单调时间和制导叠加数据的处理帧。
     * @return 帧接受并处理成功时为 true；格式、窗口或 I/O 失败时为 false。
     * @note 可能执行图像处理和磁盘写入；只由视频输出 worker 调用。
     */
    bool record(const runtime::ProcessedFrame& frame) override;
    bool take_rearm_request() noexcept override;
    void cancel() noexcept override;
    void stop() noexcept override;
    std::string last_error() const override;

    /** @brief 返回窗口录像当前缓存的像素字节数。 @return 缓存字节数。 */
    std::size_t buffered_pixel_bytes() const noexcept;
    /** @brief 返回当前缓存帧数。 @return 缓存帧数。 */
    std::size_t buffered_frame_count() const noexcept;
    /** @brief 返回超过配置内存限制而跳过的帧数。 @return 跳帧累计数。 */
    std::uint64_t frames_skipped_for_memory_limit() const noexcept;
    /** @brief 查询是否正在处理起飞窗口录像。 @return 窗口活动时为 true。 */
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
