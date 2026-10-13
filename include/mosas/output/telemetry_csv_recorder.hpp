#ifndef MOSAS_OUTPUT_TELEMETRY_CSV_RECORDER_HPP
#define MOSAS_OUTPUT_TELEMETRY_CSV_RECORDER_HPP

#include <mosas/output/local_recording_session.hpp>
#include <mosas/runtime/output_ports.hpp>

#include <atomic>
#include <cstdint>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace mosas::output {

/// 本地逐帧制导遥测 CSV sink，与视频 sink 共享窗口代际和文件路径。
class TelemetryCsvRecorder final : public runtime::ITelemetryRecorder {
public:
    /**
     * @brief 创建遥测记录器。
     * @param session 与本地视频 sink 共享的路径和窗口协调会话。
     */
    explicit TelemetryCsvRecorder(
        std::shared_ptr<LocalRecordingSession> session);
    ~TelemetryCsvRecorder() override;

    TelemetryCsvRecorder(const TelemetryCsvRecorder&) = delete;
    TelemetryCsvRecorder& operator=(const TelemetryCsvRecorder&) = delete;

    bool start() override;
    void reset() noexcept override;
    /**
     * @brief 将处理帧的制导状态追加到 CSV 或起飞窗口缓存。
     * @param frame 当前帧及其时间、姿态、视觉和制导快照。
     * @return 记录成功时为 true；文件、窗口状态或序列化失败时为 false。
     * @note 执行文本序列化和文件 I/O；仅由遥测输出 worker 调用。
     */
    bool record(const runtime::ProcessedFrame& frame) override;
    bool take_rearm_request() noexcept override;
    void cancel() noexcept override;
    void stop() noexcept override;
    std::string last_error() const override;

private:
    bool record_continuous(const runtime::ProcessedFrame& frame);
    bool record_launch_window(const runtime::ProcessedFrame& frame);
    std::string make_csv_row(std::uint64_t frame_index,
                             const runtime::ProcessedFrame& frame) const;
    bool open_recording(std::uint64_t generation);
    bool write_csv_row(const std::string& row);
    bool finalize_recording(std::uint64_t expected_row_count);
    bool persist_buffered_window(bool complete);
    void release_recording() noexcept;
    void set_error(std::string error) noexcept;

    std::shared_ptr<LocalRecordingSession> session_;
    std::ofstream writer_;
    std::vector<std::string> buffered_rows_;
    std::uint64_t recording_frame_index_ = 0;
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

#endif  // MOSAS_OUTPUT_TELEMETRY_CSV_RECORDER_HPP
