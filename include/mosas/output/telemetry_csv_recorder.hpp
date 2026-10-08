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

class TelemetryCsvRecorder final : public runtime::ITelemetryRecorder {
public:
    explicit TelemetryCsvRecorder(
        std::shared_ptr<LocalRecordingSession> session);
    ~TelemetryCsvRecorder() override;

    TelemetryCsvRecorder(const TelemetryCsvRecorder&) = delete;
    TelemetryCsvRecorder& operator=(const TelemetryCsvRecorder&) = delete;

    bool start() override;
    void reset() noexcept override;
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
