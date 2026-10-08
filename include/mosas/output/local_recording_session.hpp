#ifndef MOSAS_OUTPUT_LOCAL_RECORDING_SESSION_HPP
#define MOSAS_OUTPUT_LOCAL_RECORDING_SESSION_HPP

#include <mosas/output/output_config.hpp>
#include <mosas/runtime/processed_frame.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <mutex>
#include <optional>
#include <string>

namespace mosas::output {

enum class LocalRecordingMode {
    continuous,
    launch_window,
};

struct LocalRecordingConfig {
    std::string recording_path;
    double fps = 120.0;
    int width = 640;
    int height = 480;
    LocalRecordingMode mode = LocalRecordingMode::continuous;
    std::int64_t window_duration_ns = 10'000'000'000LL;
    std::uint64_t max_buffer_bytes = kMatchRecordingMemoryLimitBytes;
};

enum class LocalRecordingSinkKind {
    video,
    telemetry,
};

enum class LocalWindowAction {
    ignore,
    record,
    complete,
    invalid,
};

struct LocalWindowDecision {
    LocalWindowAction action = LocalWindowAction::ignore;
    std::uint64_t generation = 0;
    std::int64_t window_end_ns = -1;
    std::string error;
};

struct LocalRecordingPaths {
    std::filesystem::path video;
    std::filesystem::path telemetry_csv;
};

// Shared immutable recording configuration, path allocation, and match-window
// coordination. File writers remain owned by their individual sink workers.
class LocalRecordingSession final {
public:
    explicit LocalRecordingSession(LocalRecordingConfig config);

    LocalRecordingSession(const LocalRecordingSession&) = delete;
    LocalRecordingSession& operator=(const LocalRecordingSession&) = delete;

    bool valid() const noexcept { return configuration_error_.empty(); }
    const std::string& configuration_error() const noexcept {
        return configuration_error_;
    }
    const LocalRecordingConfig& config() const noexcept { return config_; }
    std::size_t max_window_frame_count() const noexcept;

    LocalWindowDecision classify(const runtime::ProcessedFrame& frame,
                                 LocalRecordingSinkKind sink);
    LocalRecordingPaths paths_for_generation(std::uint64_t generation);
    void mark_completed(LocalRecordingSinkKind sink,
                        std::uint64_t generation) noexcept;
    bool take_rearm_request() noexcept;
    void reset() noexcept;

private:
    struct WindowState {
        bool active = false;
        bool has_completed_generation = false;
        std::uint64_t generation = 0;
        std::uint64_t completed_generation = 0;
        std::int64_t start_ns = -1;
        std::int64_t end_ns = -1;
    };

    static std::size_t sink_index(LocalRecordingSinkKind sink) noexcept;
    static std::uint8_t sink_bit(LocalRecordingSinkKind sink) noexcept;
    void validate();

    const LocalRecordingConfig config_;
    std::string configuration_error_;
    mutable std::mutex mutex_;
    std::array<WindowState, 2> windows_{};
    std::uint8_t completed_sink_mask_ = 0;
    bool rearm_request_pending_ = false;
    std::optional<std::uint64_t> path_generation_;
    std::optional<LocalRecordingPaths> paths_;
};

}  // namespace mosas::output

#endif  // MOSAS_OUTPUT_LOCAL_RECORDING_SESSION_HPP
