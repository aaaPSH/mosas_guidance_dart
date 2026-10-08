#include <mosas/output/local_recording_session.hpp>

#include "recording_session_id.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace mosas::output {
namespace {

std::filesystem::path timestamped_video_path(
    const std::filesystem::path& configured_path,
    const std::string& session_id) {
    std::string stem = configured_path.stem().string();
    if (stem.empty()) {
        stem = "recording";
    }
    std::string extension = configured_path.extension().string();
    if (extension.empty()) {
        extension = ".avi";
    }
    return configured_path.parent_path() /
           (stem + "_" + session_id + extension);
}

}  // namespace

LocalRecordingSession::LocalRecordingSession(LocalRecordingConfig config)
    : config_(std::move(config)) {
    validate();
}

std::size_t LocalRecordingSession::max_window_frame_count() const noexcept {
    if (!valid()) {
        return 0;
    }
    const auto width = static_cast<std::uint64_t>(config_.width);
    const auto height = static_cast<std::uint64_t>(config_.height);
    const std::uint64_t frame_bytes = width * height * 3ULL;
    return static_cast<std::size_t>(config_.max_buffer_bytes / frame_bytes);
}

LocalWindowDecision LocalRecordingSession::classify(
    const runtime::ProcessedFrame& frame, LocalRecordingSinkKind sink) {
    if (!valid()) {
        return {LocalWindowAction::invalid, 0, -1, configuration_error_};
    }
    if (config_.mode == LocalRecordingMode::continuous) {
        return {LocalWindowAction::record, 0, -1, {}};
    }

    std::lock_guard<std::mutex> lock(mutex_);
    WindowState& state = windows_[sink_index(sink)];
    const std::uint64_t generation = frame.flight_state.launch.generation;
    if (state.has_completed_generation &&
        generation <= state.completed_generation) {
        return {LocalWindowAction::ignore, generation, state.end_ns, {}};
    }

    if (!state.active) {
        const std::int64_t start_ns =
            frame.flight_state.launch.transition_time.monotonic_ns;
        if (frame.flight_state.launch.phase != domain::LaunchPhase::launched ||
            start_ns < 0) {
            return {LocalWindowAction::ignore, generation, -1, {}};
        }
        if (start_ns > std::numeric_limits<std::int64_t>::max() -
                           config_.window_duration_ns) {
            return {LocalWindowAction::invalid, generation, -1,
                    "launch-window end timestamp overflow"};
        }
        state.active = true;
        state.generation = generation;
        state.start_ns = start_ns;
        state.end_ns = start_ns + config_.window_duration_ns;
    } else if (state.generation != generation) {
        return {LocalWindowAction::invalid, generation, state.end_ns,
                "launch generation changed before recording window completed"};
    }

    if (frame.frame.timestamp_ns < state.start_ns) {
        return {LocalWindowAction::ignore, generation, state.end_ns, {}};
    }
    if (frame.frame.timestamp_ns >= state.end_ns) {
        return {LocalWindowAction::complete, generation, state.end_ns, {}};
    }
    return {LocalWindowAction::record, generation, state.end_ns, {}};
}

LocalRecordingPaths LocalRecordingSession::paths_for_generation(
    std::uint64_t generation) {
    std::lock_guard<std::mutex> lock(mutex_);
    const std::uint64_t path_generation =
        config_.mode == LocalRecordingMode::continuous ? 0 : generation;
    if (paths_.has_value() && path_generation_ == path_generation) {
        return *paths_;
    }
    if (paths_.has_value()) {
        throw std::logic_error(
            "recording path requested before previous generation finalized");
    }

    const std::filesystem::path configured_path(config_.recording_path);
    LocalRecordingPaths paths;
    paths.video = timestamped_video_path(configured_path,
                                         detail::make_session_id());
    paths.telemetry_csv = paths.video;
    paths.telemetry_csv.replace_extension(".csv");
    paths_ = paths;
    path_generation_ = path_generation;
    return paths;
}

void LocalRecordingSession::mark_completed(LocalRecordingSinkKind sink,
                                           std::uint64_t generation) noexcept {
    if (config_.mode != LocalRecordingMode::launch_window) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    WindowState& state = windows_[sink_index(sink)];
    if (!state.active || state.generation != generation) {
        return;
    }
    state.active = false;
    state.has_completed_generation = true;
    state.completed_generation = generation;
    completed_sink_mask_ |= sink_bit(sink);
    if (completed_sink_mask_ == (sink_bit(LocalRecordingSinkKind::video) |
                                 sink_bit(LocalRecordingSinkKind::telemetry))) {
        completed_sink_mask_ = 0;
        rearm_request_pending_ = true;
        paths_.reset();
        path_generation_.reset();
    }
}

bool LocalRecordingSession::take_rearm_request() noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    const bool pending = rearm_request_pending_;
    rearm_request_pending_ = false;
    return pending;
}

void LocalRecordingSession::reset() noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    windows_ = {};
    completed_sink_mask_ = 0;
    rearm_request_pending_ = false;
    path_generation_.reset();
    paths_.reset();
}

std::size_t LocalRecordingSession::sink_index(
    LocalRecordingSinkKind sink) noexcept {
    return sink == LocalRecordingSinkKind::video ? 0 : 1;
}

std::uint8_t LocalRecordingSession::sink_bit(
    LocalRecordingSinkKind sink) noexcept {
    return sink == LocalRecordingSinkKind::video ? 0x01 : 0x02;
}

void LocalRecordingSession::validate() {
    if (config_.recording_path.empty()) {
        configuration_error_ = "recording path must not be empty";
    } else if (!std::isfinite(config_.fps) || config_.fps <= 0.0) {
        configuration_error_ = "recording FPS must be positive";
    } else if (config_.width <= 0 || config_.height <= 0) {
        configuration_error_ = "recording output size must be positive";
    } else if (config_.mode == LocalRecordingMode::launch_window &&
               config_.window_duration_ns <= 0) {
        configuration_error_ = "launch-window duration must be positive";
    } else if (config_.mode == LocalRecordingMode::launch_window &&
               (config_.max_buffer_bytes == 0 ||
                config_.max_buffer_bytes > kMatchRecordingMemoryLimitBytes)) {
        configuration_error_ =
            "launch-window buffer limit is outside the allowed range";
    }
}

}  // namespace mosas::output
