#ifndef MOSAS_OUTPUT_OUTPUT_CONFIG_HPP
#define MOSAS_OUTPUT_OUTPUT_CONFIG_HPP

#include <cstddef>
#include <cstdint>
#include <string>

namespace mosas::output {

enum class OutputMode : std::uint8_t {
    rtsp,
    debug,
    match,
};

constexpr std::size_t kMatchRecordingMemoryLimitBytes =
    1'105'920'000ULL;

struct OutputConfig {
    OutputMode mode = OutputMode::rtsp;
    std::string recording_path;
    bool draw_visualization = true;
    int match_window_duration_ms = 10'000;
};

// 按配置分辨率、帧率和持续时间预估窗口内需要保留的 BGR 像素字节。
inline bool match_window_fits_memory(int width, int height, int fps,
                                     int duration_ms) noexcept {
    if (width <= 0 || height <= 0 || fps <= 0 || duration_ms <= 0) {
        return false;
    }
    const std::uint64_t width_u64 = static_cast<std::uint64_t>(width);
    const std::uint64_t height_u64 = static_cast<std::uint64_t>(height);
    if (width_u64 > UINT64_MAX / height_u64) {
        return false;
    }
    const std::uint64_t pixels = width_u64 * height_u64;
    if (pixels > kMatchRecordingMemoryLimitBytes / 3) {
        return false;
    }
    const std::uint64_t bytes_per_frame = pixels * 3;
    const std::uint64_t frame_count_numerator =
        static_cast<std::uint64_t>(duration_ms) *
            static_cast<std::uint64_t>(fps) +
        999;
    const std::uint64_t expected_frames = frame_count_numerator / 1'000;
    return expected_frames > 0 &&
           expected_frames <=
               kMatchRecordingMemoryLimitBytes / bytes_per_frame;
}

}  // namespace mosas::output

#endif  // MOSAS_OUTPUT_OUTPUT_CONFIG_HPP
