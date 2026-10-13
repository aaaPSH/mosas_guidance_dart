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

/**
 * @brief 为录像输出提供共享配置、路径分配和按起飞代际划分的窗口协调。
 * @note 文件 writer 仍由各自独立的 sink worker 持有。
 */
class LocalRecordingSession final {
public:
    /**
     * @brief 创建并校验本地录像会话配置。
     * @param config 录像路径、帧率、尺寸、窗口时长和内存上限。
     */
    explicit LocalRecordingSession(LocalRecordingConfig config);

    LocalRecordingSession(const LocalRecordingSession&) = delete;
    LocalRecordingSession& operator=(const LocalRecordingSession&) = delete;

    bool valid() const noexcept { return configuration_error_.empty(); }
    const std::string& configuration_error() const noexcept {
        return configuration_error_;
    }
    const LocalRecordingConfig& config() const noexcept { return config_; }
    /**
     * @brief 按单帧 BGR 像素占用估算允许缓存的最大窗口帧数。
     * @return 当前内存上限下可缓存的帧数；配置无效时为 0。
     */
    std::size_t max_window_frame_count() const noexcept;

    /**
     * @brief 判断帧属于连续录像、起飞窗口、已结束窗口或无效状态。
     * @param frame 含相机单调时间和起飞代际的不可变处理帧。
     * @param sink 正在处理帧的视频或遥测 sink。
     * @return 窗口动作、关联代际/结束时刻；配置或代际错误时包含错误说明。
     */
    LocalWindowDecision classify(const runtime::ProcessedFrame& frame,
                                 LocalRecordingSinkKind sink);

    /**
     * @brief 为指定起飞代际分配同一会话的视频和 CSV 路径。
     * @param generation 起飞代际；连续录像模式忽略此值。
     * @return 本会话的配对文件路径。
     * @throws std::logic_error 上一代路径尚未由视频和遥测 sink 共同完成。
     */
    LocalRecordingPaths paths_for_generation(std::uint64_t generation);

    /**
     * @brief 标记一个 sink 已完成指定代际的起飞窗口。
     * @param sink 完成窗口的视频或遥测 sink。
     * @param generation 已完成的起飞代际。
     * @note 视频和遥测均完成后才产生一次重新布防请求并释放路径。
     */
    void mark_completed(LocalRecordingSinkKind sink,
                        std::uint64_t generation) noexcept;

    /**
     * @brief 消费一次待处理的重新布防请求。
     * @return 存在新请求时为 true；每个请求只返回一次。
     */
    bool take_rearm_request() noexcept;
    /** @brief 清除窗口协调、路径分配及重新布防状态。 */
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
