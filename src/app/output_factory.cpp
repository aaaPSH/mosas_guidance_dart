#include <mosas/app/output_factory.hpp>

#include <mosas/output/imu_csv_recorder.hpp>
#include <mosas/output/local_recording_session.hpp>
#include <mosas/output/local_video_recorder.hpp>
#include <mosas/output/recording_paths.hpp>
#include <mosas/output/telemetry_csv_recorder.hpp>
#include <mosas/runtime/async_info_logger.hpp>

#if defined(MOSAS_OUTPUT_HAS_WIRELESS)
#include <mosas/output/rtsp_visualization_sink.hpp>
#endif

#include <cstdint>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <utility>

namespace mosas::app {

runtime::RuntimeOutputPorts make_runtime_output_ports(
    const AppConfig& config, std::ostream& log_output,
    std::unique_ptr<runtime::IVisualizationSink> visualization_override) {
    runtime::RuntimeOutputPorts outputs;
    const std::filesystem::path recording_path =
        output::recording_path_for_run(config.output.recording_path);
    outputs.imu_recorder = std::make_unique<output::ImuCsvRecorder>(
        recording_path.string());
    outputs.logger = std::make_unique<runtime::AsyncInfoLogger>(
        log_output, config.guidance.info_log_queue_capacity);

    switch (config.output.mode) {
        case output::OutputMode::debug:
        case output::OutputMode::match: {
            if (config.output.recording_path.empty()) {
                throw std::invalid_argument(
                    "local recording path must not be empty");
            }
            output::LocalRecordingConfig recorder_config;
            recorder_config.recording_path = recording_path.string();
            recorder_config.fps = config.camera.fps;
            recorder_config.width = config.camera.width;
            recorder_config.height = config.camera.height;
            recorder_config.mode = config.output.mode == output::OutputMode::debug
                                       ? output::LocalRecordingMode::continuous
                                       : output::LocalRecordingMode::launch_window;
            recorder_config.window_duration_ns =
                static_cast<std::int64_t>(
                    config.output.match_window_duration_ms) *
                1'000'000;
            recorder_config.max_buffer_bytes =
                output::kMatchRecordingMemoryLimitBytes;
            auto session = std::make_shared<output::LocalRecordingSession>(
                std::move(recorder_config));
            output::LocalVideoRecorderOptions video_options;
            video_options.draw_visualization = config.output.draw_visualization;
            video_options.line_of_sight_reference_point =
                {config.guidance.camera_intrinsics.cx,
                 config.guidance.camera_intrinsics.cy};
            outputs.video = std::make_unique<output::LocalVideoRecorder>(
                session, video_options);
            outputs.telemetry =
                std::make_unique<output::TelemetryCsvRecorder>(std::move(session));
            return outputs;
        }
        case output::OutputMode::rtsp:
            if (visualization_override != nullptr) {
                outputs.visualization = std::move(visualization_override);
                return outputs;
            }
#if defined(MOSAS_OUTPUT_HAS_WIRELESS)
            outputs.visualization =
                std::make_unique<output::RtspVisualizationSink>(
                    config.output, config.wireless,
                    cv::Point2d{config.guidance.camera_intrinsics.cx,
                                config.guidance.camera_intrinsics.cy});
            return outputs;
#else
            throw std::invalid_argument(
                "RTSP mode is unavailable because wireless support is disabled");
#endif
    }
    throw std::invalid_argument("unknown output mode");
}

}  // namespace mosas::app
