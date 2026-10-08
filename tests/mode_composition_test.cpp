#include <mosas/app/output_factory.hpp>
#include <mosas/output/imu_csv_recorder.hpp>
#include <mosas/output/local_video_recorder.hpp>
#include <mosas/output/recording_paths.hpp>
#include <mosas/output/telemetry_csv_recorder.hpp>
#include <mosas/runtime/async_info_logger.hpp>

#if defined(MOSAS_OUTPUT_HAS_WIRELESS)
#include <mosas/output/rtsp_visualization_sink.hpp>
#endif

#include "test_support.hpp"

#include <chrono>
#include <filesystem>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {

class FakeVisualizationSink final : public mosas::runtime::IVisualizationSink {
public:
    bool start() override { return true; }
    void reset() noexcept override {}
    bool publish(const mosas::runtime::ProcessedFrame&) override {
        return true;
    }
    void cancel() noexcept override {}
    void stop() noexcept override {}
    std::string last_error() const override { return {}; }
};

class TemporaryDirectory {
public:
    TemporaryDirectory() {
        path_ = std::filesystem::temp_directory_path() /
                ("mosas-mode-composition-" +
                 std::to_string(std::chrono::steady_clock::now()
                                    .time_since_epoch()
                                    .count()));
        std::filesystem::create_directories(path_);
    }
    ~TemporaryDirectory() {
        std::error_code error;
        std::filesystem::remove_all(path_, error);
    }
    const std::filesystem::path& path() const noexcept { return path_; }

private:
    std::filesystem::path path_;
};

}  // namespace

int main() {
    using namespace mosas;

    TemporaryDirectory temporary_directory;
    app::AppConfig config;
    config.output.recording_path =
        (temporary_directory.path() / "flight.avi").string();
    config.camera.width = 320;
    config.camera.height = 240;
    config.camera.fps = 60;
    config.guidance.info_log_queue_capacity = 8;

    const auto first_path = output::recording_path_for_run(
        config.output.recording_path);
    const auto second_path = output::recording_path_for_run(
        config.output.recording_path);
    CHECK(first_path != second_path);
    CHECK(first_path.filename() == "flight.avi");
    CHECK(first_path.parent_path().parent_path() == temporary_directory.path());
    CHECK(!std::filesystem::exists(first_path.parent_path()));
    CHECK(output::recording_path_for_run("").filename() == "mosas_guidance.avi");
    CHECK(output::recording_path_for_run(
              temporary_directory.path().string() + "/").filename() ==
          "mosas_guidance.avi");

    for (const output::OutputMode mode : {output::OutputMode::debug,
                                          output::OutputMode::match}) {
        config.output.mode = mode;
        std::ostringstream log_output;
        auto outputs = app::make_runtime_output_ports(config, log_output);
        CHECK(outputs.video != nullptr);
        CHECK(outputs.telemetry != nullptr);
        CHECK(outputs.visualization == nullptr);
        CHECK(outputs.imu_recorder != nullptr);
        CHECK(outputs.logger != nullptr);
        CHECK(dynamic_cast<output::LocalVideoRecorder*>(
                  outputs.video.get()) != nullptr);
        CHECK(dynamic_cast<output::TelemetryCsvRecorder*>(
                  outputs.telemetry.get()) != nullptr);
        CHECK(dynamic_cast<output::ImuCsvRecorder*>(
                  outputs.imu_recorder.get()) != nullptr);
        CHECK(dynamic_cast<runtime::AsyncInfoLogger*>(
                  outputs.logger.get()) != nullptr);
    }

    config.output.mode = output::OutputMode::rtsp;
    std::ostringstream log_output;
    auto visualization = std::make_unique<FakeVisualizationSink>();
    FakeVisualizationSink* const visualization_control = visualization.get();
    auto outputs = app::make_runtime_output_ports(
        config, log_output, std::move(visualization));
    CHECK(outputs.video == nullptr);
    CHECK(outputs.telemetry == nullptr);
    CHECK(outputs.visualization.get() == visualization_control);
    CHECK(outputs.imu_recorder != nullptr);
    CHECK(outputs.logger != nullptr);
    CHECK(dynamic_cast<output::ImuCsvRecorder*>(
              outputs.imu_recorder.get()) != nullptr);
    CHECK(dynamic_cast<runtime::AsyncInfoLogger*>(outputs.logger.get()) !=
          nullptr);
#if defined(MOSAS_OUTPUT_HAS_WIRELESS)
    // The real adapter remains lazy: construction/start need no RTSP server.
    auto rtsp_outputs = app::make_runtime_output_ports(config, log_output);
    auto* rtsp = dynamic_cast<output::RtspVisualizationSink*>(
        rtsp_outputs.visualization.get());
    CHECK(rtsp != nullptr);
    CHECK(rtsp->start());
    CHECK(!rtsp->publish(runtime::ProcessedFrame{}));
    CHECK(rtsp->last_error() == "camera frame must be a valid BGR8 image");
    rtsp->cancel();
    CHECK(rtsp->publish(runtime::ProcessedFrame{}));
    rtsp->stop();
    rtsp->reset();
    CHECK(rtsp->last_error().empty());
    CHECK(rtsp->start());
    rtsp->stop();
#else
    bool unavailable = false;
    try {
        app::make_runtime_output_ports(config, log_output);
    } catch (const std::invalid_argument&) {
        unavailable = true;
    }
    CHECK(unavailable);
#endif
    return 0;
}
