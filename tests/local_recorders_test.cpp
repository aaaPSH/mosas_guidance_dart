#include <mosas/output/local_recording_session.hpp>
#include <mosas/output/local_video_recorder.hpp>
#include <mosas/output/telemetry_csv_recorder.hpp>

#include "test_support.hpp"

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/videoio.hpp>

namespace {

class TemporaryDirectory final {
public:
    TemporaryDirectory() {
        const auto suffix = std::chrono::steady_clock::now()
                                .time_since_epoch()
                                .count();
        path_ = std::filesystem::temp_directory_path() /
                ("mosas_local_recorders_" + std::to_string(suffix));
        std::filesystem::create_directories(path_);
    }

    ~TemporaryDirectory() {
        std::error_code ignored;
        std::filesystem::remove_all(path_, ignored);
    }

    const std::filesystem::path& path() const noexcept { return path_; }

private:
    std::filesystem::path path_;
};

mosas::runtime::ProcessedFrame make_frame(std::uint64_t frame_id,
                                          std::int64_t timestamp_ns,
                                          std::int64_t transition_ns = -1,
                                          std::uint64_t generation = 0) {
    constexpr int kWidth = 16;
    constexpr int kHeight = 16;
    auto pixels = std::make_shared<std::vector<std::uint8_t>>(
        kWidth * kHeight * 3, static_cast<std::uint8_t>(frame_id));
    mosas::runtime::ProcessedFrame frame{};
    frame.frame.source_frame_id = frame_id;
    frame.frame.timestamp_ns = timestamp_ns;
    frame.frame.received_timestamp_ns = timestamp_ns + 1;
    frame.frame.image.owner = pixels;
    frame.frame.image.data = pixels->data();
    frame.frame.image.data_size_bytes = pixels->size();
    frame.frame.image.width = kWidth;
    frame.frame.image.height = kHeight;
    frame.frame.image.stride_bytes = kWidth * 3;
    frame.frame.image.format = mosas::domain::PixelFormat::bgr8;
    frame.flight_state.time.monotonic_ns = timestamp_ns;
    frame.flight_state.launch.generation = generation;
    frame.flight_state.launch.transition_time.monotonic_ns = transition_ns;
    if (transition_ns >= 0) {
        frame.flight_state.launch.phase = mosas::domain::LaunchPhase::launched;
    }
    return frame;
}

mosas::output::LocalRecordingConfig make_config(
    const std::filesystem::path& path,
    mosas::output::LocalRecordingMode mode) {
    mosas::output::LocalRecordingConfig config;
    config.recording_path = path.string();
    config.fps = 10.0;
    config.width = 16;
    config.height = 16;
    config.mode = mode;
    config.window_duration_ns = 30;
    return config;
}

std::vector<std::filesystem::path> find_extension(
    const std::filesystem::path& directory, const std::string& extension) {
    std::vector<std::filesystem::path> files;
    for (const auto& entry : std::filesystem::directory_iterator(directory)) {
        if (entry.path().extension() == extension) {
            files.push_back(entry.path());
        }
    }
    return files;
}

std::vector<std::string> read_lines(const std::filesystem::path& path) {
    std::ifstream input(path);
    std::vector<std::string> lines;
    std::string line;
    while (std::getline(input, line)) {
        lines.push_back(line);
    }
    return lines;
}

std::size_t count_video_frames(const std::filesystem::path& path) {
    cv::VideoCapture video(path.string());
    if (!video.isOpened()) {
        return 0;
    }
    std::size_t count = 0;
    cv::Mat frame;
    while (video.read(frame)) {
        ++count;
    }
    return count;
}

}  // namespace

int main() {
    using namespace mosas;

    {
        TemporaryDirectory temporary_directory;
        const auto config = make_config(temporary_directory.path() / "flight.avi",
                                        output::LocalRecordingMode::continuous);
        auto session = std::make_shared<output::LocalRecordingSession>(config);
        output::LocalVideoRecorder video(session);
        output::TelemetryCsvRecorder telemetry(session);
        CHECK(video.start());
        CHECK(telemetry.start());
        CHECK(video.record(make_frame(101, 1'000)));
        CHECK(telemetry.record(make_frame(101, 1'000)));
        CHECK(video.record(make_frame(102, 2'000)));
        CHECK(telemetry.record(make_frame(102, 2'000)));
        video.stop();
        telemetry.stop();
        CHECK(video.last_error().empty());
        CHECK(telemetry.last_error().empty());

        const auto videos = find_extension(temporary_directory.path(), ".avi");
        const auto csv_files = find_extension(temporary_directory.path(), ".csv");
        CHECK(videos.size() == 1);
        CHECK(csv_files.size() == 1);
        CHECK(videos[0].stem() == csv_files[0].stem());
        CHECK(count_video_frames(videos[0]) == 2);
        const auto lines = read_lines(csv_files[0]);
        CHECK(lines.size() == 3);
        CHECK(lines[0].find("frame_index,frame_timestamp_ns,") == 0);
        CHECK(lines[0].find("command_phase_rad,source_frame_id") !=
              std::string::npos);
        CHECK(lines[0].substr(lines[0].rfind(',') + 1) == "source_frame_id");
        CHECK(lines[1].substr(lines[1].rfind(',') + 1) == "101");
        CHECK(lines[2].substr(lines[2].rfind(',') + 1) == "102");
    }

    {
        TemporaryDirectory temporary_directory;
        const auto config = make_config(temporary_directory.path() / "match.avi",
                                        output::LocalRecordingMode::launch_window);
        auto session = std::make_shared<output::LocalRecordingSession>(config);
        output::LocalVideoRecorder video(session);
        output::TelemetryCsvRecorder telemetry(session);
        CHECK(video.start());
        CHECK(telemetry.start());

        const auto before_launch = make_frame(200, 90, 100, 7);
        const auto first = make_frame(201, 100, 100, 7);
        const auto second = make_frame(202, 110, 100, 7);
        const auto end = make_frame(203, 130, 100, 7);
        CHECK(video.record(before_launch));
        CHECK(telemetry.record(before_launch));
        CHECK(video.record(first));
        CHECK(telemetry.record(first));
        CHECK(video.record(second));
        CHECK(telemetry.record(second));
        CHECK(video.record(end));
        CHECK(!video.take_rearm_request());
        CHECK(telemetry.record(end));
        CHECK(telemetry.take_rearm_request());
        CHECK(!video.take_rearm_request());
        video.stop();
        telemetry.stop();
        CHECK(video.last_error().empty());
        CHECK(telemetry.last_error().empty());

        const auto videos = find_extension(temporary_directory.path(), ".avi");
        const auto csv_files = find_extension(temporary_directory.path(), ".csv");
        CHECK(videos.size() == 1);
        CHECK(csv_files.size() == 1);
        CHECK(videos[0].stem() == csv_files[0].stem());
        CHECK(count_video_frames(videos[0]) == 2);
        const auto lines = read_lines(csv_files[0]);
        CHECK(lines.size() == 3);
        CHECK(lines[1].substr(lines[1].rfind(',') + 1) == "201");
        CHECK(lines[2].substr(lines[2].rfind(',') + 1) == "202");
    }

    {
        TemporaryDirectory temporary_directory;
        const auto config = make_config(temporary_directory.path() / "partial.avi",
                                        output::LocalRecordingMode::launch_window);
        auto session = std::make_shared<output::LocalRecordingSession>(config);
        output::LocalVideoRecorder video(session);
        output::TelemetryCsvRecorder telemetry(session);
        CHECK(video.start());
        CHECK(telemetry.start());
        const auto first = make_frame(301, 100, 100, 8);
        const auto second = make_frame(302, 110, 100, 8);
        CHECK(video.record(first));
        CHECK(telemetry.record(first));
        CHECK(video.record(second));
        CHECK(telemetry.record(second));
        video.stop();
        telemetry.stop();
        CHECK(!video.take_rearm_request());
        CHECK(!telemetry.take_rearm_request());
        CHECK(video.last_error().empty());
        CHECK(telemetry.last_error().empty());
        const auto videos = find_extension(temporary_directory.path(), ".avi");
        const auto csv_files = find_extension(temporary_directory.path(), ".csv");
        CHECK(videos.size() == 1);
        CHECK(csv_files.size() == 1);
        CHECK(count_video_frames(videos[0]) == 2);
        CHECK(read_lines(csv_files[0]).size() == 3);
    }

    // Odd native sensor sizes must retain the last row/column in both modes.
    for (const auto mode : {output::LocalRecordingMode::continuous,
                            output::LocalRecordingMode::launch_window}) {
        TemporaryDirectory temporary_directory;
        auto config = make_config(temporary_directory.path() / "odd.avi", mode);
        config.width = 17;
        config.height = 15;
        auto session = std::make_shared<output::LocalRecordingSession>(config);
        output::LocalVideoRecorderOptions options;
        options.draw_visualization = false;
        output::LocalVideoRecorder video(session, options);
        CHECK(video.start());
        auto frame = make_frame(400, 100, 100, 9);
        auto pixels = std::make_shared<std::vector<std::uint8_t>>(17 * 15 * 3, 220);
        frame.frame.image = {pixels, pixels->data(), pixels->size(), 17, 15,
                             17 * 3, domain::PixelFormat::bgr8};
        CHECK(video.record(frame));
        video.stop();
        CHECK(video.last_error().empty());
        const auto videos = find_extension(temporary_directory.path(), ".avi");
        CHECK(videos.size() == 1);
        cv::VideoCapture capture(videos.front().string());
        cv::Mat decoded;
        CHECK(capture.read(decoded));
        CHECK(decoded.cols == 18 && decoded.rows == 16);
        CHECK(decoded.at<cv::Vec3b>(14, 16)[0] > 200);
        CHECK(decoded.at<cv::Vec3b>(15, 17)[0] > 200);
        CHECK(!capture.read(decoded));
        CHECK(frame.frame.image.width == 17 && frame.frame.image.height == 15);
        CHECK((*pixels)[0] == 220);
    }

    {
        TemporaryDirectory temporary_directory;
        const auto config = make_config(temporary_directory.path() / "failure.avi",
                                        output::LocalRecordingMode::continuous);
        auto session = std::make_shared<output::LocalRecordingSession>(config);
        output::LocalVideoRecorder video(session);
        output::TelemetryCsvRecorder telemetry(session);
        CHECK(video.start());
        CHECK(telemetry.start());
        auto invalid = make_frame(350, 1'000);
        invalid.frame.image = {};
        CHECK(!video.record(invalid));
        CHECK(!video.last_error().empty());
        CHECK(telemetry.record(make_frame(351, 2'000)));
        telemetry.stop();
        video.stop();
        CHECK(telemetry.last_error().empty());
        CHECK(find_extension(temporary_directory.path(), ".csv").size() == 1);
    }


    return 0;
}
