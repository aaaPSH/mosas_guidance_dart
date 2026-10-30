#include <mosas/output/local_recording_session.hpp>
#include <mosas/output/local_video_recorder.hpp>
#include <mosas/output/telemetry_csv_recorder.hpp>

#include <mosas/output/output_mapping.hpp>

#include "body_vector_test_support.hpp"
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

/** @return CSV 行按逗号拆分出的字段数组。 */
std::vector<std::string> split_csv(const std::string& line) {
    std::vector<std::string> tokens;
    std::string::size_type start = 0;
    while (true) {
        const auto end = line.find(',', start);
        if (end == std::string::npos) {
            tokens.push_back(line.substr(start));
            break;
        }
        tokens.push_back(line.substr(start, end - start));
        start = end + 1;
    }
    return tokens;
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
        CHECK(lines[0].find("command_phase_rad,source_frame_id,output_present") !=
              std::string::npos);
        CHECK(lines[0].substr(lines[0].rfind(',') + 1) == "output_mapping_timestamp_ns");
        const auto cols1 = split_csv(lines[1]);
        const auto cols2 = split_csv(lines[2]);
        CHECK(cols1.size() == 41);
        CHECK(cols2.size() == 41);
        CHECK(cols1[34] == "101");
        CHECK(cols2[34] == "102");
        CHECK(cols1[35] == "0");
        CHECK(cols2[35] == "0");
        // 无姿态或尚未输出时对应列留空，不能伪装成有效零值。
        CHECK(cols1[15].empty() && cols1[16].empty() && cols1[17].empty());
        CHECK(cols1[20] == "0");
        CHECK(cols1[29].empty() && cols1[30].empty() && cols1[31].empty());
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
        const auto cols1 = split_csv(lines[1]);
        const auto cols2 = split_csv(lines[2]);
        CHECK(cols1.size() == 41);
        CHECK(cols2.size() == 41);
        CHECK(cols1[34] == "201");
        CHECK(cols2[34] == "202");
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

    {
        TemporaryDirectory temporary_directory;
        const auto config = make_config(temporary_directory.path() / "emission.avi",
                                        output::LocalRecordingMode::continuous);
        auto session = std::make_shared<output::LocalRecordingSession>(config);
        output::TelemetryCsvRecorder telemetry(session);
        CHECK(telemetry.start());

        auto frame = make_frame(501, 5'000'000);
        frame.flight_state.attitude = test::vector_test_attitude(0.25, -0.5, 0.35);
        frame.flight_state.attitude.time.monotonic_ns = frame.frame.timestamp_ns;
        frame.flight_state.imu_sample.angular_velocity = {4.0, 5.0, 6.0};
        frame.overlay.line_of_sight_valid = true;
        frame.overlay.line_of_sight_q_y = {0.1};
        frame.overlay.line_of_sight_q_z = {-0.2};
        frame.overlay.line_of_sight_rate_valid = true;
        frame.overlay.line_of_sight_rate_q_y = {0.3};
        frame.overlay.line_of_sight_rate_q_z = {-0.4};
        domain::CommandEmissionSnapshot emission;
        emission.attempt_id = 7;
        emission.disposition = domain::CommandEmissionDisposition::serial_written;
        emission.mapping.command.valid = true;
        emission.mapping.command.source_frame_id = 501;
        emission.mapping.trace.valid = true;
        emission.mapping.trace.body_overload = {0.0, 0.25, -0.35};
        emission.mapping.trace.command_overload.value = 0.43;
        emission.mapping.trace.command_phase.value = -0.95;
        emission.mapping.trace.attitude.time.monotonic_ns = 5'000'000;
        emission.mapping.trace.emit_time_ns = 5'000'200;
        frame.command_emission = emission;

        CHECK(telemetry.record(frame));
        telemetry.stop();
        CHECK(telemetry.last_error().empty());

        const auto csv_files = find_extension(temporary_directory.path(), ".csv");
        CHECK(csv_files.size() == 1);
        const auto lines = read_lines(csv_files[0]);
        CHECK(lines.size() == 2);
        const auto header = split_csv(lines[0]);
        const auto cols = split_csv(lines[1]);
        CHECK(cols.size() == 41);
        CHECK(header.size() == cols.size());
        CHECK(lines[0].find("imu_angular_velocity_") == std::string::npos);
        CHECK(header[15] == "imu_roll_rad");
        CHECK(header[16] == "imu_pitch_rad");
        CHECK(header[17] == "imu_yaw_rad");
        CHECK(header[23] == "los_inertial_q_y_rad");
        CHECK(header[24] == "los_inertial_q_z_rad");
        CHECK(header[26] == "los_inertial_rate_q_y_rad_s");
        CHECK(header[27] == "los_inertial_rate_q_z_rad_s");
        CHECK(cols[1] == "5000000" && cols[3] == cols[1]);
        CHECK(cols[20] == "1");
        const auto displayed = output::detail::make_legacy_overlay(frame.overlay, frame.flight_state);
        CHECK_NEAR(std::stod(cols[15]), displayed.attitude_roll_rad, 1e-6);
        CHECK_NEAR(std::stod(cols[16]), displayed.attitude_pitch_rad, 1e-6);
        CHECK_NEAR(std::stod(cols[17]), displayed.attitude_yaw_rad, 1e-6);
        CHECK_NEAR(std::stod(cols[23]), displayed.line_of_sight_q_y_rad, 1e-6);
        CHECK_NEAR(std::stod(cols[24]), displayed.line_of_sight_q_z_rad, 1e-6);
        CHECK_NEAR(std::stod(cols[26]), displayed.line_of_sight_rate_q_y_rad_s, 1e-6);
        CHECK_NEAR(std::stod(cols[27]), displayed.line_of_sight_rate_q_z_rad_s, 1e-6);
        CHECK(cols[29] == "0");
        CHECK(cols[30] == "0.25");
        CHECK(cols[31] == "-0.35");
        CHECK(cols[34] == "501");
        CHECK(cols[35] == "1");
        CHECK(cols[36] == "1");
        CHECK(cols[37] == "7");
        CHECK(cols[38] == std::to_string(static_cast<int>(domain::CommandEmissionDisposition::serial_written)));
        CHECK(cols[39] == "5000000");
        CHECK(cols[40] == "5000200");
    }


    return 0;
}
