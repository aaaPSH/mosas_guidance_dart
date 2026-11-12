#include <mosas/output/local_recording_session.hpp>
#include <mosas/output/local_video_recorder.hpp>
#include <mosas/output/telemetry_csv_recorder.hpp>

#include <mosas/output/output_mapping.hpp>

#include "body_vector_test_support.hpp"
#include "output_dispatcher.hpp"
#include "test_support.hpp"

#include <mosas/runtime/command_output.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <future>
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
        CHECK(lines[0].substr(lines[0].rfind(',') + 1) == "body_overload_z_unlimited_g");
        const auto cols1 = split_csv(lines[1]);
        const auto cols2 = split_csv(lines[2]);
        CHECK(cols1.size() == 38);
        CHECK(cols2.size() == 38);
        CHECK(cols1[35].empty() && cols2[35].empty());
        CHECK(cols1[36].empty() && cols1[37].empty());
        CHECK(cols2[36].empty() && cols2[37].empty());
        CHECK(cols1[28] == "101");
        CHECK(cols2[28] == "102");
        CHECK(cols1[29] == "0");
        CHECK(cols2[29] == "0");
        // 无姿态或尚未输出时对应列留空，不能伪装成有效零值。
        CHECK(cols1[11].empty() && cols1[12].empty() && cols1[13].empty());
        CHECK(cols1[14] == "0");
        for (const auto index : {23, 24, 25, 26, 27, 30, 31, 32, 33, 34}) {
            CHECK(cols1[index].empty());
        }
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
        CHECK(cols1.size() == 38);
        CHECK(cols2.size() == 38);
        CHECK(cols1[28] == "201");
        CHECK(cols2[28] == "202");
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
        emission.mapping.trace.unlimited_body_overload = domain::DartOverloadG{0, 1.2500000000000002, -1.75};
        emission.mapping.trace.command_overload.value = 0.43;
        emission.mapping.trace.command_phase.value = -0.95;
        emission.mapping.trace.attitude.time.monotonic_ns = 5'000'000;
        emission.mapping.trace.emit_time_ns = 5'000'200;
        frame.command_emission = emission;

        CHECK(telemetry.record(frame));
        auto zero_frame = frame;
        zero_frame.command_emission->mapping.trace.unlimited_body_overload = domain::DartOverloadG{};
        zero_frame.command_emission->mapping.trace.body_overload = {};
        zero_frame.command_emission->mapping.trace.command_overload = {};
        zero_frame.command_emission->mapping.trace.command_phase = {};
        CHECK(telemetry.record(zero_frame));
        telemetry.stop();
        CHECK(telemetry.last_error().empty());

        const auto csv_files = find_extension(temporary_directory.path(), ".csv");
        CHECK(csv_files.size() == 1);
        const auto lines = read_lines(csv_files[0]);
        CHECK(lines.size() == 3);
        const auto header = split_csv(lines[0]);
        const auto cols = split_csv(lines[1]);
        CHECK(cols.size() == 38);
        CHECK(header[35] == "png_compensated_pitch_rate_rad_s");
        CHECK(header[36] == "body_overload_y_unlimited_g");
        CHECK(header[37] == "body_overload_z_unlimited_g");
        CHECK(header.size() == cols.size());
        for (const auto* removed : {"imu_timestamp_ns", "imu_acceleration_",
                                    "imu_angular_velocity_", "servo_control_"}) {
            CHECK(lines[0].find(removed) == std::string::npos);
        }
        CHECK(header[11] == "imu_roll_rad");
        CHECK(header[12] == "imu_pitch_rad");
        CHECK(header[13] == "imu_yaw_rad");
        CHECK(header[17] == "los_inertial_q_y_rad");
        CHECK(header[18] == "los_inertial_q_z_rad");
        CHECK(header[20] == "los_inertial_rate_q_y_rad_s");
        CHECK(header[21] == "los_inertial_rate_q_z_rad_s");
        CHECK(cols[1] == "5000000");
        CHECK(cols[14] == "1");
        const auto displayed = output::detail::make_legacy_overlay(frame.overlay, frame.flight_state);
        CHECK_NEAR(std::stod(cols[11]), displayed.attitude_roll_rad, 1e-6);
        CHECK_NEAR(std::stod(cols[12]), displayed.attitude_pitch_rad, 1e-6);
        CHECK_NEAR(std::stod(cols[13]), displayed.attitude_yaw_rad, 1e-6);
        CHECK_NEAR(std::stod(cols[17]), displayed.line_of_sight_q_y_rad, 1e-6);
        CHECK_NEAR(std::stod(cols[18]), displayed.line_of_sight_q_z_rad, 1e-6);
        CHECK_NEAR(std::stod(cols[20]), displayed.line_of_sight_rate_q_y_rad_s, 1e-6);
        CHECK_NEAR(std::stod(cols[21]), displayed.line_of_sight_rate_q_z_rad_s, 1e-6);
        CHECK(cols[23] == "0");
        CHECK(cols[24] == "0.25");
        CHECK(cols[25] == "-0.35");
        CHECK(cols[28] == "501");
        CHECK(cols[29] == "1");
        CHECK(cols[30] == "1");
        CHECK(cols[31] == "7");
        CHECK(cols[32] == std::to_string(static_cast<int>(domain::CommandEmissionDisposition::serial_written)));
        CHECK(cols[33] == "5000000");
        CHECK(cols[34] == "5000200");
        CHECK(std::stod(cols[36]) == emission.mapping.trace.unlimited_body_overload->y);
        CHECK(cols[37] == "-1.75");
        const auto zero_columns = split_csv(lines[2]);
        CHECK(zero_columns[36] == "0" && zero_columns[37] == "0");
    }

    // 计算诊断不依赖是否下发；未计算、无效与有效零速率必须区分。
    {
        TemporaryDirectory temporary_directory;
        auto session = std::make_shared<output::LocalRecordingSession>(
            make_config(temporary_directory.path() / "pitch_rate.avi",
                        output::LocalRecordingMode::continuous));
        output::TelemetryCsvRecorder telemetry(session);
        CHECK(telemetry.start());
        auto frame = make_frame(601, 6'000'000);
        frame.guidance.valid = true;
        frame.guidance.diagnostics.compensated_pitch_rate =
            domain::Quantity<domain::RadiansPerSecond>{-0.125};
        CHECK(telemetry.record(frame));
        frame.guidance.valid = false;
        CHECK(telemetry.record(frame));
        frame.guidance.valid = true;
        frame.guidance.diagnostics.compensated_pitch_rate->value = 0.0;
        CHECK(telemetry.record(frame));
        telemetry.stop();
        CHECK(telemetry.last_error().empty());
        const auto files = find_extension(temporary_directory.path(), ".csv");
        CHECK(files.size() == 1);
        const auto lines = read_lines(files.front());
        CHECK(lines.size() == 4);
        const auto valid = split_csv(lines[1]);
        const auto invalid = split_csv(lines[2]);
        const auto zero = split_csv(lines[3]);
        CHECK(valid.size() == 38 && invalid.size() == 38 && zero.size() == 38);
        CHECK(valid[29] == "0");
        CHECK(valid[35] == "-0.125");
        CHECK(invalid[35].empty());
        CHECK(zero[35] == "0");
    }

    // 覆盖真实 CSV 链路：先入队后异步完成发送，重发不得覆盖首次输出。
    {
        TemporaryDirectory temporary_directory;
        const auto config = make_config(temporary_directory.path() / "delayed.avi",
                                        output::LocalRecordingMode::continuous);
        auto session = std::make_shared<output::LocalRecordingSession>(config);
        runtime::CommandOutputProcessor processor;
        std::promise<void> dequeued;
        auto first_dequeued = dequeued.get_future();
        std::size_t dequeued_count = 0;
        std::atomic<std::size_t> published_count{0};
        runtime::OutputDispatcherCallbacks callbacks;
        callbacks.active_generation = [] { return 0; };
        callbacks.record_timing = [&](runtime::OutputDispatchTimingStage stage,
                                      std::chrono::steady_clock::duration) {
            if (stage == runtime::OutputDispatchTimingStage::queue_delay &&
                ++dequeued_count == 1) {
                dequeued.set_value();
            }
        };
        callbacks.record_published_frame = [&] { ++published_count; };
        runtime::OutputDispatcher dispatcher(
            nullptr, std::make_unique<output::TelemetryCsvRecorder>(session),
            nullptr, nullptr, 4, std::move(callbacks));
        dispatcher.reset();
        dispatcher.start();

        auto frame = make_frame(601, 6'000'000);
        frame.guidance.valid = true;
        domain::GuidanceCommand request;
        request.source_frame_id = frame.frame.source_frame_id;
        request.source_time.monotonic_ns = frame.frame.timestamp_ns;
        frame.pending_command_emission = processor.track_emission(
            request, std::chrono::seconds(1));
        CHECK(dispatcher.enqueue(frame));
        CHECK(first_dequeued.wait_for(std::chrono::seconds(1)) ==
              std::future_status::ready);
        CHECK(published_count == 0);

        domain::CommandEmissionSnapshot emission;
        emission.attempt_id = 17;
        emission.mapping.command.source_frame_id = request.source_frame_id;
        emission.mapping.command.source_time = request.source_time;
        emission.mapping.command.valid = true;
        emission.mapping.trace.valid = true;
        emission.mapping.trace.body_overload = {0, 0.25, -0.5};
        emission.mapping.trace.unlimited_body_overload = domain::DartOverloadG{0, 3, -4};
        emission.mapping.trace.command_overload.value = 0.56;
        emission.mapping.trace.command_phase.value = -1.1;
        emission.mapping.trace.attitude.time.monotonic_ns = 6'000'100;
        emission.mapping.trace.emit_time_ns = 6'000'200;
        processor.complete(emission, domain::CommandEmissionDisposition::serial_written);
        emission.attempt_id = 18;
        emission.mapping.trace.body_overload.y = 9;
        emission.mapping.trace.unlimited_body_overload->y = 9;
        processor.complete(emission, domain::CommandEmissionDisposition::serial_written);

        // 无发送不能借用上一帧结果；实际发送的无效零指令则应记录为零。
        CHECK(dispatcher.enqueue(make_frame(602, 6'100'000)));
        auto neutral = make_frame(603, 6'200'000);
        request.source_frame_id = neutral.frame.source_frame_id;
        request.source_time.monotonic_ns = neutral.frame.timestamp_ns;
        neutral.pending_command_emission = processor.track_emission(
            request, std::chrono::seconds(1));
        CHECK(dispatcher.enqueue(neutral));
        emission = {};
        emission.attempt_id = 19;
        emission.mapping.command.source_frame_id = request.source_frame_id;
        emission.mapping.command.source_time = request.source_time;
        emission.mapping.trace.attitude.time.monotonic_ns = 6'200'100;
        emission.mapping.trace.emit_time_ns = 6'200'200;
        processor.complete(emission, domain::CommandEmissionDisposition::serial_written);
        dispatcher.close_queue();
        dispatcher.join();
        CHECK(dispatcher.stop_sink().empty());
        CHECK(published_count == 3);

        const auto csv_files = find_extension(temporary_directory.path(), ".csv");
        CHECK(csv_files.size() == 1);
        const auto lines = read_lines(csv_files[0]);
        CHECK(lines.size() == 4);
        const auto sent = split_csv(lines[1]);
        const auto missing = split_csv(lines[2]);
        const auto zero = split_csv(lines[3]);
        CHECK(sent.size() == 38 && missing.size() == 38 && zero.size() == 38);
        CHECK(sent[22] == "1");
        CHECK(sent[23] == "0" && sent[24] == "0.25" && sent[25] == "-0.5");
        CHECK(sent[26] == "0.56" && sent[27] == "-1.1");
        CHECK(sent[28] == "601" && sent[29] == "1" && sent[30] == "1");
        CHECK(sent[31] == "17");
        CHECK(sent[32] == std::to_string(static_cast<int>(domain::CommandEmissionDisposition::serial_written)));
        CHECK(sent[33] == "6000100" && sent[34] == "6000200");
        CHECK(sent[36] == "3" && sent[37] == "-4");
        CHECK(missing[28] == "602" && missing[29] == "0");
        for (const auto index : {23, 24, 25, 26, 27, 30, 31, 32, 33, 34, 36, 37}) {
            CHECK(missing[index].empty());
        }
        CHECK(zero[28] == "603" && zero[29] == "1" && zero[30] == "0");
        for (std::size_t index = 23; index <= 27; ++index) {
            CHECK(zero[index] == "0");
        }
        CHECK(zero[31] == "19");
        CHECK(zero[33] == "6200100" && zero[34] == "6200200");
        CHECK(zero[36].empty() && zero[37].empty());
    }

    return 0;
}
