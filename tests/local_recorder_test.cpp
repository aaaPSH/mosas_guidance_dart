#include <mosas/output/local_recorder.hpp>

#include <opencv2/videoio.hpp>

#include <cassert>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace {

namespace fs = std::filesystem;

fs::path make_test_directory() {
    const auto unique = std::chrono::steady_clock::now()
                            .time_since_epoch()
                            .count();
    fs::path path = fs::temp_directory_path() /
                    ("mosas_local_recorder_test_" + std::to_string(unique));
    fs::create_directories(path);
    return path;
}

mosas::runtime::CameraFrame make_frame(std::int64_t timestamp_ns) {
    mosas::runtime::CameraFrame frame{};
    frame.timestamp_ns = timestamp_ns;
    frame.received_timestamp_ns = timestamp_ns + 1;
    frame.image = cv::Mat(32, 32, CV_8UC3,
                         cv::Scalar(timestamp_ns % 255, 40, 90));
    return frame;
}

mosas::runtime::ImuStateSnapshot make_imu(
    std::int64_t timestamp_ns, bool launched = true,
    std::int64_t launch_timestamp_ns = -1,
    std::uint64_t launch_generation = 0) {
    mosas::runtime::ImuStateSnapshot state{};
    state.timestamp_ns = timestamp_ns;
    state.acceleration = {1.0, 2.0, 3.0};
    state.angular_velocity = {0.1, 0.2, 0.3};
    state.pitch_rad = 0.25;
    state.attitude_valid = true;
    state.launched = launched;
    state.launch_timestamp_ns = launch_timestamp_ns;
    state.launch_generation = launch_generation;
    return state;
}

struct OutputFiles {
    fs::path video;
    fs::path csv;
};

std::vector<OutputFiles> find_recordings(const fs::path& directory,
                                         const std::string& stem) {
    std::vector<OutputFiles> files;
    for (const auto& entry : fs::directory_iterator(directory)) {
        if (!entry.is_regular_file() ||
            entry.path().filename().string().rfind(stem + "_", 0) != 0) {
            continue;
        }
        if (entry.path().extension() == ".avi") {
            fs::path csv = entry.path();
            csv.replace_extension(".csv");
            if (fs::exists(csv)) {
                files.push_back({entry.path(), csv});
            }
        }
    }
    return files;
}

std::size_t csv_data_row_count(const fs::path& path) {
    std::ifstream input(path);
    assert(input.is_open());
    std::string line;
    std::size_t rows = 0;
    assert(static_cast<bool>(std::getline(input, line)));
    while (std::getline(input, line)) {
        ++rows;
    }
    return rows;
}

std::size_t video_frame_count(const fs::path& path) {
    cv::VideoCapture capture(path.string());
    assert(capture.isOpened());
    std::size_t count = 0;
    cv::Mat frame;
    while (capture.read(frame)) {
        ++count;
    }
    return count;
}

mosas::output::LocalRecorderConfig make_config(
    const fs::path& directory, const std::string& stem,
    mosas::output::LocalRecordingMode mode =
        mosas::output::LocalRecordingMode::launch_window) {
    mosas::output::LocalRecorderConfig config;
    config.recording_path = (directory / (stem + ".avi")).string();
    config.fps = 120.0;
    config.width = 32;
    config.height = 32;
    config.mode = mode;
    config.window_duration_ns = 30;
    config.max_buffer_bytes = 65'536;
    return config;
}

bool publish(mosas::output::LocalRecorder& recorder,
             std::int64_t timestamp_ns,
             const mosas::runtime::ImuStateSnapshot& imu) {
    VisionResult vision{};
    vision.found = true;
    vision.blob = {1, 1, 2, 2, 4, 2.0, 2.0};
    PngGuidanceOutput guidance{};
    guidance.valid = true;
    guidance.navigation_acceleration = {1.0, 2.0, 3.0};
    guidance.body_acceleration = {4.0, 5.0, 6.0};
    guidance.body_overload = {0.25, 0.5, 0.75};
    guidance.command_overload = 0.9;
    guidance.command_phase = 0.1;
    VisionOverlayData overlay{};
    overlay.line_of_sight_valid = true;
    overlay.line_of_sight_q_y_rad = 0.4;
    overlay.line_of_sight_q_z_rad = 0.5;
    overlay.line_of_sight_rate_valid = true;
    overlay.line_of_sight_rate_q_y_rad_s = 0.6;
    overlay.line_of_sight_rate_q_z_rad_s = 0.7;
    return recorder.publish(make_frame(timestamp_ns), vision, imu, guidance,
                            overlay);
}

void test_launch_window_is_half_open_and_persisted_rows_match_video() {
    const fs::path directory = make_test_directory();
    auto config = make_config(directory, "window");
    mosas::output::LocalRecorder recorder(config);
    const auto launch = make_imu(100, true, 100, 1);

    assert(publish(recorder, 99, make_imu(99, false)));
    assert(publish(recorder, 100, launch));
    assert(publish(recorder, 110, launch));
    assert(publish(recorder, 129, launch));
    assert(find_recordings(directory, "window").empty());
    assert(publish(recorder, 130, launch));

    const auto recordings = find_recordings(directory, "window");
    assert(recordings.size() == 1);
    assert(csv_data_row_count(recordings.front().csv) == 3);
    assert(video_frame_count(recordings.front().video) == 3);
    assert(recorder.take_rearm_request());
    assert(!recorder.take_rearm_request());
    recorder.stop();
    fs::remove_all(directory);
}

void test_csv_header_keeps_existing_schema() {
    const fs::path directory = make_test_directory();
    auto config = make_config(directory, "schema");
    mosas::output::LocalRecorder recorder(config);
    const auto launch = make_imu(100, true, 100, 1);
    assert(publish(recorder, 100, launch));
    assert(publish(recorder, 130, launch));

    const auto recordings = find_recordings(directory, "schema");
    assert(recordings.size() == 1);
    std::ifstream input(recordings.front().csv);
    std::string header;
    assert(static_cast<bool>(std::getline(input, header)));
    assert(header ==
           "frame_index,frame_timestamp_ns,frame_received_timestamp_ns,"
           "imu_timestamp_ns,vision_found,target_x,target_y,target_width,"
           "target_height,target_area,target_center_x,target_center_y,"
           "imu_acceleration_x_mps2,imu_acceleration_y_mps2,"
           "imu_acceleration_z_mps2,imu_angular_velocity_x_rad_s,"
           "imu_angular_velocity_y_rad_s,imu_angular_velocity_z_rad_s,"
           "imu_pitch_rad,imu_attitude_valid,imu_launched,los_valid,"
           "los_q_y_rad,los_q_z_rad,los_rate_valid,los_rate_q_y_rad_s,"
           "los_rate_q_z_rad_s,guidance_valid,"
           "navigation_acceleration_x_mps2,navigation_acceleration_y_mps2,"
           "navigation_acceleration_z_mps2,body_acceleration_x_mps2,"
           "body_acceleration_y_mps2,body_acceleration_z_mps2,"
           "body_overload_x_g,body_overload_y_g,body_overload_z_g,"
           "command_overload_g,command_phase_rad");
    recorder.stop();
    fs::remove_all(directory);
}

void test_stop_saves_partial_window_without_rearm() {
    const fs::path directory = make_test_directory();
    auto config = make_config(directory, "partial");
    config.window_duration_ns = 1000;
    mosas::output::LocalRecorder recorder(config);
    const auto launch = make_imu(200, true, 200, 7);
    assert(publish(recorder, 200, launch));
    assert(publish(recorder, 210, launch));
    assert(find_recordings(directory, "partial").empty());

    recorder.stop();
    const auto recordings = find_recordings(directory, "partial");
    assert(recordings.size() == 1);
    assert(csv_data_row_count(recordings.front().csv) == 2);
    assert(video_frame_count(recordings.front().video) == 2);
    assert(!recorder.take_rearm_request());
    fs::remove_all(directory);
}

void test_window_without_frames_completes_without_creating_empty_files() {
    const fs::path directory = make_test_directory();
    auto config = make_config(directory, "empty");
    mosas::output::LocalRecorder recorder(config);
    const auto launch = make_imu(100, true, 100, 9);

    assert(publish(recorder, 130, launch));
    assert(find_recordings(directory, "empty").empty());
    assert(!recorder.stats().launch_window_active);
    assert(recorder.take_rearm_request());
    assert(!recorder.take_rearm_request());
    recorder.stop();
    fs::remove_all(directory);
}

void test_pixel_buffer_never_exceeds_cap_and_skips_extra_frames() {
    const fs::path directory = make_test_directory();
    auto config = make_config(directory, "bounded");
    config.window_duration_ns = 100;
    config.max_buffer_bytes = 2 * 32 * 32 * 3;
    mosas::output::LocalRecorder recorder(config);
    const auto launch = make_imu(300, true, 300, 2);
    for (std::int64_t timestamp = 300; timestamp < 340; timestamp += 10) {
        assert(publish(recorder, timestamp, launch));
        const auto stats = recorder.stats();
        assert(stats.buffered_pixel_bytes <= config.max_buffer_bytes);
    }
    assert(recorder.stats().buffered_pixel_bytes == config.max_buffer_bytes);
    assert(recorder.stats().buffered_frame_count == 2);
    assert(recorder.stats().frames_skipped_for_memory_limit == 2);

    recorder.stop();
    const auto recordings = find_recordings(directory, "bounded");
    assert(recordings.size() == 1);
    assert(csv_data_row_count(recordings.front().csv) == 2);
    fs::remove_all(directory);
}

void test_config_cannot_raise_the_global_memory_cap() {
    const fs::path directory = make_test_directory();
    auto config = make_config(directory, "global_cap");
    config.max_buffer_bytes =
        mosas::output::kMatchRecordingMemoryLimitBytes + 1;
    mosas::output::LocalRecorder recorder(config);

    const auto launch = make_imu(400, true, 400, 3);
    assert(!publish(recorder, 400, launch));
    assert(!recorder.last_error().empty());
    assert(find_recordings(directory, "global_cap").empty());
    fs::remove_all(directory);
}

void test_reset_allows_launch_generation_zero_to_start_again() {
    const fs::path directory = make_test_directory();
    auto config = make_config(directory, "reset");
    mosas::output::LocalRecorder recorder(config);

    const auto first_launch = make_imu(100, true, 100, 4);
    assert(publish(recorder, 100, first_launch));
    assert(publish(recorder, 130, first_launch));
    assert(recorder.take_rearm_request());
    recorder.stop();
    const auto first_recordings = find_recordings(directory, "reset");
    assert(first_recordings.size() == 1);

    recorder.reset();
    const auto reset_launch = make_imu(200, true, 200, 0);
    assert(publish(recorder, 200, reset_launch));
    assert(publish(recorder, 230, reset_launch));
    const auto recordings = find_recordings(directory, "reset");
    assert(recordings.size() == 2);
    assert(recorder.take_rearm_request());
    recorder.stop();
    fs::remove_all(directory);
}

void test_write_failure_does_not_request_rearm() {
    const fs::path directory = make_test_directory();
    const fs::path blocker = directory / "not_a_directory";
    {
        std::ofstream output(blocker);
        output << "block";
    }
    auto config = make_config(blocker, "failure");
    mosas::output::LocalRecorder recorder(config);
    const auto launch = make_imu(100, true, 100, 3);
    assert(publish(recorder, 100, launch));
    assert(!publish(recorder, 130, launch));
    assert(!recorder.last_error().empty());
    assert(!recorder.take_rearm_request());
    fs::remove_all(directory);
}

void test_continuous_mode_writes_each_frame() {
    const fs::path directory = make_test_directory();
    auto config = make_config(directory, "continuous",
                              mosas::output::LocalRecordingMode::continuous);
    mosas::output::LocalRecorder recorder(config);
    const auto not_launched = make_imu(10, false);
    assert(publish(recorder, 10, not_launched));
    assert(publish(recorder, 20, not_launched));
    recorder.stop();

    const auto recordings = find_recordings(directory, "continuous");
    assert(recordings.size() == 1);
    assert(csv_data_row_count(recordings.front().csv) == 2);
    assert(video_frame_count(recordings.front().video) == 2);
    fs::remove_all(directory);
}

void test_memory_limit_frame_skip_logs_warning() {
    const fs::path directory = make_test_directory();
    auto config = make_config(directory, "warn_bounded");
    config.window_duration_ns = 100;
    config.max_buffer_bytes = 1 * 32 * 32 * 3;
    mosas::output::LocalRecorder recorder(config);
    const auto launch = make_imu(300, true, 300, 2);

    std::stringstream clog_capture;
    auto* old_clog = std::clog.rdbuf(clog_capture.rdbuf());

    assert(publish(recorder, 300, launch));
    assert(clog_capture.str().empty());

    assert(publish(recorder, 310, launch));
    std::clog.rdbuf(old_clog);

    assert(recorder.stats().frames_skipped_for_memory_limit == 1);
    assert(!clog_capture.str().empty());
    assert(clog_capture.str().find("起飞录制窗口达到内存上限") != std::string::npos);

    recorder.stop();
    fs::remove_all(directory);
}

}  // namespace

int main() {
    test_launch_window_is_half_open_and_persisted_rows_match_video();
    test_csv_header_keeps_existing_schema();
    test_stop_saves_partial_window_without_rearm();
    test_window_without_frames_completes_without_creating_empty_files();
    test_pixel_buffer_never_exceeds_cap_and_skips_extra_frames();
    test_config_cannot_raise_the_global_memory_cap();
    test_reset_allows_launch_generation_zero_to_start_again();
    test_write_failure_does_not_request_rearm();
    test_continuous_mode_writes_each_frame();
    test_memory_limit_frame_skip_logs_warning();
    std::cout << "local_recorder_test passed\n";
    return 0;
}
