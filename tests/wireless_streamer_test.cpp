#include <filesystem>
#include <fstream>
#include <chrono>
#include <stdexcept>
#include <string>

#include <mosas/wireless/rtsp_frame_sink.hpp>
#include <mosas/wireless/rtsp_streamer.hpp>

#include <opencv2/core.hpp>

namespace {

void test_check(bool condition, const char* expression, const char* file,
                int line) {
    if (!condition) {
        throw std::runtime_error(std::string(file) + ":" +
                                 std::to_string(line) +
                                 ": check failed: " + expression);
    }
}

#define TEST_CHECK(condition) \
    test_check(static_cast<bool>(condition), #condition, __FILE__, __LINE__)

void test_streamer_rejects_non_rtsp_url() {
    mosas::wireless::RtspStreamConfig config;
    config.rtsp_url = "udp://127.0.0.1:5000";

    mosas::wireless::RtspStreamer streamer(config);
    std::string error;
    TEST_CHECK(!streamer.start(&error));
    TEST_CHECK(error.find("rtsp://") != std::string::npos);
}

void test_streamer_rejects_empty_rtsp_url() {
    mosas::wireless::RtspStreamConfig config;
    config.rtsp_url.clear();

    mosas::wireless::RtspStreamer streamer(config);
    std::string error;
    TEST_CHECK(!streamer.start(&error));
    TEST_CHECK(error.find("RTSP") != std::string::npos);
}

void test_frame_sink_switch_disables_wireless_output() {
    mosas::wireless::RtspFrameSinkConfig config;
    config.enable_wireless_stream = false;
    config.stream.rtsp_url = "invalid://disabled-output";

    mosas::wireless::RtspFrameSink sink(config, cv::Point2d(20.0, 18.0));
    mosas::runtime::CameraFrame frame{
        123,
        cv::Mat(40, 40, CV_8UC3, cv::Scalar(0, 0, 0)),
    };
    const VisionResult vision_result{
        false, {0, 0, 0, 0, 0, 0.0, 0.0}, {0, 0, 40, 40}};
    const mosas::runtime::ImuStateSnapshot imu_state{
        123, mosas::runtime::FlightPhase::free_flight,
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
    };
    const PngGuidanceOutput guidance{
        false, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, 0.0,
        0.0,
    };

    TEST_CHECK(sink.publish(frame, vision_result, imu_state, guidance));
}

void test_frame_sink_rejects_recording_without_path() {
    mosas::wireless::RtspFrameSinkConfig config;
    config.enable_recording = true;
    config.enable_wireless_stream = false;

    mosas::wireless::RtspFrameSink sink(config, cv::Point2d(20.0, 18.0));
    mosas::runtime::CameraFrame frame{
        123,
        cv::Mat(40, 40, CV_8UC3, cv::Scalar(0, 0, 0)),
    };
    const VisionResult vision_result{
        false, {0, 0, 0, 0, 0, 0, 0}, {0, 0, 40, 40}};
    const mosas::runtime::ImuStateSnapshot imu_state{};
    const PngGuidanceOutput guidance{};

    TEST_CHECK(!sink.publish(frame, vision_result, imu_state, guidance));
    TEST_CHECK(sink.last_error().find("recording") != std::string::npos);
}

void test_recording_creates_csv_flight_data() {
    const auto unique_id = std::chrono::steady_clock::now().time_since_epoch().count();
    const std::filesystem::path recording_directory =
        std::filesystem::temp_directory_path() /
        ("mosas_flight_data_test_" + std::to_string(unique_id)) / "nested";
    const std::filesystem::path video_path =
        recording_directory / "mosas_flight_data_test.avi";
    const std::filesystem::path csv_path =
        recording_directory / "mosas_flight_data_test.csv";
    std::error_code cleanup_error;
    std::filesystem::remove_all(recording_directory.parent_path(), cleanup_error);

    mosas::wireless::RtspFrameSinkConfig config;
    config.enable_recording = true;
    config.enable_wireless_stream = false;
    // 无线传输已关闭时，即使 RTSP 地址无效也不应影响本地内录。
    config.stream.rtsp_url = "invalid://wireless-disabled";
    config.recording_path = video_path.string();
    config.recording_fps = 30.0;
    config.stream.width = 40;
    config.stream.height = 40;

    {
        mosas::wireless::RtspFrameSink sink(config, cv::Point2d(20.0, 20.0));
        const mosas::runtime::CameraFrame frame{
            123,
            cv::Mat(40, 40, CV_8UC3, cv::Scalar(0, 0, 0)),
        };
        const VisionResult vision_result{
            true, {10, 11, 5, 6, 30, 12.0, 13.0}, {0, 0, 40, 40}};
        const mosas::runtime::ImuStateSnapshot imu_state{
            122, mosas::runtime::FlightPhase::free_flight,
            {0.1, 0.2, 0.3}, {100.0, 2.0, 3.0}, {1.0, 2.0, 9.8},
            {0.01, 0.02, 0.03},
        };
        const PngGuidanceOutput guidance{
            true, {1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}, {0.4, 0.5, 0.6},
            0.7, 0.8,
        };
        VisionOverlayData overlay{};
        overlay.line_of_sight_valid = true;
        overlay.line_of_sight_q_y_rad = 0.11;
        overlay.line_of_sight_q_z_rad = 0.12;
        overlay.line_of_sight_rate_valid = true;
        overlay.line_of_sight_rate_q_y_rad_s = 0.21;
        overlay.line_of_sight_rate_q_z_rad_s = 0.22;
        TEST_CHECK(sink.publish(frame, vision_result, imu_state, guidance,
                                overlay));
    }

    std::ifstream csv(csv_path);
    TEST_CHECK(static_cast<bool>(csv));
    std::string contents((std::istreambuf_iterator<char>(csv)),
                         std::istreambuf_iterator<char>());
    TEST_CHECK(contents.find("frame_timestamp_ns") != std::string::npos);
    TEST_CHECK(contents.find("imu_acceleration_x_mps2") != std::string::npos);
    TEST_CHECK(contents.find("123") != std::string::npos);
    TEST_CHECK(contents.find("free_flight") != std::string::npos);
    TEST_CHECK(contents.find("0.11") != std::string::npos);
    TEST_CHECK(contents.find("0.7") != std::string::npos);

    std::filesystem::remove_all(recording_directory.parent_path(), cleanup_error);
}

void test_recording_survives_wireless_failure() {
    const std::filesystem::path video_path =
        std::filesystem::temp_directory_path() / "mosas_wireless_failure.avi";
    const std::filesystem::path csv_path =
        std::filesystem::temp_directory_path() / "mosas_wireless_failure.csv";
    std::error_code cleanup_error;
    std::filesystem::remove(video_path, cleanup_error);
    std::filesystem::remove(csv_path, cleanup_error);

    mosas::wireless::RtspFrameSinkConfig config;
    config.enable_wireless_stream = true;
    config.enable_recording = true;
    // 用非法地址模拟 RTSP 失败；本地录像不应因此失败。
    config.stream.rtsp_url = "invalid://wireless-failure";
    config.recording_path = video_path.string();
    config.stream.width = 40;
    config.stream.height = 40;

    mosas::wireless::RtspFrameSink sink(config, cv::Point2d(20.0, 20.0));
    const mosas::runtime::CameraFrame frame{
        123,
        cv::Mat(40, 40, CV_8UC3, cv::Scalar(0, 0, 0)),
    };
    const VisionResult vision_result{
        false, {0, 0, 0, 0, 0, 0.0, 0.0}, {0, 0, 40, 40}};
    const mosas::runtime::ImuStateSnapshot imu_state{
        122, mosas::runtime::FlightPhase::free_flight,
        {}, {}, {}, {}};
    const PngGuidanceOutput guidance{false, {}, {}, {}, 0.0, 0.0};

    TEST_CHECK(sink.publish(frame, vision_result, imu_state, guidance));
    TEST_CHECK(std::filesystem::exists(video_path));
    TEST_CHECK(std::filesystem::exists(csv_path));

    std::filesystem::remove(video_path, cleanup_error);
    std::filesystem::remove(csv_path, cleanup_error);
}

}  // namespace

int main() {
    test_streamer_rejects_non_rtsp_url();
    test_streamer_rejects_empty_rtsp_url();
    test_frame_sink_switch_disables_wireless_output();
    test_frame_sink_rejects_recording_without_path();
    test_recording_creates_csv_flight_data();
    test_recording_survives_wireless_failure();
    return 0;
}
