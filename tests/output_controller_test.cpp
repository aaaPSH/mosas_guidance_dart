#include <mosas/output/output_controller.hpp>

#include <atomic>
#include <cassert>
#include <chrono>
#include <filesystem>
#include <string>
#include <thread>

namespace {

namespace fs = std::filesystem;

fs::path make_directory() {
    const auto nonce = std::chrono::steady_clock::now()
                           .time_since_epoch()
                           .count();
    const auto path = fs::temp_directory_path() /
                      ("mosas_output_controller_" + std::to_string(nonce));
    fs::create_directories(path);
    return path;
}

mosas::output::OutputControllerConfig make_config(
    const fs::path& directory, const std::string& stem,
    mosas::output::OutputMode mode) {
    mosas::output::OutputControllerConfig config;
    config.output.mode = mode;
    config.output.recording_path = (directory / (stem + ".avi")).string();
    config.output.draw_visualization = false;
    config.output.match_window_duration_ms = 1;
    config.camera_width = 32;
    config.camera_height = 32;
    config.camera_fps = 120;
    config.wireless.width = 32;
    config.wireless.height = 32;
    return config;
}

mosas::runtime::CameraFrame make_frame(std::int64_t timestamp_ns) {
    mosas::runtime::CameraFrame frame{};
    frame.timestamp_ns = timestamp_ns;
    frame.received_timestamp_ns = timestamp_ns + 1;
    frame.image = cv::Mat(32, 32, CV_8UC3, cv::Scalar(0, 100, 200));
    return frame;
}

mosas::runtime::ImuStateSnapshot make_imu(std::int64_t timestamp_ns,
                                         bool launched = false) {
    mosas::runtime::ImuStateSnapshot state{};
    state.timestamp_ns = timestamp_ns;
    state.attitude_valid = true;
    state.launched = launched;
    if (launched) {
        state.launch_timestamp_ns = timestamp_ns;
        state.launch_generation = 1;
    }
    return state;
}

bool publish(mosas::output::OutputController& controller,
             std::int64_t timestamp_ns,
             const mosas::runtime::ImuStateSnapshot& imu_state) {
    VisionResult vision{};
    PngGuidanceOutput guidance{};
    VisionOverlayData overlay{};
    return controller.publish(make_frame(timestamp_ns), vision, imu_state,
                              guidance, overlay);
}

std::size_t recording_count(const fs::path& directory) {
    std::size_t count = 0;
    for (const auto& entry : fs::directory_iterator(directory)) {
        if (entry.path().extension() == ".avi") {
            ++count;
        }
    }
    return count;
}

void test_debug_mode_writes_continuously_without_selecting_rtsp() {
    const fs::path directory = make_directory();
    auto config = make_config(directory, "debug",
                              mosas::output::OutputMode::debug);
    mosas::output::OutputController controller(config);

    assert(publish(controller, 10, make_imu(10)));
    assert(recording_count(directory) == 1);
    controller.stop();
    fs::remove_all(directory);
}

void test_match_mode_waits_for_window_end_before_writing_and_rearms() {
    const fs::path directory = make_directory();
    auto config = make_config(directory, "match",
                              mosas::output::OutputMode::match);
    mosas::output::OutputController controller(config);
    const auto launch = make_imu(100, true);

    assert(publish(controller, 100, launch));
    assert(recording_count(directory) == 0);
    assert(publish(controller, 1'000'100, launch));
    assert(recording_count(directory) == 1);
    assert(controller.take_rearm_request());
    assert(!controller.take_rearm_request());
    controller.stop();
    fs::remove_all(directory);
}

void test_reset_preserves_config_error_when_configuration_is_invalid() {
    const fs::path directory = make_directory();
    auto config = make_config(directory, "invalid",
                              mosas::output::OutputMode::debug);
    config.camera_width = -1;
    mosas::output::OutputController controller(config);
    assert(!controller.last_error().empty());
    const std::string initial_error = controller.last_error();

    controller.reset();
    assert(controller.last_error() == initial_error);
    fs::remove_all(directory);
}

void test_match_mode_supports_multiple_launch_cycles() {
    const fs::path directory = make_directory();
    auto config = make_config(directory, "match_multi",
                              mosas::output::OutputMode::match);
    mosas::output::OutputController controller(config);

    // Cycle 1: generation 1
    const auto launch1 = make_imu(100, true);
    assert(publish(controller, 100, launch1));
    assert(recording_count(directory) == 0);
    assert(publish(controller, 1'000'100, launch1));
    assert(recording_count(directory) == 1);
    assert(controller.take_rearm_request());

    // Cycle 2: generation 2
    auto launch2 = make_imu(2'000'000, true);
    launch2.launch_timestamp_ns = 2'000'000;
    launch2.launch_generation = 2;
    assert(publish(controller, 2'000'000, launch2));
    assert(recording_count(directory) == 1);
    assert(publish(controller, 3'000'000, launch2));
    assert(recording_count(directory) == 2);
    assert(controller.take_rearm_request());

    controller.stop();
    fs::remove_all(directory);
}

void test_rtsp_cancel_is_nonblocking_and_thread_safe_during_stop() {
    const fs::path directory = make_directory();
    auto config = make_config(directory, "rtsp",
                              mosas::output::OutputMode::rtsp);
    config.wireless.rtsp_url = "rtsp://127.0.0.1:8554/nonexistent_test";
    config.wireless.rtsp_timeout_us = 100'000;
    mosas::output::OutputController controller(config);

    // Initial publish creates rtsp_streamer_ (even if connection fails)
    publish(controller, 10, make_imu(10));

    // Concurrently invoke stop() and cancel() across threads multiple times
    for (int i = 0; i < 5; ++i) {
        controller.reset();
        publish(controller, 10 + i, make_imu(10 + i));

        std::atomic<bool> stop_started{false};
        std::atomic<bool> cancel_done{false};

        std::thread stop_thread([&]() {
            stop_started.store(true);
            controller.stop();
        });

        std::thread cancel_thread([&]() {
            while (!stop_started.load()) {
                std::this_thread::yield();
            }
            const auto start = std::chrono::steady_clock::now();
            controller.cancel();
            const auto elapsed = std::chrono::steady_clock::now() - start;
            assert(elapsed < std::chrono::milliseconds(500));
            cancel_done.store(true);
        });

        cancel_thread.join();
        stop_thread.join();

        assert(cancel_done.load());
    }

    controller.stop();
    fs::remove_all(directory);
}

}  // namespace

int main() {
    test_debug_mode_writes_continuously_without_selecting_rtsp();
    test_match_mode_waits_for_window_end_before_writing_and_rearms();
    test_reset_preserves_config_error_when_configuration_is_invalid();
    test_match_mode_supports_multiple_launch_cycles();
    test_rtsp_cancel_is_nonblocking_and_thread_safe_during_stop();
    return 0;
}


