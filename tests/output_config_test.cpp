#include <mosas/app/app_config.hpp>

#include <cassert>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>

namespace {

std::filesystem::path write_config(const std::string& contents) {
    const auto nonce = std::chrono::steady_clock::now()
                           .time_since_epoch()
                           .count();
    const auto path = std::filesystem::temp_directory_path() /
                      ("mosas_output_config_" + std::to_string(nonce) +
                       ".conf");
    std::ofstream file(path);
    file << contents;
    return path;
}

bool load_config(const std::string& contents,
                 mosas::app::AppConfig* config,
                 std::string* error) {
    const auto path = write_config(contents);
    const bool loaded =
        mosas::app::AppConfigLoader::load(path, config, error);
    std::filesystem::remove(path);
    return loaded;
}

void test_requires_explicit_output_mode() {
    mosas::app::AppConfig config;
    std::string error;
    assert(!load_config("[camera]\nwidth=640\n", &config, &error));
    assert(error.find("output.mode") != std::string::npos);
}

void test_accepts_three_exclusive_modes() {
    for (const std::string mode : {"rtsp", "debug", "match"}) {
        std::string contents = "[output]\nmode=" + mode + "\n";
        if (mode != "rtsp") {
            contents += "recording_path=recordings/mosas_guidance.avi\n";
        }
        contents += "match_window_duration_ms=10000\n";
        mosas::app::AppConfig config;
        std::string error;
        assert(load_config(contents, &config, &error));
    }
}

void test_rejects_unknown_mode_and_removed_boolean_keys() {
    mosas::app::AppConfig config;
    std::string error;
    assert(!load_config("[output]\nmode=both\n", &config, &error));
    assert(error.find("output.mode") != std::string::npos);

    error.clear();
    assert(!load_config(
        "[output]\nmode=debug\nrecording_path=video.avi\n"
        "[wireless]\nenable_recording=true\n",
        &config, &error));
    assert(error.find("enable_recording") != std::string::npos);

    error.clear();
    assert(!load_config(
        "[output]\nmode=rtsp\n"
        "[wireless]\nenable_wireless_stream=true\n",
        &config, &error));
    assert(error.find("enable_wireless_stream") != std::string::npos);

    error.clear();
    assert(!load_config(
        "[output]\nmode=rtsp\ndraw_vision_debug=true\n",
        &config, &error));
    assert(error.find("draw_vision_debug") != std::string::npos);
}

void test_rejects_invalid_and_over_limit_match_windows() {
    mosas::app::AppConfig config;
    std::string error;
    assert(!load_config(
        "[output]\nmode=match\nrecording_path=video.avi\n"
        "match_window_duration_ms=0\n",
        &config, &error));
    assert(error.find("duration") != std::string::npos ||
           error.find("窗口") != std::string::npos);

    error.clear();
    assert(!load_config(
        "[output]\nmode=match\nrecording_path=video.avi\n"
        "match_window_duration_ms=-1\n",
        &config, &error));
    assert(error.find("duration") != std::string::npos ||
           error.find("窗口") != std::string::npos);

    error.clear();
    assert(!load_config(
        "[camera]\nwidth=1280\nheight=720\nfps=120\n"
        "[output]\nmode=match\nrecording_path=video.avi\n"
        "match_window_duration_ms=4000\n",
        &config, &error));
    assert(error.find("memory") != std::string::npos ||
           error.find("内存") != std::string::npos);
}

void test_accepts_default_window_at_640x480_120hz() {
    mosas::app::AppConfig config;
    std::string error;
    assert(load_config(
        "[camera]\nwidth=640\nheight=480\nfps=120\n"
        "[output]\nmode=match\nrecording_path=video.avi\n"
        "match_window_duration_ms=10000\n",
        &config, &error));
}

}  // namespace

int main() {
    test_requires_explicit_output_mode();
    test_accepts_three_exclusive_modes();
    test_rejects_unknown_mode_and_removed_boolean_keys();
    test_rejects_invalid_and_over_limit_match_windows();
    test_accepts_default_window_at_640x480_120hz();
    return 0;
}
