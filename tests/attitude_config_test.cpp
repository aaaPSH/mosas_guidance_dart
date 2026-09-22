#include <mosas/app/app_config.hpp>

#include <array>
#include <cassert>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

namespace {

std::filesystem::path default_config_path() {
    for (const auto* prefix : {"", "../", "../../", "../../../"}) {
        const auto candidate =
            std::filesystem::path(prefix) / "config/mosas_guidance.conf";
        if (std::filesystem::exists(candidate)) {
            return candidate;
        }
    }
    return "config/mosas_guidance.conf";
}

std::filesystem::path write_config(const std::string& values) {
    std::ifstream input(default_config_path());
    std::stringstream source;
    source << input.rdbuf();
    const std::string marker = "[output]";
    std::string text = source.str();
    for (const char* key : std::array{
             "attitude_stationary_acceleration_tolerance_mps2",
             "attitude_gyro_stationary_threshold_rad_s",
             "attitude_gyro_bias_filter_alpha",
             "launch_ax_threshold_mps2",
             "launch_confirm_ms",
             "accelerometer_specific_force",
             "png_gravity"}) {
        const std::string assignment = std::string(key) + "=";
        const std::size_t assignment_position = text.find(assignment);
        assert(assignment_position != std::string::npos);
        const std::size_t line_begin = text.rfind('\n', assignment_position);
        const std::size_t line_end = text.find('\n', assignment_position);
        text.erase(line_begin == std::string::npos ? 0 : line_begin + 1,
                   line_end == std::string::npos
                       ? text.size() - (line_begin == std::string::npos
                                           ? 0
                                           : line_begin + 1)
                       : line_end - (line_begin == std::string::npos
                                         ? 0
                                         : line_begin + 1) + 1);
    }
    const std::size_t position = text.find(marker);
    assert(position != std::string::npos);

    const auto suffix = std::chrono::steady_clock::now()
                            .time_since_epoch()
                            .count();
    const auto path = std::filesystem::temp_directory_path() /
                      ("mosas_attitude_config_" + std::to_string(suffix) +
                       ".conf");
    std::ofstream output(path);
    output << text.substr(0, position) << values << '\n'
           << text.substr(position);
    return path;
}

void test_attitude_config_is_loaded() {
    const auto path = write_config(
        "attitude_stationary_acceleration_tolerance_mps2=0.4\n"
        "attitude_gyro_stationary_threshold_rad_s=0.25\n"
        "attitude_gyro_bias_filter_alpha=0.1\n"
        "launch_ax_threshold_mps2=7.5\n"
        "launch_confirm_ms=30\n"
        "accelerometer_specific_force=false");
    mosas::app::AppConfig config;
    std::string error;
    const bool loaded = mosas::app::AppConfigLoader::load(path, &config, &error);
    if (!loaded) {
        std::cerr << "配置加载失败: " << error << '\n';
    }
    assert(loaded);
    std::filesystem::remove(path);
    assert(config.guidance.attitude.stationary_acceleration_tolerance_mps2 ==
           0.4);
    assert(config.guidance.attitude.gyro_stationary_threshold_rad_s == 0.25);
    assert(config.guidance.attitude.gyro_bias_filter_alpha == 0.1);
    assert(config.guidance.attitude.launch_ax_threshold_mps2 == 7.5);
    assert(config.guidance.attitude.launch_confirm_ns == 30'000'000);
    assert(!config.guidance.attitude.accelerometer_specific_force);
}

void test_removed_gravity_tolerance_config_is_rejected() {
    const auto path = write_config("attitude_gravity_tolerance_mps2=0.037");
    mosas::app::AppConfig config;
    std::string error;
    assert(!mosas::app::AppConfigLoader::load(path, &config, &error));
    std::filesystem::remove(path);
}

void test_default_config_file_loads() {
    mosas::app::AppConfig config;
    std::string error;
    assert(mosas::app::AppConfigLoader::load(
        default_config_path(), &config, &error));
    assert(config.guidance.attitude.gravity_mps2 ==
           config.guidance.png_guidance.gravity);
}

void test_invalid_attitude_config_is_rejected() {
    const auto path = write_config("attitude_gyro_bias_filter_alpha=0");
    mosas::app::AppConfig config;
    std::string error;
    assert(!mosas::app::AppConfigLoader::load(path, &config, &error));
    std::filesystem::remove(path);
}

void test_png_gravity_is_shared_with_attitude_estimator() {
    const auto path = write_config("png_gravity=9.81");
    mosas::app::AppConfig config;
    std::string error;
    assert(mosas::app::AppConfigLoader::load(path, &config, &error));
    std::filesystem::remove(path);
    assert(config.guidance.png_guidance.gravity == 9.81);
    assert(config.guidance.attitude.gravity_mps2 == 9.81);
}

}  // namespace

int main() {
    test_default_config_file_loads();
    test_attitude_config_is_loaded();
    test_removed_gravity_tolerance_config_is_rejected();
    test_invalid_attitude_config_is_rejected();
    test_png_gravity_is_shared_with_attitude_estimator();
    return 0;
}
