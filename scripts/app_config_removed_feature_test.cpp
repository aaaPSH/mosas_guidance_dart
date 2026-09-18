#include <mosas/app/app_config.hpp>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

int main() {
    const auto unique_suffix = std::chrono::steady_clock::now()
                                   .time_since_epoch()
                                   .count();
    const std::filesystem::path config_path =
        std::filesystem::temp_directory_path() /
        ("mosas_removed_undistort_" + std::to_string(unique_suffix) + ".conf");

    {
        std::ofstream config(config_path);
        config << "[camera]\nundistort=true\n";
    }

    mosas::app::AppConfig loaded_config;
    std::string error;
    const bool loaded =
        mosas::app::AppConfigLoader::load(config_path, &loaded_config, &error);
    std::filesystem::remove(config_path);

    if (loaded) {
        std::cerr << "已移除的 undistort 配置仍被接受\n";
        return 1;
    }
    if (error.find("未知配置键或值无效: camera.undistort") ==
        std::string::npos) {
        std::cerr << "配置错误信息不符合预期: " << error << '\n';
        return 1;
    }
    return 0;
}
