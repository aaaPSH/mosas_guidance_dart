#include <mosas/app/app_config.hpp>

#include <cassert>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

namespace {

std::filesystem::path write_config_with_suffix(const std::string& suffix) {
    std::ifstream source("config/mosas_guidance.conf");
    assert(source.good());
    std::ostringstream contents;
    contents << source.rdbuf() << suffix;

    const auto unique_suffix = std::chrono::steady_clock::now()
                                   .time_since_epoch()
                                   .count();
    const auto path = std::filesystem::temp_directory_path() /
                      ("mosas_cpu_affinity_removed_" +
                       std::to_string(unique_suffix) + ".conf");
    std::ofstream output(path);
    assert(output.good());
    output << contents.str();
    assert(output.good());
    return path;
}

bool rejected_config(const std::string& suffix,
                     const std::string& expected_error) {
    const auto path = write_config_with_suffix(suffix);
    mosas::app::AppConfig config;
    std::string error;
    const bool loaded = mosas::app::AppConfigLoader::load(path, &config, &error);
    std::filesystem::remove(path);
    if (loaded) {
        std::cerr << "已移除的 CPU 亲和性配置仍被接受\n";
        return false;
    }
    if (error.find(expected_error) == std::string::npos) {
        std::cerr << "旧配置未按预期被解析器拒绝: " << error << '\n';
        return false;
    }
    return true;
}

void test_legacy_threads_section_is_rejected() {
    assert(rejected_config("\n[threads]\nimu_cpu_affinity=0\n",
                           "未知配置节: threads"));
}

void test_legacy_imu_affinity_key_is_rejected() {
    assert(rejected_config("\n[imu]\nreceive_cpu_affinity=0\n",
                           "配置值无效: imu.receive_cpu_affinity=0"));
}

void test_realtime_priority_remains_independent() {
    mosas::app::AppConfig config;
    std::string error;
    assert(mosas::app::AppConfigLoader::load(
        "config/mosas_guidance.conf", &config, &error));
    assert(config.imu.receive_thread.realtime_priority == 20);
    assert(config.imu.receive_thread.require_realtime_priority);
}

}  // namespace

int main() {
    test_legacy_threads_section_is_rejected();
    test_legacy_imu_affinity_key_is_rejected();
    test_realtime_priority_remains_independent();
    return 0;
}
