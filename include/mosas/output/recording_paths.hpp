#ifndef MOSAS_OUTPUT_RECORDING_PATHS_HPP
#define MOSAS_OUTPUT_RECORDING_PATHS_HPP

#include <filesystem>
#include <string>

namespace mosas::output {

/**
 * @brief 为一次程序运行生成供本地录像和 IMU recorder 共用的唯一基础路径。
 * @param configured_recording_path 配置中的录像文件路径。
 * @return 带本次运行会话标识的路径；只生成路径，不创建目录或文件。
 */
std::filesystem::path recording_path_for_run(
    const std::string& configured_recording_path);

}  // namespace mosas::output

#endif  // MOSAS_OUTPUT_RECORDING_PATHS_HPP
