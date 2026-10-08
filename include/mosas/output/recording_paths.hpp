#ifndef MOSAS_OUTPUT_RECORDING_PATHS_HPP
#define MOSAS_OUTPUT_RECORDING_PATHS_HPP

#include <filesystem>
#include <string>

namespace mosas::output {

// Allocate a unique run path shared by local and IMU recorders. No file I/O.
std::filesystem::path recording_path_for_run(
    const std::string& configured_recording_path);

}  // namespace mosas::output

#endif  // MOSAS_OUTPUT_RECORDING_PATHS_HPP
