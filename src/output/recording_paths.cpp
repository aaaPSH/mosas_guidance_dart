#include <mosas/output/recording_paths.hpp>

#include "recording_session_id.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <ctime>
#include <iomanip>
#include <sstream>

namespace mosas::output {

std::filesystem::path recording_path_for_run(
    const std::string& configured_recording_path) {
    std::filesystem::path configured_path(configured_recording_path);
    if (configured_path.empty()) {
        configured_path = "recordings/mosas_guidance.avi";
    } else if (configured_path.filename().empty()) {
        configured_path /= "mosas_guidance.avi";
    }

    std::string stem = configured_path.stem().string();
    if (stem.empty()) {
        stem = "recording";
    }
    const std::filesystem::path run_directory =
        configured_path.parent_path() / (stem + "_" + detail::make_session_id());
    return run_directory / configured_path.filename();
}

namespace detail {

std::string make_session_id() {
    static std::atomic<std::uint64_t> next_id{0};
    const auto now = std::chrono::system_clock::now();
    const auto seconds = std::chrono::time_point_cast<std::chrono::seconds>(now);
    const auto micros = std::chrono::duration_cast<std::chrono::microseconds>(
                            now - seconds)
                            .count();
    const std::time_t time = std::chrono::system_clock::to_time_t(now);
    std::tm local_time{};
    localtime_r(&time, &local_time);

    std::ostringstream stream;
    stream << std::put_time(&local_time, "%Y%m%d_%H%M%S") << '_'
           << std::setfill('0') << std::setw(6) << micros << '_'
           << std::setfill('0') << std::setw(6) << next_id.fetch_add(1);
    return stream.str();
}

}  // namespace detail
}  // namespace mosas::output
