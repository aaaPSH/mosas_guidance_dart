#ifndef MOSAS_OUTPUT_RECORDING_PATHS_HPP
#define MOSAS_OUTPUT_RECORDING_PATHS_HPP

#include <atomic>
#include <chrono>
#include <cstdint>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <string>

namespace mosas::output::detail {

// 为录制目录和录制文件生成带时间及序号的标识。
inline std::string make_session_id() {
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

}  // namespace mosas::output::detail

#endif  // MOSAS_OUTPUT_RECORDING_PATHS_HPP
