#ifndef MOSAS_RUNTIME_OUTPUT_DIAGNOSTICS_HPP
#define MOSAS_RUNTIME_OUTPUT_DIAGNOSTICS_HPP

#include <cstddef>
#include <cstdint>
#include <string>

namespace mosas::runtime {

struct SinkDiagnostics {
    bool enabled = false;
    bool healthy = true;
    std::uint64_t accepted_count = 0;
    std::uint64_t dropped_count = 0;
    std::uint64_t failure_count = 0;
    std::size_t queue_depth = 0;
    std::size_t queue_capacity = 0;
    std::string last_error;
};

struct RuntimeOutputDiagnostics {
    SinkDiagnostics video;
    SinkDiagnostics telemetry;
    SinkDiagnostics visualization;
    SinkDiagnostics imu_recording;
    SinkDiagnostics logging;
};

}  // namespace mosas::runtime

#endif  // MOSAS_RUNTIME_OUTPUT_DIAGNOSTICS_HPP
