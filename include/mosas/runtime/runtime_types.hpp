#ifndef MOSAS_RUNTIME_TYPES_HPP
#define MOSAS_RUNTIME_TYPES_HPP

#include <mosas/domain/algorithm_results.hpp>
#include <mosas/domain/image_frame.hpp>
#include <mosas/runtime/frame_timing.hpp>

#include <cstdint>
#include <optional>

namespace mosas::runtime {

using TimestampNs = std::int64_t;

struct CameraFrame {
    std::uint64_t source_frame_id = 0;
    TimestampNs timestamp_ns = -1;
    TimestampNs received_timestamp_ns = -1;
    CameraTimestampSource timestamp_source = CameraTimestampSource::unknown;
    domain::ImageFrame image;
    // Optional metadata preserves non-host time sources without changing hardware defaults.
    std::optional<domain::SensorTime> sensor_time = std::nullopt;
    /** @brief Acquisition synchronization segment; prevents processing pre-resync queued frames. */
    std::uint64_t acquisition_epoch = 0;
    std::uint64_t acquisition_sequence = 0;
};

using ImuStateSnapshot = domain::FlightStateSnapshot;
using GuidanceCommand = domain::GuidanceCommand;

}  // namespace mosas::runtime

#endif  // MOSAS_RUNTIME_TYPES_HPP
