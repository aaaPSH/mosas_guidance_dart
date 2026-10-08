#ifndef MOSAS_RUNTIME_PROCESSED_FRAME_HPP
#define MOSAS_RUNTIME_PROCESSED_FRAME_HPP

#include <mosas/domain/algorithm_results.hpp>
#include <mosas/runtime/runtime_types.hpp>

namespace mosas::runtime {

// Each sink receives its own snapshot; image storage remains shared and immutable.
struct ProcessedFrame {
    CameraFrame frame;
    domain::VisionDetection vision_detection{};
    domain::FlightStateSnapshot flight_state{};
    domain::GuidanceOutput guidance{};
    domain::GuidanceOverlay overlay{};
};

}  // namespace mosas::runtime

#endif  // MOSAS_RUNTIME_PROCESSED_FRAME_HPP
