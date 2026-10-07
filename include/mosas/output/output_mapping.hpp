#ifndef MOSAS_OUTPUT_OUTPUT_MAPPING_HPP
#define MOSAS_OUTPUT_OUTPUT_MAPPING_HPP

#include <mosas/domain/algorithm_results.hpp>
#include <mosas/domain/contracts.hpp>
#include <mosas/vision/vision_types.hpp>

namespace mosas::output::detail {

VisionResult make_legacy_vision_result(
    const domain::VisionDetection& detection);
VisionOverlayData make_legacy_overlay(
    const domain::GuidanceOverlay& overlay,
    const domain::FlightStateSnapshot& state);

}  // namespace mosas::output::detail

#endif  // MOSAS_OUTPUT_OUTPUT_MAPPING_HPP
