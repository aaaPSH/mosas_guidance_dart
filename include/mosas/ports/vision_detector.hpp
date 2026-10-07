#ifndef MOSAS_PORTS_VISION_DETECTOR_HPP
#define MOSAS_PORTS_VISION_DETECTOR_HPP

#include <mosas/domain/algorithm_results.hpp>
#include <mosas/domain/image_frame.hpp>

#include <cstdint>

namespace mosas::ports {

class IVisionDetector {
public:
    virtual ~IVisionDetector() = default;

    virtual domain::VisionDetection detect(
        const domain::ImageView& image, std::uint64_t source_frame_id,
        const domain::SensorTime& time) = 0;
    virtual void reset() noexcept = 0;
};

}  // namespace mosas::ports

#endif  // MOSAS_PORTS_VISION_DETECTOR_HPP
