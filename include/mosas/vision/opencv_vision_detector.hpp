#ifndef MOSAS_VISION_OPENCV_VISION_DETECTOR_HPP
#define MOSAS_VISION_OPENCV_VISION_DETECTOR_HPP

#include <mosas/ports/vision_detector.hpp>
#include <mosas/vision/vision_recognizer.hpp>

namespace mosas::vision {

class OpenCvVisionDetector final : public ports::IVisionDetector {
public:
    explicit OpenCvVisionDetector(const VisionConfig& config);

    domain::VisionDetection detect(
        const domain::ImageView& image, std::uint64_t source_frame_id,
        const domain::SensorTime& time) override;
    void reset() noexcept override;

private:
    VisionRecognizer recognizer_;
};

}  // namespace mosas::vision

#endif  // MOSAS_VISION_OPENCV_VISION_DETECTOR_HPP
