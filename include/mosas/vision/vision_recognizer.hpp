#ifndef VISION_RECOGNIZER_HPP
#define VISION_RECOGNIZER_HPP

#include <mosas/vision/vision_types.hpp>

class VisionRecognizer {
public:
    VisionRecognizer();
    explicit VisionRecognizer(const VisionConfig& config);

    void reset();
    VisionResult process(const Rgb888Frame& frame);
    VisionRoi current_roi() const;

private:
    VisionConfig config_;
    VisionRoi current_roi_;
};

#endif  // VISION_RECOGNIZER_HPP
