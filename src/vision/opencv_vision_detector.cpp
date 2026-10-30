#include <mosas/vision/opencv_vision_detector.hpp>

#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>

#include <cstddef>
#include <limits>

namespace mosas::vision {
namespace {

void copy_diagnostics(const VisionResult& result,
                      domain::VisionDiagnostics* diagnostics) {
    if (diagnostics == nullptr) {
        return;
    }
    diagnostics->next_search_region = {
        result.next_roi.x, result.next_roi.y, result.next_roi.width,
        result.next_roi.height};
    if (result.found) {
        diagnostics->bounding_box = {result.blob.x, result.blob.y,
                                     result.blob.width, result.blob.height};
        diagnostics->area_pixels = result.blob.area;
    }
}

}  // namespace

OpenCvVisionDetector::OpenCvVisionDetector(const VisionConfig& config)
    : recognizer_(config) {}

domain::VisionDetection OpenCvVisionDetector::detect(
    const domain::ImageView& image, std::uint64_t source_frame_id,
    const domain::SensorTime& time) {
    domain::VisionDetection detection{};
    detection.observation.source_frame_id = source_frame_id;
    detection.observation.time = time;
    if (!image.is_valid() ||
        image.stride_bytes >
            static_cast<std::size_t>(std::numeric_limits<std::ptrdiff_t>::max())) {
        detection.observation.reason = domain::ObservationReason::invalid_frame;
        copy_diagnostics(recognizer_.process(cv::Mat{}),
                         &detection.diagnostics);
        return detection;
    }

    try {
        // This header borrows the frame's storage. detect() is synchronous and
        // the caller retains the owning frame for the duration of this call.
        cv::Mat frame(image.height, image.width, CV_8UC3,
                      const_cast<std::uint8_t*>(image.data), image.stride_bytes);
        const VisionResult result = recognizer_.process(frame);
        detection.observation.valid = result.found;
        detection.observation.reason =
            result.found ? domain::ObservationReason::found
                         : domain::ObservationReason::not_found;
        if (result.found) {
            detection.observation.center = {result.blob.center_x,
                                            result.blob.center_y};
        }
        copy_diagnostics(result, &detection.diagnostics);
    } catch (const cv::Exception&) {
        detection.observation.valid = false;
        detection.observation.reason = domain::ObservationReason::invalid_frame;
    }
    return detection;
}

void OpenCvVisionDetector::reset() noexcept { recognizer_.reset(); }

}  // namespace mosas::vision
