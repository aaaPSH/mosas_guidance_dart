#include <mosas/vision/opencv_vision_detector.hpp>
#include <mosas/vision/vision_recognizer.hpp>

#include "test_support.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

int main() {
    const VisionConfig config{
        {0, 0, 128, 96},
        {50, 70, 100, 255, 100, 255},
        6,
        4,
        20,
        0.5,
        0.35,
    };
    VisionRecognizer recognizer(config);

    cv::Mat frame(96, 128, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::rectangle(frame, cv::Rect(60, 40, 10, 10), cv::Scalar(0, 255, 0),
                  cv::FILLED);
    const VisionResult found = recognizer.process(frame);
    CHECK(found.found);
    CHECK(found.blob.x == 60);
    CHECK(found.blob.y == 40);
    CHECK(found.blob.width == 10);
    CHECK(found.blob.height == 10);
    CHECK(found.blob.area == 100);
    CHECK_NEAR(found.blob.center_x, 64.5, 1e-12);
    CHECK_NEAR(found.blob.center_y, 44.5, 1e-12);
    CHECK(found.next_roi.x == 54);
    CHECK(found.next_roi.y == 36);
    CHECK(found.next_roi.width == 22);
    CHECK(found.next_roi.height == 18);
    CHECK(recognizer.is_tracking());

    mosas::vision::OpenCvVisionDetector detector(config);
    const auto byte_size = frame.step[0] *
                               static_cast<std::size_t>(frame.rows - 1) +
                           static_cast<std::size_t>(frame.cols) * 3;
    const mosas::domain::ImageView view{
        frame.data, byte_size, frame.cols, frame.rows, frame.step[0],
        mosas::domain::PixelFormat::bgr8};
    mosas::domain::SensorTime frame_time{};
    frame_time.monotonic_ns = 1234;
    const auto adapted = detector.detect(view, 77, frame_time);
    CHECK(adapted.observation.valid);
    CHECK(adapted.observation.reason ==
          mosas::domain::ObservationReason::found);
    CHECK(adapted.observation.source_frame_id == 77);
    CHECK(adapted.observation.time.monotonic_ns == 1234);
    CHECK_NEAR(adapted.observation.center.u, found.blob.center_x, 1e-12);
    CHECK_NEAR(adapted.observation.center.v, found.blob.center_y, 1e-12);
    CHECK(adapted.diagnostics.bounding_box.x == found.blob.x);
    CHECK(adapted.diagnostics.bounding_box.y == found.blob.y);
    CHECK(adapted.diagnostics.bounding_box.width == found.blob.width);
    CHECK(adapted.diagnostics.bounding_box.height == found.blob.height);
    CHECK(adapted.diagnostics.area_pixels == found.blob.area);
    CHECK(adapted.diagnostics.next_search_region.x == found.next_roi.x);
    CHECK(adapted.diagnostics.next_search_region.y == found.next_roi.y);
    CHECK(adapted.diagnostics.next_search_region.width == found.next_roi.width);
    CHECK(adapted.diagnostics.next_search_region.height == found.next_roi.height);

    const cv::Mat empty_frame(96, 128, CV_8UC3, cv::Scalar(0, 0, 0));
    const VisionResult lost = recognizer.process(empty_frame);
    CHECK(!lost.found);
    CHECK(lost.next_roi.x == config.initial_roi.x);
    CHECK(lost.next_roi.y == config.initial_roi.y);
    CHECK(lost.next_roi.width == config.initial_roi.width);
    CHECK(lost.next_roi.height == config.initial_roi.height);
    CHECK(!recognizer.is_tracking());
    const auto adapted_lost = detector.detect(
        {empty_frame.data,
         empty_frame.step[0] * static_cast<std::size_t>(empty_frame.rows - 1) +
             static_cast<std::size_t>(empty_frame.cols) * 3,
         empty_frame.cols, empty_frame.rows, empty_frame.step[0],
         mosas::domain::PixelFormat::bgr8},
        78, frame_time);
    CHECK(!adapted_lost.observation.valid);
    CHECK(adapted_lost.observation.reason ==
          mosas::domain::ObservationReason::not_found);
    CHECK(adapted_lost.diagnostics.next_search_region.x ==
          config.initial_roi.x);

    const cv::Mat invalid_frame;
    const VisionResult invalid = recognizer.process(invalid_frame);
    CHECK(!invalid.found);
    CHECK(invalid.next_roi.x == config.initial_roi.x);
    CHECK(invalid.next_roi.y == config.initial_roi.y);
    CHECK(invalid.next_roi.width == config.initial_roi.width);
    CHECK(invalid.next_roi.height == config.initial_roi.height);
    const auto adapted_invalid = detector.detect(
        {nullptr, 0, 0, 0, 0, mosas::domain::PixelFormat::unknown}, 79,
        frame_time);
    CHECK(!adapted_invalid.observation.valid);
    CHECK(adapted_invalid.observation.reason ==
          mosas::domain::ObservationReason::invalid_frame);
    CHECK(adapted_invalid.diagnostics.next_search_region.x ==
          config.initial_roi.x);
    return 0;
}
