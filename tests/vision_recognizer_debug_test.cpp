#include <mosas/vision/vision_recognizer.hpp>
#include <mosas/vision/vision_visualizer.hpp>

#include <cassert>
#include <type_traits>
#include <utility>

#include <opencv2/imgproc.hpp>

namespace {

template <typename T, typename = void>
struct has_debug_stage : std::false_type {};

template <typename T>
struct has_debug_stage<
    T, std::void_t<decltype(std::declval<T>().debug_stage)>>
    : std::true_type {};

template <typename T, typename = void>
struct has_vision_diagnostic_counts : std::false_type {};

template <typename T>
struct has_vision_diagnostic_counts<
    T, std::void_t<decltype(std::declval<T>().mask_pixel_count),
                   decltype(std::declval<T>().component_count),
                   decltype(std::declval<T>().candidate_count)>>
    : std::true_type {};

template <typename T, typename = void>
struct has_suspicious_blob : std::false_type {};

template <typename T>
struct has_suspicious_blob<
    T, std::void_t<decltype(std::declval<T>().debug_blob),
                   decltype(std::declval<T>().debug_blob_valid)>>
    : std::true_type {};

template <typename T, typename = void>
struct has_diagnostic_visualizer : std::false_type {};

template <typename T>
struct has_diagnostic_visualizer<
    T, std::void_t<decltype(T::draw_vision_debug(
           std::declval<cv::Mat&>(), std::declval<const VisionResult&>()))>>
    : std::true_type {};

static_assert(!has_debug_stage<VisionResult>::value,
              "VisionResult 不应包含视觉阶段诊断");
static_assert(!has_vision_diagnostic_counts<VisionResult>::value,
              "VisionResult 不应包含视觉诊断计数");
static_assert(!has_suspicious_blob<VisionResult>::value,
              "VisionResult 不应暴露未通过识别的可疑区域");
static_assert(!has_diagnostic_visualizer<VisionVisualizer>::value,
              "VisionVisualizer 不应提供诊断专用叠加");

VisionConfig make_config() {
    return {{0, 0, 64, 48}, {35, 85, 80, 255, 100, 255}, 4, 4, 8, 0.5,
            0.35};
}

void test_target_detection_keeps_blob_and_expands_roi() {
    VisionRecognizer recognizer(make_config());
    cv::Mat frame = cv::Mat::zeros(48, 64, CV_8UC3);
    cv::circle(frame, {32, 24}, 5, cv::Scalar(0, 255, 0), cv::FILLED);

    const VisionResult result = recognizer.process(frame);

    assert(result.found);
    assert(result.blob.area > 0);
    assert(result.blob.width > 0);
    assert(result.blob.height > 0);
    assert(result.next_roi.width >= result.blob.width);
    assert(result.next_roi.height >= result.blob.height);
}

void test_missing_target_keeps_global_search_roi() {
    VisionRecognizer recognizer(make_config());
    const cv::Mat frame = cv::Mat::zeros(48, 64, CV_8UC3);

    const VisionResult result = recognizer.process(frame);

    assert(!result.found);
    assert(result.next_roi.x == 0);
    assert(result.next_roi.y == 0);
    assert(result.next_roi.width == 64);
    assert(result.next_roi.height == 48);
}

void test_normal_visualizer_draws_detection_status() {
    cv::Mat frame = cv::Mat::zeros(48, 64, CV_8UC3);
    VisionResult result{};

    VisionVisualizer::draw_result(frame, result);

    assert(cv::countNonZero(frame.reshape(1)) > 0);
}

void test_normal_visualizer_draws_target_box_and_roi() {
    cv::Mat frame = cv::Mat::zeros(48, 64, CV_8UC3);
    VisionResult result{};
    result.found = true;
    result.blob = {20, 15, 10, 10, 80, 25.0, 20.0};
    result.next_roi = {16, 11, 18, 18};

    VisionVisualizer::draw_result(frame, result);

    assert(cv::countNonZero(frame.reshape(1)) > 0);
    assert(frame.at<cv::Vec3b>(11, 16)[2] > 0);
    assert(frame.at<cv::Vec3b>(15, 20)[0] > 0);
}

}  // 匿名命名空间结束

int main() {
    test_target_detection_keeps_blob_and_expands_roi();
    test_missing_target_keeps_global_search_roi();
    test_normal_visualizer_draws_detection_status();
    test_normal_visualizer_draws_target_box_and_roi();
    return 0;
}
