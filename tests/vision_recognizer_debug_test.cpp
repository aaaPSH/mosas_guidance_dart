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

void test_two_stage_global_search_and_roi_tracking_lifecycle() {
    VisionRecognizer recognizer(make_config());
    assert(!recognizer.is_tracking());

    // 1. 全局搜索态：通过 1/4 像素池化整图扫描粗筛，再由 ROI 原画质扫描精确定位。
    cv::Mat frame1 = cv::Mat::zeros(48, 64, CV_8UC3);
    cv::circle(frame1, {32, 24}, 5, cv::Scalar(0, 255, 0), cv::FILLED);
    const VisionResult res1 = recognizer.process(frame1);
    assert(res1.found);
    assert(recognizer.is_tracking());
    assert(std::abs(res1.blob.center_x - 32.0) < 0.5);
    assert(std::abs(res1.blob.center_y - 24.0) < 0.5);

    // 2. 局部跟踪态：目标在当前 ROI 范围内微移，直接在原画质 ROI 内跟踪。
    cv::Mat frame2 = cv::Mat::zeros(48, 64, CV_8UC3);
    cv::circle(frame2, {34, 25}, 5, cv::Scalar(0, 255, 0), cv::FILLED);
    const VisionResult res2 = recognizer.process(frame2);
    assert(res2.found);
    assert(recognizer.is_tracking());
    assert(std::abs(res2.blob.center_x - 34.0) < 0.5);
    assert(std::abs(res2.blob.center_y - 25.0) < 0.5);

    // 3. 丢失目标：局部 ROI 未能检出，退回到全局搜索态，下一帧将重新使用 1/4 池化扫描。
    cv::Mat frame3 = cv::Mat::zeros(48, 64, CV_8UC3);
    const VisionResult res3 = recognizer.process(frame3);
    assert(!res3.found);
    assert(!recognizer.is_tracking());
    assert(res3.next_roi.x == 0 && res3.next_roi.y == 0);
    assert(res3.next_roi.width == 64 && res3.next_roi.height == 48);

    // 4. 重新全局捕获：目标出现在远处 (12, 12)，池化全图粗筛+原画质精检重新锁定。
    cv::Mat frame4 = cv::Mat::zeros(48, 64, CV_8UC3);
    cv::circle(frame4, {12, 12}, 5, cv::Scalar(0, 255, 0), cv::FILLED);
    const VisionResult res4 = recognizer.process(frame4);
    assert(res4.found);
    assert(recognizer.is_tracking());
    assert(std::abs(res4.blob.center_x - 12.0) < 0.5);
    assert(std::abs(res4.blob.center_y - 12.0) < 0.5);
}

void test_fine_scan_original_resolution_accuracy() {
    VisionRecognizer recognizer(make_config());
    cv::Mat frame = cv::Mat::zeros(48, 64, CV_8UC3);
    // 绘制一个非 2 偶数对齐的矩形目标，验证原画质扫描得到的尺寸与坐标非粗糙量子化
    cv::Rect target_rect(15, 11, 7, 7);
    frame(target_rect).setTo(cv::Scalar(0, 255, 0));

    const VisionResult result = recognizer.process(frame);
    assert(result.found);
    assert(result.blob.x == 15);
    assert(result.blob.y == 11);
    assert(result.blob.width == 7);
    assert(result.blob.height == 7);
    assert(result.blob.area == 49);
}

}  // 匿名命名空间结束

int main() {
    test_target_detection_keeps_blob_and_expands_roi();
    test_missing_target_keeps_global_search_roi();
    test_normal_visualizer_draws_detection_status();
    test_normal_visualizer_draws_target_box_and_roi();
    test_two_stage_global_search_and_roi_tracking_lifecycle();
    test_fine_scan_original_resolution_accuracy();
    return 0;
}
