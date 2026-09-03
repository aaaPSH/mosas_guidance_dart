#include <mosas/vision/vision_recognizer.hpp>
#include <mosas/vision/vision_visualizer.hpp>

#include <cassert>
#include <cmath>

#include <opencv2/core.hpp>

namespace {

cv::Mat make_rgb_frame(int width, int height) {
    return cv::Mat(height, width, CV_8UC3, cv::Scalar(0, 0, 0)).clone();
}

void fill_rgb_rect(cv::Mat& frame, int x, int y, int width, int height) {
    frame(cv::Rect(x, y, width, height)).setTo(cv::Scalar(0, 255, 0));
}

void test_finds_green_blob_and_centroid() {
    VisionRecognizer recognizer;
    cv::Mat frame = make_rgb_frame(320, 240);
    fill_rgb_rect(frame, 40, 50, 12, 10);

    const VisionResult result = recognizer.process(frame);

    assert(result.found);
    assert(result.blob.x == 40);
    assert(result.blob.y == 50);
    assert(result.blob.width == 12);
    assert(result.blob.height == 10);
    assert(std::abs(result.blob.center_x - 45.5) < 1e-9);
    assert(std::abs(result.blob.center_y - 54.5) < 1e-9);
    assert(result.next_roi.x == 0);
    assert(result.next_roi.y == 10);
    assert(result.next_roi.width == 92);
    assert(result.next_roi.height == 90);
}

void test_selects_largest_valid_blob() {
    VisionRecognizer recognizer;
    cv::Mat frame = make_rgb_frame(320, 240);
    fill_rgb_rect(frame, 20, 20, 10, 10);
    fill_rgb_rect(frame, 80, 60, 20, 10);

    const VisionResult result = recognizer.process(frame);

    assert(result.found);
    assert(result.blob.x == 80);
    assert(result.blob.y == 60);
    assert(result.blob.width == 20);
    assert(result.blob.height == 10);
    assert(result.blob.area == 200);
}

void test_rejects_invalid_blob_and_restores_initial_roi() {
    VisionConfig config{
        {10, 20, 100, 80},
        {35, 85, 80, 255, 100, 255},
        5,
        5,
        20,
        0.5,
        0.75,
    };
    VisionRecognizer recognizer(config);
    cv::Mat frame = make_rgb_frame(160, 120);
    fill_rgb_rect(frame, 30, 30, 20, 2);

    const VisionResult result = recognizer.process(frame);

    assert(!result.found);
    assert(result.next_roi.x == 10);
    assert(result.next_roi.y == 20);
    assert(result.next_roi.width == 100);
    assert(result.next_roi.height == 80);
}

void test_non_contiguous_frame_is_supported() {
    VisionRecognizer recognizer;
    cv::Mat storage(120, 164, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat frame = storage(cv::Rect(0, 0, 160, 120));
    assert(!frame.isContinuous());
    fill_rgb_rect(frame, 20, 30, 10, 10);

    assert(recognizer.process(frame).found);
}

void test_invalid_mat_returns_not_found() {
    VisionRecognizer recognizer;
    assert(!recognizer.process(cv::Mat()).found);
    assert(!recognizer.process(cv::Mat(20, 20, CV_8UC1)).found);
    assert(!recognizer.process(cv::Mat(20, 20, CV_16UC3)).found);
}

void test_process_does_not_modify_input() {
    VisionRecognizer recognizer;
    cv::Mat frame = make_rgb_frame(320, 240);
    fill_rgb_rect(frame, 40, 50, 12, 10);
    const cv::Mat before = frame.clone();

    recognizer.process(frame);

    assert(cv::norm(frame, before, cv::NORM_INF) == 0.0);
}

void test_visualizer_draws_result() {
    cv::Mat frame = make_rgb_frame(40, 40);
    const VisionResult result{true, {10, 12, 8, 6, 48, 12.25, 13.75},
                              {5, 5, 30, 30}};
    const cv::Point2d line_of_sight_reference_point(20.0, 18.0);

    VisionVisualizer::draw_result(frame, result,
                                  line_of_sight_reference_point);

    assert(frame.at<cv::Vec3b>(5, 5) == cv::Vec3b(0, 0, 255));
    assert(frame.at<cv::Vec3b>(12, 10) == cv::Vec3b(255, 0, 0));
    assert(frame.at<cv::Vec3b>(14, 12) == cv::Vec3b(0, 0, 255));
    assert(frame.at<cv::Vec3b>(15, 14) == cv::Vec3b(0, 0, 0));
    assert(frame.at<cv::Vec3b>(18, 20) == cv::Vec3b(0, 255, 255));
}

void test_visualizer_draws_green_guidance_overlay() {
    cv::Mat frame = make_rgb_frame(320, 240);
    const VisionOverlayData overlay{
        true,
        0.1,
        -0.2,
        true,
        0.3,
        -0.4,
        true,
        1.1,
        -2.2,
        0.5,
        true,
        0.01,
        -0.02,
        0.03,
        true,
        10.0,
        -5.0,
        98.0,
    };

    VisionVisualizer::draw_guidance_overlay(frame, overlay);

    bool has_green_text = false;
    for (int y = 0; y < frame.rows && !has_green_text; ++y) {
        for (int x = 0; x < frame.cols; ++x) {
            if (frame.at<cv::Vec3b>(y, x) == cv::Vec3b(0, 255, 0)) {
                has_green_text = true;
                break;
            }
        }
    }
    assert(has_green_text);
}

}  // namespace

int main() {
    test_finds_green_blob_and_centroid();
    test_selects_largest_valid_blob();
    test_rejects_invalid_blob_and_restores_initial_roi();
    test_non_contiguous_frame_is_supported();
    test_invalid_mat_returns_not_found();
    test_process_does_not_modify_input();
    test_visualizer_draws_result();
    test_visualizer_draws_green_guidance_overlay();
    return 0;
}
