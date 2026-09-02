#include <mosas/guidance/guidance_estimator.hpp>
#include <mosas/vision/vision_recognizer.hpp>

#include <opencv2/core.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <string>
#include <vector>

namespace {

struct Options {
    int width = 640;
    int height = 480;
    int iterations = 10000;
    bool full_scan = false;
    bool show_help = false;
};

bool parse_positive_int(const char* text, int* value) {
    if (text == nullptr || value == nullptr || *text == '\0') {
        return false;
    }
    char* end = nullptr;
    const long parsed = std::strtol(text, &end, 10);
    if (*end != '\0' || parsed <= 0 || parsed > 100000000) {
        return false;
    }
    *value = static_cast<int>(parsed);
    return true;
}

void print_usage(const char* program) {
    std::cout << "用法：" << program
              << " [--width N] [--height N] [--iterations N] [--full-scan]\n";
    std::cout << "默认测试 640x480、10000 次、ROI 跟踪模式。\n";
}

bool parse_options(int argc, char** argv, Options* options) {
    if (options == nullptr) {
        return false;
    }
    for (int index = 1; index < argc; ++index) {
        const std::string argument(argv[index]);
        if (argument == "--full-scan") {
            options->full_scan = true;
            continue;
        }
        if (argument == "--help" || argument == "-h") {
            options->show_help = true;
            return true;
        }
        if (argument == "--width" || argument == "--height" ||
            argument == "--iterations") {
            if (index + 1 >= argc) {
                return false;
            }
            int* destination = argument == "--width"
                                   ? &options->width
                                   : argument == "--height" ? &options->height
                                                             : &options->iterations;
            if (!parse_positive_int(argv[++index], destination)) {
                return false;
            }
            continue;
        }
        return false;
    }
    return options->width >= 32 && options->height >= 32;
}

cv::Mat make_frame(int width, int height) {
    cv::Mat frame(height, width, CV_8UC3, cv::Scalar(0, 0, 0));
    const int target_x = width / 2 - 6;
    const int target_y = height / 2 - 5;
    frame(cv::Rect(target_x, target_y, 12, 10)).setTo(cv::Scalar(0, 255, 0));
    return frame;
}

double percentile(std::vector<double> values, double fraction) {
    const std::size_t index = static_cast<std::size_t>(
        fraction * static_cast<double>(values.size() - 1));
    std::nth_element(values.begin(), values.begin() + index, values.end());
    return values[index];
}

int run_benchmark(const Options& options) {
    const VisionConfig vision_config{
        {0, 0, options.width, options.height},
        {35, 85, 80, 255, 100, 255},
        40,
        40,
        8,
        0.5,
        0.35,
    };
    VisionRecognizer recognizer(vision_config);
    const cv::Mat frame = make_frame(options.width, options.height);
    const CameraIntrinsics intrinsics{
        500.0 * static_cast<double>(options.width) / 320.0,
        500.0 * static_cast<double>(options.height) / 240.0,
        static_cast<double>(options.width) / 2.0,
        static_cast<double>(options.height) / 2.0,
    };
    const EulerAngles attitude{0.0, 0.0, 0.0};
    const Vector3 velocity{100.0, 0.0, 0.0};
    LineOfSightRateEstimator rate_estimator;
    const int warmup_iterations = std::min(1000, options.iterations);

    for (int index = 0; index < warmup_iterations; ++index) {
        const VisionResult result = recognizer.process(frame);
        const LineOfSight line_of_sight =
            GuidanceEstimator::calculate(result, intrinsics);
        rate_estimator.update(line_of_sight, 0.001);
    }

    std::vector<double> durations_us;
    durations_us.reserve(static_cast<std::size_t>(options.iterations));
    int valid_commands = 0;
    for (int index = 0; index < options.iterations; ++index) {
        if (options.full_scan) {
            recognizer.reset();
        }
        const auto start = std::chrono::steady_clock::now();
        const VisionResult result = recognizer.process(frame);
        const LineOfSight line_of_sight =
            GuidanceEstimator::calculate(result, intrinsics);
        const LineOfSightAngularVelocity rate =
            rate_estimator.update(line_of_sight, 0.001);
        const PngGuidanceOutput output =
            PngGuidance::calculate(line_of_sight, rate, velocity, attitude);
        const auto finish = std::chrono::steady_clock::now();
        durations_us.push_back(static_cast<double>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(finish - start)
                .count()) /
                               1000.0);
        if (result.found && output.valid) {
            ++valid_commands;
        }
    }

    const double total_us =
        std::accumulate(durations_us.begin(), durations_us.end(), 0.0);
    const double mean_us = total_us / static_cast<double>(durations_us.size());
    const double min_us = *std::min_element(durations_us.begin(), durations_us.end());
    const double max_us = *std::max_element(durations_us.begin(), durations_us.end());
    std::cout << std::fixed << std::setprecision(3)
              << "mode=" << (options.full_scan ? "full_scan" : "roi")
              << " resolution=" << options.width << "x" << options.height
              << " iterations=" << options.iterations
              << " valid_commands=" << valid_commands
              << " mean_us=" << mean_us
              << " median_us=" << percentile(durations_us, 0.50)
              << " p95_us=" << percentile(durations_us, 0.95)
              << " min_us=" << min_us
              << " max_us=" << max_us << '\n';
    return valid_commands == options.iterations ? 0 : 1;
}

}  // namespace

int main(int argc, char** argv) {
    Options options;
    if (!parse_options(argc, argv, &options)) {
        print_usage(argv[0]);
        return 2;
    }
    if (options.show_help) {
        print_usage(argv[0]);
        return 0;
    }
    return run_benchmark(options);
}
