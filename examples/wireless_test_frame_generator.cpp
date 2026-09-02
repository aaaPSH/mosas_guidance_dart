#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace {

struct Options {
    std::string template_path = "assets/wireless_test_frames/test_chart_template.png";
    std::string output_dir = "assets/wireless_test_frames/sequence";
    int width = 640;
    int height = 480;
    int count = 8;
    bool template_explicit = false;
};

void print_usage(const char* program) {
    std::cout << "用法: " << program << " [选项]\n"
              << "  --template PATH     测试图模板\n"
              << "  --output-dir PATH   输出图片目录\n"
              << "  --width N           图片宽度，默认 640\n"
              << "  --height N          图片高度，默认 480\n"
              << "  --count N           图片数量，默认 8\n"
              << "  --help              显示帮助\n";
}

bool parse_positive_int(const char* text, int* value) {
    char* end = nullptr;
    const long parsed = std::strtol(text, &end, 10);
    if (end == text || *end != '\0' || parsed <= 0 || parsed > 10000) {
        return false;
    }
    *value = static_cast<int>(parsed);
    return true;
}

bool read_value(int* index, int argc, char** argv, std::string* value) {
    if (*index + 1 >= argc) {
        std::cerr << "参数缺少值: " << argv[*index] << '\n';
        return false;
    }
    *value = argv[++(*index)];
    return true;
}

bool parse_options(int argc, char** argv, Options* options) {
    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];
        std::string value;
        if (argument == "--help") {
            print_usage(argv[0]);
            return false;
        }
        if (argument == "--template" && read_value(&index, argc, argv, &value)) {
            options->template_path = value;
            options->template_explicit = true;
        } else if (argument == "--output-dir" &&
                   read_value(&index, argc, argv, &value)) {
            options->output_dir = value;
        } else if ((argument == "--width" || argument == "--height" ||
                    argument == "--count") &&
                   read_value(&index, argc, argv, &value)) {
            int* destination = argument == "--width"
                                   ? &options->width
                                   : argument == "--height" ? &options->height
                                                              : &options->count;
            if (!parse_positive_int(value.c_str(), destination)) {
                std::cerr << "无效数值: " << argument << " " << value << '\n';
                return false;
            }
        } else {
            std::cerr << "未知参数: " << argument << '\n';
            print_usage(argv[0]);
            return false;
        }
    }
    return true;
}

cv::Mat make_default_template() {
    cv::Mat frame(480, 640, CV_8UC3, cv::Scalar(24, 32, 40));
    for (int x = 0; x < frame.cols; x += 40) {
        cv::line(frame, cv::Point(x, 0), cv::Point(x, frame.rows - 1),
                 cv::Scalar(48, 64, 72), 1);
    }
    for (int y = 0; y < frame.rows; y += 40) {
        cv::line(frame, cv::Point(0, y), cv::Point(frame.cols - 1, y),
                 cv::Scalar(48, 64, 72), 1);
    }
    cv::rectangle(frame, cv::Rect(8, 8, frame.cols - 16, frame.rows - 16),
                  cv::Scalar(96, 128, 144), 2);
    cv::putText(frame, "MOSAS WIRELESS TEST", cv::Point(20, 36),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(220, 240, 240), 2,
                cv::LINE_AA);
    return frame;
}

cv::Mat make_frame(const cv::Mat& template_frame, int width, int height,
                   int index, int count) {
    cv::Mat frame;
    cv::resize(template_frame, frame, cv::Size(width, height), 0.0, 0.0,
               cv::INTER_AREA);

    const int radius = std::max(8, std::min(width, height) / 24);
    const int left = radius + 4;
    const int right = std::max(left + 1, width - radius - 4);
    const int x = left + ((right - left) * index) / std::max(1, count - 1);
    const int y = std::max(radius + 4, height / 2);
    cv::circle(frame, cv::Point(x, y), radius, cv::Scalar(0, 255, 255),
               cv::FILLED);

    std::ostringstream label;
    label << "FRAME " << std::setw(02) << std::setfill('0') << index;
    cv::putText(frame, label.str(), cv::Point(16, height - 18),
                cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(255, 255, 255), 2,
                cv::LINE_AA);
    return frame;
}

}  // namespace

int main(int argc, char** argv) {
    Options options;
    if (!parse_options(argc, argv, &options)) {
        return argc > 1 && std::string(argv[1]) == "--help" ? 0 : 1;
    }

    cv::Mat template_frame =
        cv::imread(options.template_path, cv::IMREAD_COLOR);
    if (template_frame.empty()) {
        if (options.template_explicit) {
            std::cerr << "读取模板图片失败: " << options.template_path << '\n';
            return 1;
        }
        std::cerr << "未找到默认模板图片，使用内置测试底图\n";
        template_frame = make_default_template();
    }
    std::error_code filesystem_error;
    std::filesystem::create_directories(options.output_dir, filesystem_error);
    if (filesystem_error) {
        std::cerr << "创建输出目录失败: " << filesystem_error.message() << '\n';
        return 1;
    }

    for (int index = 0; index < options.count; ++index) {
        const cv::Mat frame = make_frame(template_frame, options.width,
                                         options.height, index, options.count);
        std::ostringstream filename;
        filename << options.output_dir << "/frame_" << std::setw(03)
                 << std::setfill('0') << index << ".png";
        if (!cv::imwrite(filename.str(), frame)) {
            std::cerr << "写入测试图片失败: " << filename.str() << '\n';
            return 1;
        }
    }
    std::cout << "已生成 " << options.count << " 张测试图片到 "
              << options.output_dir << '\n';
    return 0;
}
