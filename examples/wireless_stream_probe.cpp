#include <mosas/wireless/rtsp_streamer.hpp>

#include <chrono>
#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

namespace {

struct Options {
    std::string device = "/dev/video0";
    std::string url = "rtsp://127.0.0.1:8554/mosas";
    std::string transport = "tcp";
    int width = 640;
    int height = 480;
    int fps = 30;
    int bitrate = 2'000'000;
    int frames = 0;
    std::string encoder;
    std::string image_dir;
    bool pattern = false;
};

void print_usage(const char* program) {
    std::cout
        << "用法: " << program << " [选项]\n"
        << "  --device PATH       V4L2 摄像头，默认 /dev/video0\n"
        << "  --url URL           RTSP 推流地址，默认 rtsp://127.0.0.1:8554/mosas\n"
        << "  --transport MODE    RTSP 传输方式：tcp 或 udp，默认 tcp\n"
        << "  --width N           宽度，默认 640\n"
        << "  --height N          高度，默认 480\n"
        << "  --fps N             帧率，默认 30\n"
        << "  --bitrate N         H.264 码率，默认 2000000\n"
        << "  --encoder NAME      编码器，如 libx264 或 h264_v4l2m2m\n"
        << "  --image-dir PATH    按顺序循环播放 PNG/JPEG 图片\n"
        << "  --frames N          发送 N 帧，0 表示持续发送\n"
        << "  --pattern            使用测试图案，不打开摄像头\n"
        << "  --help              显示帮助\n";
}

bool parse_int(const char* text, int* value) {
    char* end = nullptr;
    const long parsed = std::strtol(text, &end, 10);
    if (end == text || *end != '\0' || parsed < 0 || parsed > 2'000'000'000L) {
        return false;
    }
    *value = static_cast<int>(parsed);
    return true;
}

bool next_value(int index, int argc, char** argv, std::string* value) {
    if (index + 1 >= argc) {
        std::cerr << "参数缺少值: " << argv[index] << '\n';
        return false;
    }
    *value = argv[index + 1];
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
        if (argument == "--pattern") {
            options->pattern = true;
        } else if (argument == "--device" && next_value(index++, argc, argv, &value)) {
            options->device = value;
        } else if (argument == "--url" && next_value(index++, argc, argv, &value)) {
            options->url = value;
        } else if (argument == "--transport" &&
                   next_value(index++, argc, argv, &value)) {
            options->transport = value;
        } else if (argument == "--encoder" && next_value(index++, argc, argv, &value)) {
            options->encoder = value;
        } else if (argument == "--image-dir" &&
                   next_value(index++, argc, argv, &value)) {
            options->image_dir = value;
        } else if ((argument == "--width" || argument == "--height" ||
                    argument == "--fps" || argument == "--bitrate" ||
                    argument == "--frames") &&
                   next_value(index++, argc, argv, &value)) {
            int* destination = nullptr;
            if (argument == "--width") destination = &options->width;
            if (argument == "--height") destination = &options->height;
            if (argument == "--fps") destination = &options->fps;
            if (argument == "--bitrate") destination = &options->bitrate;
            if (argument == "--frames") destination = &options->frames;
            if (!parse_int(value.c_str(), destination)) {
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

cv::Mat make_test_pattern(int width, int height, int frame_index) {
    cv::Mat frame(height, width, CV_8UC3, cv::Scalar(24, 24, 24));
    const int box_width = std::max(20, width / 5);
    const int box_height = std::max(20, height / 5);
    const int max_x = std::max(1, width - box_width);
    const int x = (frame_index * 4) % max_x;
    const int y = height / 2 - box_height / 2;
    cv::rectangle(frame, cv::Rect(x, y, box_width, box_height),
                  cv::Scalar(0, 255, 0), cv::FILLED);
    return frame;
}

}  // namespace

int main(int argc, char** argv) {
    Options options;
    if (!parse_options(argc, argv, &options)) {
        return argc > 1 && std::string(argv[1]) == "--help" ? 0 : 1;
    }

    mosas::wireless::RtspStreamConfig config;
    config.rtsp_url = options.url;
    config.rtsp_transport = options.transport;
    config.width = options.width;
    config.height = options.height;
    config.fps = options.fps;
    config.bitrate = options.bitrate;
    config.encoder_name = options.encoder;

    mosas::wireless::RtspStreamer streamer(config);
    std::string error;
    if (!streamer.start(&error)) {
        std::cerr << "启动无线图传失败: " << error << '\n';
        return 1;
    }

    std::vector<std::filesystem::path> image_paths;
    if (!options.image_dir.empty()) {
        const std::filesystem::path image_dir(options.image_dir);
        if (!std::filesystem::is_directory(image_dir)) {
            std::cerr << "图片目录不存在: " << options.image_dir << '\n';
            streamer.stop();
            return 1;
        }
        for (const auto& entry : std::filesystem::directory_iterator(image_dir)) {
            if (!entry.is_regular_file()) {
                continue;
            }
            const std::string extension = entry.path().extension().string();
            if (extension == ".png" || extension == ".PNG" ||
                extension == ".jpg" || extension == ".JPG" ||
                extension == ".jpeg" || extension == ".JPEG") {
                image_paths.push_back(entry.path());
            }
        }
        std::sort(image_paths.begin(), image_paths.end());
        if (image_paths.empty()) {
            std::cerr << "图片目录中没有 PNG/JPEG 文件: " << options.image_dir
                      << '\n';
            streamer.stop();
            return 1;
        }
    }

    cv::VideoCapture camera;
    if (!options.pattern && options.image_dir.empty()) {
        if (!camera.open(options.device, cv::CAP_V4L2)) {
            std::cerr << "打开摄像头失败: " << options.device << '\n';
            streamer.stop();
            return 1;
        }
        camera.set(cv::CAP_PROP_FRAME_WIDTH, options.width);
        camera.set(cv::CAP_PROP_FRAME_HEIGHT, options.height);
        camera.set(cv::CAP_PROP_FPS, options.fps);
    }

    std::cout << "无线图传已启动: " << options.url
              << "，VLC 接收地址: " << options.url << '\n';
    std::cout << "编码器: "
              << (options.encoder.empty() ? "FFmpeg 默认 H.264" : options.encoder)
              << "，按 Ctrl+C 停止\n";

    const auto frame_period = std::chrono::microseconds(1'000'000 / options.fps);
    auto next_deadline = std::chrono::steady_clock::now();
    int frame_index = 0;
    bool failed = false;
    while (options.frames == 0 || frame_index < options.frames) {
        cv::Mat frame;
        if (options.pattern) {
            frame = make_test_pattern(options.width, options.height, frame_index);
        } else if (!image_paths.empty()) {
            const auto& image_path = image_paths[frame_index % image_paths.size()];
            frame = cv::imread(image_path.string(), cv::IMREAD_COLOR);
            if (frame.empty()) {
                std::cerr << "读取图片失败: " << image_path << '\n';
                failed = true;
                break;
            }
        } else if (!camera.read(frame)) {
            std::cerr << "摄像头读取失败\n";
            failed = true;
            break;
        }

        if (frame.empty()) {
            std::cerr << "摄像头返回空帧\n";
            failed = true;
            break;
        }
        if (frame.channels() == 1) {
            cv::cvtColor(frame, frame, cv::COLOR_GRAY2BGR);
        } else if (frame.channels() == 4) {
            cv::cvtColor(frame, frame, cv::COLOR_BGRA2BGR);
        }
        if (frame.cols != options.width || frame.rows != options.height) {
            cv::resize(frame, frame, cv::Size(options.width, options.height),
                       0.0, 0.0, cv::INTER_LINEAR);
        }

        if (!streamer.send_bgr(
                frame,
                std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
                    .count(),
                &error)) {
            std::cerr << "发送视频帧失败: " << error << '\n';
            failed = true;
            break;
        }
        ++frame_index;
        if ((frame_index % options.fps) == 0) {
            std::cout << "已发送 " << frame_index << " 帧\n";
        }
        if (options.pattern || !image_paths.empty()) {
            next_deadline += frame_period;
            std::this_thread::sleep_until(next_deadline);
        }
    }

    streamer.stop();
    std::cout << "无线图传已停止，共发送 " << frame_index << " 帧\n";
    return !failed && frame_index > 0 ? 0 : 1;
}
