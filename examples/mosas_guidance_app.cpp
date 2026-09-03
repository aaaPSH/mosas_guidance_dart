#include <mosas/app/app_config.hpp>
#include <mosas/app/host_adapters.hpp>
#include <mosas/runtime/guidance_runtime.hpp>

#if defined(MOSAS_APP_HAS_WIRELESS)
#include <mosas/wireless/rtsp_frame_sink.hpp>
#endif

#include <chrono>
#include <csignal>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

namespace {

volatile std::sig_atomic_t g_stop_requested = 0;

void handle_signal(int) {
    g_stop_requested = 1;
}

struct CommandLineOptions {
    std::string config_path;
    bool show_help = false;
};

void print_usage(const char* program) {
    std::cout << "用法: " << program << " --config PATH\n"
              << "  --config PATH  统一配置文件路径\n"
              << "  --help         显示帮助\n";
}

bool parse_options(int argc, char** argv, CommandLineOptions* options) {
    for (int index = 1; index < argc; ++index) {
        const std::string argument(argv[index]);
        if (argument == "--help" || argument == "-h") {
            options->show_help = true;
            continue;
        }
        if (argument == "--config") {
            if (index + 1 >= argc) {
                std::cerr << "参数缺少值: --config\n";
                return false;
            }
            options->config_path = argv[++index];
            continue;
        }
        std::cerr << "未知参数: " << argument << '\n';
        return false;
    }
    return options->show_help || !options->config_path.empty();
}

}  // namespace

int main(int argc, char** argv) {
    CommandLineOptions options;
    if (!parse_options(argc, argv, &options)) {
        print_usage(argv[0]);
        return 2;
    }
    if (options.show_help) {
        print_usage(argv[0]);
        return 0;
    }

    mosas::app::AppConfig config;
    std::string error;
    if (!mosas::app::AppConfigLoader::load(options.config_path, &config, &error)) {
        std::cerr << "加载配置失败: " << error << '\n';
        return 2;
    }

    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    auto imu_source = std::make_unique<mosas::app::SimulatedImuSource>(
        config.imu.sample_rate_hz);
    auto camera_source = std::make_unique<mosas::app::OpenCvCameraSource>(
        config.camera);
    auto command_sink = std::make_unique<mosas::app::LoggingCommandSink>();

    std::unique_ptr<mosas::runtime::FrameSink> frame_sink;
#if defined(MOSAS_APP_HAS_WIRELESS)
    if (config.wireless.enable_wireless_stream ||
        config.wireless.enable_recording) {
        frame_sink = std::make_unique<mosas::wireless::RtspFrameSink>(
            config.wireless,
            cv::Point2d(config.guidance.camera_intrinsics.cx,
                        config.guidance.camera_intrinsics.cy));
    }
#else
    if (config.wireless.enable_wireless_stream ||
        config.wireless.enable_recording) {
        std::cerr << "当前构建未启用无线图传模块\n";
        return 2;
    }
#endif

    mosas::runtime::GuidanceRuntime runtime(
        std::move(imu_source), std::move(camera_source),
        std::move(command_sink), config.guidance, std::move(frame_sink));
    if (!runtime.start()) {
        std::cerr << "启动制导程序失败: " << runtime.last_error() << '\n';
        return 1;
    }

    std::cout << "统一制导程序已启动\n";
    while (g_stop_requested == 0 && !runtime.faulted()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    const bool faulted = runtime.faulted();
    if (faulted) {
        std::cerr << "制导程序发生故障: " << runtime.last_error() << '\n';
    }
    runtime.stop();
    return faulted ? 1 : 0;
}
