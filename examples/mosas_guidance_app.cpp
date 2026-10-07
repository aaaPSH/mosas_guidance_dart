#include <mosas/app/app_config.hpp>
#include <mosas/app/host_adapters.hpp>
#include <mosas/app/nori_camera_source.hpp>
#include <mosas/app/serial_imu_source.hpp>
#include <mosas/app/default_guidance_algorithms.hpp>
#include <mosas/output/output_controller.hpp>
#include <mosas/runtime/runtime_supervisor.hpp>

#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <utility>

namespace {

volatile std::sig_atomic_t g_stop_requested = 0;

void handle_signal(int) {
    g_stop_requested = 1;
}

struct CommandLineOptions {
    std::string config_path;
    bool show_help = false;
    bool show_timing = false;
};

void print_usage(const char* program) {
    std::cout << "用法: " << program << " --config PATH\n"
              << "  --config PATH  统一配置文件路径\n"
              << "  --timing       退出时输出各流水线阶段耗时\n"
              << "  --help         显示帮助\n";
}

void print_timing_stage(
    const char* name,
    const mosas::runtime::RuntimeTimingStage& stage) {
    std::ostringstream output;
    output << std::fixed << std::setprecision(3)
           << "[TIMING] " << name << " n=" << stage.samples
           << " min_ms=" << stage.minimum_ms
           << " avg_ms=" << stage.average_ms
           << " p50_ms=" << stage.p50_ms
           << " p95_ms=" << stage.p95_ms
           << " p99_ms=" << stage.p99_ms
           << " max_ms=" << stage.maximum_ms;
    std::cout << output.str() << '\n';
}

void print_timing_stage_ns(
    const char* name,
    const mosas::runtime::TimingDistributionSummary& stage) {
    constexpr double kNanosecondsPerMillisecond = 1e6;
    std::ostringstream output;
    output << std::fixed << std::setprecision(3)
           << "[TIMING] " << name << " n=" << stage.samples
           << " min_ms=" << stage.minimum_ns / kNanosecondsPerMillisecond
           << " avg_ms=" << stage.average_ns / kNanosecondsPerMillisecond
           << " p50_ms=" << stage.p50_ns / kNanosecondsPerMillisecond
           << " p95_ms=" << stage.p95_ns / kNanosecondsPerMillisecond
           << " p99_ms=" << stage.p99_ns / kNanosecondsPerMillisecond
           << " max_ms=" << stage.maximum_ns / kNanosecondsPerMillisecond;
    std::cout << output.str() << '\n';
}

void print_timing(const mosas::runtime::RuntimeTiming& timing) {
    std::cout << "[TIMING] P50/P95/P99 为对数直方图估值，分位桶最大相对误差约 0.8%\n";
    print_timing_stage("capture", timing.capture);
    print_timing_stage("capture_queue_wait", timing.capture_queue_wait);
    print_timing_stage("capture_queue_delay", timing.capture_queue_delay);
    print_timing_stage("prepare", timing.prepare);
    print_timing_stage("vision", timing.vision);
    print_timing_stage("line_of_sight", timing.line_of_sight);
    print_timing_stage("guidance", timing.guidance);
    print_timing_stage("command", timing.command);
    print_timing_stage("processing_total", timing.processing_total);
    print_timing_stage("output_queue_wait", timing.output_queue_wait);
    print_timing_stage("output_queue_delay", timing.output_queue_delay);
    print_timing_stage("output", timing.output);
    std::cout << "[QUEUE] runtime_capture_dropped="
              << timing.capture_queue_dropped
              << " runtime_output_dropped=" << timing.output_queue_dropped
              << '\n';
}

void print_camera_timing(const mosas::app::NoriCameraTiming& timing) {
    print_timing_stage_ns("camera_callback_wait", timing.callback_wait);
    print_timing_stage_ns("camera_callback_queue_delay",
                          timing.callback_queue_delay);
    print_timing_stage_ns("camera_decode_convert", timing.decode_convert);
    std::cout << "[QUEUE] sdk_callback_dropped="
              << timing.callback_queue_dropped
              << " sdk_timestamp_dropped=" << timing.timestamp_dropped
              << '\n';
}

void print_serial_imu_interval_statistics(
    const mosas::app::SerialImuIntervalStatistics& statistics) {
    std::ostringstream output;
    output << "[串口统计] IMU 数据帧间隔: count="
           << statistics.interval_count;
    if (statistics.interval_count == 0) {
        std::cout << output.str() << '\n';
        return;
    }
    output << std::fixed << std::setprecision(3)
           << " avg_ms="
           << static_cast<double>(statistics.total_interval_ns) /
                  static_cast<double>(statistics.interval_count) / 1e6
           << " min_ms="
           << static_cast<double>(statistics.minimum_interval_ns) / 1e6
           << " max_ms="
           << static_cast<double>(statistics.maximum_interval_ns) / 1e6;
    std::cout << output.str() << '\n';
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
        if (argument == "--timing") {
            options->show_timing = true;
            continue;
        }
        std::cerr << "未知参数: " << argument << '\n';
        return false;
    }
    return options->show_help || !options->config_path.empty();
}

}  // 匿名命名空间结束

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
#if !defined(MOSAS_APP_HAS_WIRELESS)
    if (config.output.mode == mosas::output::OutputMode::rtsp) {
        std::cerr << "当前构建未启用无线图传，output.mode 不能设为 rtsp\n";
        return 2;
    }
#endif
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    std::shared_ptr<serial_package::SerialPort> serial_port;
    const bool serial_required =
        config.imu.mode == mosas::app::ImuMode::serial ||
        config.command.mode == mosas::app::CommandMode::serial;
    if (serial_required) {
        serial_port = std::make_shared<serial_package::SerialPort>();
        if (!serial_port->open(config.serial)) {
            std::cerr << "打开共享下位机串口失败: " << serial_port->last_error()
                      << '\n';
            return 1;
        }
    }

    std::unique_ptr<mosas::runtime::ImuSource> imu_source;
    mosas::app::SerialImuSource* serial_imu_source = nullptr;
    if (config.imu.mode == mosas::app::ImuMode::simulated) {
        imu_source = std::make_unique<mosas::app::SimulatedImuSource>(
            config.imu.sample_rate_hz, config.algorithms.attitude.gravity_mps2,
            config.algorithms.attitude.accelerometer_specific_force);
    } else if (config.imu.mode == mosas::app::ImuMode::serial) {
        auto source = std::make_unique<mosas::app::SerialImuSource>(
            serial_port, config.enable_serial_interval_statistics,
            config.imu.measurement_latency_ns, config.imu.receive_thread);
        serial_imu_source = source.get();
        imu_source = std::move(source);
    } else {
        std::cerr << "未知的 IMU 数据源模式\n";
        return 2;
    }
    auto camera_source = std::make_unique<mosas::app::NoriSdkCameraSource>(
        config.camera);
    auto* camera_source_status = camera_source.get();
    std::unique_ptr<mosas::runtime::CommandSink> command_sink;
    if (config.command.mode == mosas::app::CommandMode::log) {
        command_sink = std::make_unique<mosas::app::LoggingCommandSink>();
    } else if (config.command.mode == mosas::app::CommandMode::serial) {
        command_sink = std::make_unique<mosas::app::SerialCommandSink>(
            serial_port);
    } else {
        std::cerr << "未知的指令输出模式\n";
        return 2;
    }

    mosas::output::OutputControllerConfig output_config;
    output_config.output = config.output;
    output_config.wireless = config.wireless;
    output_config.camera_width = config.camera.width;
    output_config.camera_height = config.camera.height;
    output_config.camera_fps = config.camera.fps;
    output_config.line_of_sight_reference_point = cv::Point2d(
        config.guidance.camera_intrinsics.cx,
        config.guidance.camera_intrinsics.cy);
    std::unique_ptr<mosas::runtime::FrameSink> frame_sink =
        std::make_unique<mosas::output::OutputController>(
            std::move(output_config));

    auto algorithm_dependencies =
        mosas::app::make_default_guidance_algorithms(config.algorithms);
    mosas::runtime::RuntimeSupervisor runtime(
        std::move(imu_source), std::move(camera_source),
        std::move(command_sink), config.guidance,
        std::move(algorithm_dependencies), std::move(frame_sink));
    if (!runtime.start()) {
        std::cerr << "启动制导程序失败: " << runtime.last_error() << '\n';
        return 1;
    }

    std::cout << "统一制导程序已启动\n";
    const auto camera_configuration = camera_source_status->sdk_configuration();
    std::ostringstream camera_configuration_output;
    camera_configuration_output
        << "[CAMERA] SDK启动读回: " << camera_configuration.width << "x"
        << camera_configuration.height << " @ " << std::fixed
        << std::setprecision(2) << camera_configuration.fps
        << " FPS, 曝光=" << camera_configuration.exposure_us
        << " us, 增益=" << camera_configuration.gain << "倍";
    std::cout << camera_configuration_output.str() << '\n';
    auto next_statistics_at = std::chrono::steady_clock::now() +
                              std::chrono::seconds(1);
    while (g_stop_requested == 0 && !runtime.faulted()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        const auto now = std::chrono::steady_clock::now();
        if (now >= next_statistics_at) {
            const auto statistics = runtime.statistics();
            std::cout << "[FPS] capture=" << statistics.capture_fps
                      << " processing=" << statistics.processing_fps
                      << " output=" << statistics.output_fps << '\n';
            next_statistics_at = now + std::chrono::seconds(1);
        }
    }
    const bool faulted = runtime.faulted();
    if (faulted) {
        std::cerr << "制导程序发生故障: " << runtime.last_error() << '\n';
    }
    runtime.stop();
    if (serial_imu_source != nullptr &&
        config.enable_serial_interval_statistics) {
        print_serial_imu_interval_statistics(
            serial_imu_source->interval_statistics());
    }
    if (options.show_timing) {
        print_timing(runtime.timing());
        print_camera_timing(camera_source_status->camera_timing());
    }
    return faulted ? 1 : 0;
}
