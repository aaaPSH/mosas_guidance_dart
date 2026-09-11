#include <mosas/app/app_config.hpp>
#include <mosas/app/host_adapters.hpp>
#include <mosas/app/nori_camera_source.hpp>
#include <mosas/app/serial_imu_source.hpp>
#include <mosas/runtime/guidance_runtime.hpp>

#if defined(MOSAS_APP_HAS_WIRELESS)
#include <mosas/wireless/rtsp_frame_sink.hpp>
#endif

#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
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
    const mosas::runtime::GuidanceRuntimeTimingStage& stage) {
    std::cout << "[TIMING] " << name << " count=" << stage.samples
              << " avg_ms=" << stage.average_ms
              << " p95_ms=" << stage.p95_ms
              << " p99_ms=" << stage.p99_ms
              << " max_ms=" << stage.maximum_ms
              << " deadline_miss_count=" << stage.deadline_miss_count
              << '\n';
}

void print_timing(const mosas::runtime::GuidanceRuntimeTiming& timing) {
    print_timing_stage("capture", timing.capture);
    print_timing_stage("prepare", timing.prepare);
    print_timing_stage("vision", timing.vision);
    print_timing_stage("line_of_sight", timing.line_of_sight);
    print_timing_stage("guidance", timing.guidance);
    print_timing_stage("command", timing.command);
    print_timing_stage("processing_total", timing.processing_total);
    print_timing_stage("output", timing.output);
}

void print_runtime_observability(
    const mosas::runtime::GuidanceRuntimeStatistics& statistics,
    const mosas::runtime::GuidanceRuntimeControlStatus& control_status) {
    std::cout << "[RUNTIME] processing_queue_depth="
              << statistics.processing_queue_depth
              << " processing_queue_high_water_mark="
              << statistics.processing_queue_high_water_mark
              << " frame_drop_count=" << statistics.frame_drop_count
              << " control_generated="
              << statistics.control_command_generated_count
              << " control_sent=" << statistics.control_command_sent_count
              << " control_superseded="
              << statistics.control_command_superseded_count
              << " control_stale=" << statistics.stale_command_count
              << " send_failure_count=" << statistics.send_failure_count
              << '\n';
    std::cout << "[CONTROL] state="
              << mosas::runtime::control_watchdog_state_name(
                     control_status.state)
              << " last_generated_sequence="
              << control_status.last_generated_sequence
              << " last_sent_sequence=" << control_status.last_sent_sequence
              << " last_generated_time_ns="
              << control_status.last_generated_time_ns
              << " last_sent_time_ns=" << control_status.last_sent_time_ns
              << " consecutive_send_failures="
              << control_status.consecutive_send_failures
              << " consecutive_stale_commands="
              << control_status.consecutive_stale_commands
              << " last_stale_sequence=" << control_status.last_stale_sequence
              << " last_stale_age_ns=" << control_status.last_stale_age_ns
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
    config.guidance.skip_imu_self_check = config.imu.skip_self_check;

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
            config.imu.sample_rate_hz);
    } else if (config.imu.mode == mosas::app::ImuMode::serial) {
        auto source = std::make_unique<mosas::app::SerialImuSource>(
            serial_port, config.imu.sample_rate_hz,
            config.serial.read_timeout_ms,
            config.enable_serial_interval_statistics);
        serial_imu_source = source.get();
        imu_source = std::move(source);
    } else {
        std::cerr << "未知的 IMU 数据源模式\n";
        return 2;
    }
    auto camera_source = std::make_unique<mosas::app::NoriSdkCameraSource>(
        config.camera);
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
        print_runtime_observability(runtime.statistics(),
                                     runtime.control_status());
    }
    return faulted ? 1 : 0;
}
