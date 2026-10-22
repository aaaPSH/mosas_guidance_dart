#include <mosas/app/app_config.hpp>
#include <mosas/adapters/host_adapters.hpp>
#include <mosas/adapters/nori_camera_source.hpp>
#include <mosas/adapters/serial_imu_source.hpp>
#include <mosas/app/default_guidance_algorithms.hpp>
#include <mosas/app/output_factory.hpp>
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

void print_sink_diagnostics(
    const char* name, const mosas::runtime::SinkDiagnostics& diagnostics) {
    std::cout << ' ' << name << "{enabled="
              << (diagnostics.enabled ? "yes" : "no")
              << " healthy=" << (diagnostics.healthy ? "yes" : "no")
              << " accepted=" << diagnostics.accepted_count
              << " dropped=" << diagnostics.dropped_count
              << " failures=" << diagnostics.failure_count
              << " queue=" << diagnostics.queue_depth << '/'
              << diagnostics.queue_capacity;
    if (!diagnostics.last_error.empty()) {
        std::cout << " error=\"" << diagnostics.last_error << '\"';
    }
    std::cout << '}';
}

void print_output_diagnostics(
    const mosas::runtime::RuntimeOutputDiagnostics& diagnostics) {
    std::cout << "[OUTPUT]";
    print_sink_diagnostics("video", diagnostics.video);
    print_sink_diagnostics("telemetry", diagnostics.telemetry);
    print_sink_diagnostics("visualization", diagnostics.visualization);
    print_sink_diagnostics("imu", diagnostics.imu_recording);
    print_sink_diagnostics("logging", diagnostics.logging);
    std::cout << '\n';
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

/**
 * @brief 加载命令行参数与配置，不启动设备。
 * @param argc 参数数量。
 * @param argv 参数数组。
 * @param options 接收命令行选项。
 * @param config 接收应用配置。
 * @param should_run 接收是否继续启动的标志；帮助请求时为 false。
 * @return 成功或帮助请求返回 0，参数或配置错误返回 2。
 */
int load_configuration(int argc, char** argv, CommandLineOptions& options,
                       mosas::app::AppConfig& config, bool& should_run) {
    should_run = false;
    if (!parse_options(argc, argv, &options)) {
        print_usage(argv[0]);
        return 2;
    }
    if (options.show_help) {
        print_usage(argv[0]);
        return 0;
    }

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
    should_run = true;
    return 0;
}

/** @brief 持有运行组件；声明顺序确保运行时先于共享串口析构。 */
struct ApplicationComponents {
    std::shared_ptr<serial_package::SerialPort> serial_port;
    mosas::app::SerialImuSource* serial_imu_source = nullptr;
    mosas::app::NoriSdkCameraSource* camera_source_status = nullptr;
    std::unique_ptr<mosas::runtime::RuntimeSupervisor> runtime;
};

/**
 * @brief 初始化组件并启动运行时，在打开串口之前注册退出信号。
 * @param config 已校验的应用配置。
 * @param components 接收组件所有权以及由运行时持有的数据源观察指针。
 * @return 成功返回 0，设备或启动失败返回 1，模式错误返回 2。
 */
int initialize_components(const mosas::app::AppConfig& config,
                          ApplicationComponents& components) {
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    auto& serial_port = components.serial_port;
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
    if (config.imu.mode == mosas::app::ImuMode::simulated) {
        imu_source = std::make_unique<mosas::app::SimulatedImuSource>(
            config.imu.sample_rate_hz, config.algorithms.attitude.gravity_mps2,
            config.algorithms.attitude.accelerometer_specific_force);
    } else if (config.imu.mode == mosas::app::ImuMode::serial) {
        auto source = std::make_unique<mosas::app::SerialImuSource>(
            serial_port, config.enable_serial_interval_statistics,
            config.imu.measurement_latency_ns, config.imu.receive_thread);
        components.serial_imu_source = source.get();
        imu_source = std::move(source);
    } else {
        std::cerr << "未知的 IMU 数据源模式\n";
        return 2;
    }
    auto camera_source = std::make_unique<mosas::app::NoriSdkCameraSource>(
        config.camera);
    components.camera_source_status = camera_source.get();
    std::unique_ptr<mosas::runtime::CommandSink> command_sink;
    if (config.command.mode == mosas::app::CommandMode::log) {
        command_sink = std::make_unique<mosas::app::LoggingCommandSink>();
    } else if (config.command.mode == mosas::app::CommandMode::serial) {
        command_sink = std::make_unique<mosas::app::SerialCommandSink>(
            serial_port, config.command, std::cerr);
    } else {
        std::cerr << "未知的指令输出模式\n";
        return 2;
    }

    auto algorithm_dependencies =
        mosas::app::make_default_guidance_algorithms(config.algorithms);
    auto output_ports =
        mosas::app::make_runtime_output_ports(config, std::cerr);
    components.runtime = std::make_unique<mosas::runtime::RuntimeSupervisor>(
        std::move(imu_source), std::move(camera_source),
        std::move(command_sink), config.guidance,
        std::move(algorithm_dependencies), std::move(output_ports));
    auto& runtime = *components.runtime;
    if (!runtime.start()) {
        std::cerr << "启动制导程序失败: " << runtime.last_error() << '\n';
        return 1;
    }

    return 0;
}

/**
 * @brief 输出启动信息并监控运行状态，直到收到退出信号或发生故障。
 * @param components 已成功启动的组件，调用期间必须保持存活。
 */
void monitor_runtime(ApplicationComponents& components) {
    auto& runtime = *components.runtime;
    std::cout << "统一制导程序已启动\n";
    const auto camera_configuration =
        components.camera_source_status->sdk_configuration();
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
            print_output_diagnostics(runtime.output_diagnostics());
            next_statistics_at = now + std::chrono::seconds(1);
        }
    }
}

/**
 * @brief 按原顺序报告故障、停止运行时并输出退出统计。
 * @param components 已启动的组件，数据源在统计完成之前保持存活。
 * @param config 决定是否输出串口统计的配置。
 * @param options 决定是否输出耗时统计的命令行选项。
 * @return 故障退出返回 1，正常退出返回 0。
 */
int stop_and_report(ApplicationComponents& components,
                    const mosas::app::AppConfig& config,
                    const CommandLineOptions& options) {
    auto& runtime = *components.runtime;
    const auto* serial_imu_source = components.serial_imu_source;
    const auto* camera_source_status = components.camera_source_status;
    const bool faulted = runtime.faulted();
    if (faulted) {
        std::cerr << "制导程序发生故障: " << runtime.last_error() << '\n';
    }
    runtime.stop();
    print_output_diagnostics(runtime.output_diagnostics());
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

}  // 匿名命名空间结束

/**
 * @brief 串联配置加载、组件启动、运行监控和退出流程。
 * @param argc 参数数量。
 * @param argv 参数数组。
 * @return 正常退出或帮助请求返回 0，运行失败返回 1，参数配置错误返回 2。
 */
int main(int argc, char** argv) {
    CommandLineOptions options;
    mosas::app::AppConfig config;
    bool should_run = false;
    const int configuration_result =
        load_configuration(argc, argv, options, config, should_run);
    if (!should_run) {
        return configuration_result;
    }

    ApplicationComponents components;
    const int initialization_result = initialize_components(config, components);
    if (initialization_result != 0) {
        return initialization_result;
    }

    monitor_runtime(components);
    return stop_and_report(components, config, options);
}
