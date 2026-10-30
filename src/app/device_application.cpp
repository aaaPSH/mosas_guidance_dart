#include <mosas/app/device_application.hpp>
#include <mosas/app/acquisition_time_report.hpp>
#include <cstdio>

#include <mosas/adapters/host_adapters.hpp>
#include <mosas/app/default_guidance_algorithms.hpp>
#include <mosas/app/output_factory.hpp>

#include <csignal>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <utility>

namespace mosas::app {
namespace {

volatile std::sig_atomic_t g_stop_requested = 0;

void handle_signal(int) {
    g_stop_requested = 1;
}

void print_timing_stage(const char* name, const runtime::RuntimeTimingStage& stage) {
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

void print_timing_stage_ns(const char* name, const runtime::TimingDistributionSummary& stage) {
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

/** @brief 输出原运行时的耗时指标。 @param timing 停止生产者后取得的统计快照。 */
void print_timing(const runtime::RuntimeTiming& timing) {
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
    std::cout << "[QUEUE] runtime_capture_dropped=" << timing.capture_queue_dropped
              << " runtime_output_dropped=" << timing.output_queue_dropped << '\n';
}

void print_sink_diagnostics(const char* name, const runtime::SinkDiagnostics& diagnostics) {
    std::cout << ' ' << name << "{enabled="
              << (diagnostics.enabled ? "yes" : "no")
              << " healthy=" << (diagnostics.healthy ? "yes" : "no")
              << " accepted=" << diagnostics.accepted_count
              << " dropped=" << diagnostics.dropped_count
              << " failures=" << diagnostics.failure_count
              << " queue=" << diagnostics.queue_depth << '/' << diagnostics.queue_capacity;
    if (!diagnostics.last_error.empty()) {
        std::cout << " error=\"" << diagnostics.last_error << '\"';
    }
    std::cout << '}';
}

void print_output_diagnostics(const runtime::RuntimeOutputDiagnostics& diagnostics) {
    std::cout << "[OUTPUT]";
    print_sink_diagnostics("video", diagnostics.video);
    print_sink_diagnostics("telemetry", diagnostics.telemetry);
    print_sink_diagnostics("visualization", diagnostics.visualization);
    print_sink_diagnostics("imu", diagnostics.imu_recording);
    print_sink_diagnostics("logging", diagnostics.logging);
    std::cout << '\n';
}

void print_camera_timing(const NoriCameraTiming& timing) {
    print_timing_stage_ns("camera_callback_wait", timing.callback_wait);
    print_timing_stage_ns("camera_callback_queue_delay", timing.callback_queue_delay);
    print_timing_stage_ns("camera_decode_convert", timing.decode_convert);
    std::cout << "[QUEUE] sdk_callback_dropped=" << timing.callback_queue_dropped
              << " sdk_timestamp_dropped=" << timing.timestamp_dropped << '\n';
}

void print_serial_imu_interval_statistics(const SerialImuIntervalStatistics& statistics) {
    std::ostringstream output;
    output << "[串口统计] IMU 数据帧间隔: count=" << statistics.interval_count;
    if (statistics.interval_count == 0) {
        std::cout << output.str() << '\n';
        return;
    }
    output << std::fixed << std::setprecision(3)
           << " avg_ms=" << static_cast<double>(statistics.total_interval_ns) /
                               static_cast<double>(statistics.interval_count) / 1e6
           << " min_ms=" << static_cast<double>(statistics.minimum_interval_ns) / 1e6
           << " max_ms=" << static_cast<double>(statistics.maximum_interval_ns) / 1e6;
    std::cout << output.str() << '\n';
}

}  // namespace

int load_device_configuration(const std::string& path, AppConfig& config) {
    std::string error;
    if (!AppConfigLoader::load(path, &config, &error)) {
        std::cerr << "加载配置失败: " << error << '\n';
        return 2;
    }
#if !defined(MOSAS_OUTPUT_HAS_WIRELESS)
    if (config.output.mode == output::OutputMode::rtsp) {
        std::cerr << "当前构建未启用无线图传，output.mode 不能设为 rtsp\n";
        return 2;
    }
#endif
    return 0;
}

int load_default_device_configuration(const std::string& filename, AppConfig& config) {
    namespace fs = std::filesystem;
    const fs::path name(filename);
    if (name.empty() || name.has_parent_path() || name == "." || name == "..") {
        std::cerr << "默认配置必须使用不含目录的文件名\n";
        return 2;
    }
    std::error_code error;
    const auto executable = fs::read_symlink("/proc/self/exe", error);
    if (!error) {
        const auto directory = executable.parent_path();
        const auto installed = (directory / MOSAS_DEVICE_INSTALL_CONFIG_DIRECTORY).lexically_normal();
        // 安装脚本与二进制同目录；即使配置目录被删除，也不能误用当前目录的另一份配置。
        const bool installed_layout = fs::is_directory(installed, error) ||
            fs::is_regular_file(directory / "run_guidance.sh", error) ||
            fs::is_regular_file(directory / "run_sync_calibration.sh", error);
        if (installed_layout) {
            return load_device_configuration((installed / name).string(), config);
        }
    }
    return load_device_configuration((fs::path("config") / name).string(), config);
}

DeviceApplicationComponents::~DeviceApplicationComponents() noexcept {
    try {
        if (runtime) runtime->stop();
        if (imu_source) imu_source->stop();
        if (camera_source) camera_source->stop();
        report_acquisition_time();
    } catch (...) {
        std::fputs("[ACQUISITION_TIME] shutdown_failed; final timing report unavailable\n", stderr);
    }
}

void DeviceApplicationComponents::report_acquisition_time() noexcept {
    if (acquisition_time_reported) return;
    acquisition_time_reported = true;
    try {
        std::ostringstream report;
        // A failed ownership-transfer constructor may already have destroyed the sources.
        if (camera_source_status && (runtime || camera_source)) {
            write_acquisition_time_report(report, "camera", camera_source_status->time_diagnostics());
            const auto camera = camera_source_status->camera_timing();
            report << "[ACQUISITION_TIME] camera_queue_dropped=" << camera.callback_queue_dropped
                   << " camera_timestamp_dropped=" << camera.timestamp_dropped << '\n';
        } else report << "[ACQUISITION_TIME] source=camera unavailable=not_initialized_or_construction_failed\n";
        if (serial_imu_source && (runtime || imu_source)) {
            const auto imu = serial_imu_source->diagnostics();
            write_acquisition_time_report(report, "imu", imu.timing);
            report << "[ACQUISITION_TIME] imu_crc_errors=" << imu.crc_error_count
                   << " parse_errors=" << imu.parse_error_count << " raw_chunk_gaps=" << imu.sequence_gap_count
                   << " raw_queue_dropped=" << imu.dropped_chunk_count
                   << " sample_queue_dropped=" << imu.dropped_sample_count << '\n';
        } else report << "[ACQUISITION_TIME] source=imu unavailable=not_serial_or_not_initialized\n";
        const auto text = report.str();
        std::cerr << text << std::flush;
        if (!std::cerr) std::fwrite(text.data(), 1, text.size(), stderr);
    } catch (...) {
        std::fputs("[ACQUISITION_TIME] report_failed; timing quality unknown\n", stderr);
    }
}

void install_device_signal_handlers() {
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);
}

bool device_stop_requested() noexcept {
    return g_stop_requested != 0;
}

int initialize_device_sources(const AppConfig& config, DeviceApplicationComponents& components) {
    const bool serial_required = config.imu.mode == ImuMode::serial ||
                                 config.command.mode == CommandMode::serial;
    if (serial_required) {
        components.serial_port = std::make_shared<serial_package::SerialPort>();
        if (!components.serial_port->open(config.serial)) {
            std::cerr << "打开共享下位机串口失败: " << components.serial_port->last_error() << '\n';
            return 1;
        }
    }
    if (config.imu.mode == ImuMode::simulated) {
        components.imu_source = std::make_unique<SimulatedImuSource>(
            config.imu.timing.frequency_hz, config.algorithms.attitude.gravity_mps2,
            config.algorithms.attitude.accelerometer_specific_force);
    } else if (config.imu.mode == ImuMode::serial) {
        auto source = std::make_unique<SerialImuSource>(
            components.serial_port, config.enable_serial_interval_statistics,
            config.imu.measurement_latency_ns, config.imu.receive_thread, config.imu.timing);
        components.serial_imu_source = source.get();
        components.imu_source = std::move(source);
    } else {
        std::cerr << "未知的 IMU 数据源模式\n";
        return 2;
    }
    auto camera = std::make_unique<NoriSdkCameraSource>(config.camera);
    components.camera_source_status = camera.get();
    components.camera_source = std::move(camera);
    return 0;
}

int start_device_runtime(const AppConfig& config, DeviceApplicationComponents& components,
                         std::unique_ptr<runtime::CommandSink> command_sink) {
    auto algorithms = make_default_guidance_algorithms(config.algorithms);
    auto outputs = make_runtime_output_ports(config, std::cerr);
    components.runtime = std::make_unique<runtime::RuntimeSupervisor>(
        std::move(components.imu_source), std::move(components.camera_source),
        std::move(command_sink), config.guidance, std::move(algorithms), std::move(outputs));
    if (!components.runtime->start()) {
        std::cerr << "启动制导程序失败: " << components.runtime->last_error() << '\n';
        return 1;
    }
    return 0;
}

void print_device_startup(const DeviceApplicationComponents& components) {
    std::cout << "统一制导程序已启动\n";
    const auto camera = components.camera_source_status->sdk_configuration();
    std::ostringstream output;
    output << "[CAMERA] SDK启动读回: " << camera.width << "x" << camera.height
           << " @ " << std::fixed << std::setprecision(2) << camera.fps
           << " FPS, 曝光=" << camera.exposure_us << " us, 增益=" << camera.gain << "倍";
    std::cout << output.str() << '\n';
}

int stop_and_report_device_runtime(DeviceApplicationComponents& components,
                                   const AppConfig& config) {
    auto& runtime = *components.runtime;
    runtime.stop();
    components.report_acquisition_time();
    const bool faulted = runtime.faulted();
    if (faulted) {
        std::cerr << "制导程序发生故障: " << runtime.last_error() << '\n';
    }
    print_output_diagnostics(runtime.output_diagnostics());
    if (components.serial_imu_source != nullptr && config.enable_serial_interval_statistics) {
        print_serial_imu_interval_statistics(components.serial_imu_source->interval_statistics());
    }
    if (config.enable_timing_statistics) {
        print_timing(runtime.timing());
        print_camera_timing(components.camera_source_status->camera_timing());
    }
    return faulted ? 1 : 0;
}

}  // namespace mosas::app
