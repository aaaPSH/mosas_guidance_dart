#include <mosas/app/camera_imu_sync_session.hpp>

#include "camera_imu_sync_queue.hpp"
#include <mosas/vision/opencv_image_frame.hpp>

#include <opencv2/imgproc.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>

namespace mosas::app {
namespace {
using Ns = runtime::TimestampNs;
using Clock = std::chrono::steady_clock;
namespace fs = std::filesystem;

/**
 * @brief 安全撤销固定时间补偿，拒绝负时间或溢出结果。
 * @param value 已补偿时间，单位 ns。
 * @param correction 待撤销的有符号补偿，单位 ns。
 * @param result 接收未补偿时间。
 * @return 结果非负且无溢出时为 true。
 */
bool subtract_time(Ns value, Ns correction, Ns& result) noexcept {
    if (value < 0 || (correction > 0 && value < correction) ||
        (correction < 0 && value > std::numeric_limits<Ns>::max() + correction)) return false;
    result = value - correction;
    return true;
}

bool finite_vector(const Vector3& value) noexcept {
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

bool add_latency(Ns value, Ns latency, Ns& result) noexcept {
    if (value < 0 || latency < 0 || value > std::numeric_limits<Ns>::max() - latency) return false;
    result = value + latency;
    return true;
}

const char* domain_name(runtime::CameraTimestampDomain domain) noexcept {
    return domain == runtime::CameraTimestampDomain::monotonic ? "monotonic" : "realtime";
}

const char* source_name(runtime::CameraTimestampSource source) noexcept {
    switch (source) {
        case runtime::CameraTimestampSource::sdk_monotonic: return "sdk_monotonic";
        case runtime::CameraTimestampSource::sdk_realtime_converted: return "sdk_realtime_converted";
        default: return "unknown";
    }
}

const char* end_name(CameraImuSyncEnd end) noexcept {
    switch (end) {
        case CameraImuSyncEnd::duration_elapsed: return "duration_elapsed";
        case CameraImuSyncEnd::interrupted: return "interrupted";
        case CameraImuSyncEnd::runtime_fault: return "runtime_fault";
        case CameraImuSyncEnd::startup_failure: return "startup_failure";
    }
    return "unknown";
}

void timing_json(std::ostream& output, const runtime::TimingDistributionSummary& timing) {
    output << "{\"samples\":" << timing.samples << ",\"min_ns\":" << timing.minimum_ns
           << ",\"mean_ns\":" << timing.average_ns << ",\"p50_ns\":" << timing.p50_ns
           << ",\"p95_ns\":" << timing.p95_ns << ",\"p99_ns\":" << timing.p99_ns
           << ",\"max_ns\":" << timing.maximum_ns << '}';
}

void runtime_json(std::ostream& output, const runtime::RuntimeTiming& timing) {
    using Stage = runtime::RuntimeTimingStage;
    const std::pair<const char*, Stage runtime::RuntimeTiming::*> stages[] = {
        {"capture", &runtime::RuntimeTiming::capture},
        {"capture_queue_wait", &runtime::RuntimeTiming::capture_queue_wait},
        {"capture_queue_delay", &runtime::RuntimeTiming::capture_queue_delay},
        {"prepare", &runtime::RuntimeTiming::prepare},
        {"vision", &runtime::RuntimeTiming::vision},
        {"line_of_sight", &runtime::RuntimeTiming::line_of_sight},
        {"guidance", &runtime::RuntimeTiming::guidance},
        {"command", &runtime::RuntimeTiming::command},
        {"processing_total", &runtime::RuntimeTiming::processing_total},
        {"output_queue_wait", &runtime::RuntimeTiming::output_queue_wait},
        {"output_queue_delay", &runtime::RuntimeTiming::output_queue_delay},
        {"output", &runtime::RuntimeTiming::output},
    };
    output << '{';
    bool first = true;
    for (const auto& entry : stages) {
        const auto& value = timing.*(entry.second);
        if (!first) output << ',';
        first = false;
        output << '"' << entry.first << "\":{\"samples\":" << value.samples
               << ",\"min_ms\":" << value.minimum_ms << ",\"mean_ms\":" << value.average_ms
               << ",\"p50_ms\":" << value.p50_ms << ",\"p95_ms\":" << value.p95_ms
               << ",\"p99_ms\":" << value.p99_ms << ",\"max_ms\":" << value.maximum_ms << '}';
    }
    output << ",\"capture_queue_dropped\":" << timing.capture_queue_dropped
           << ",\"output_queue_dropped\":" << timing.output_queue_dropped << '}';
}

constexpr const char* kFrameHeader =
    "frame_index,runtime_timestamp_ns,timestamp_ns,received_timestamp_ns,observed_timestamp_ns,"
    "timestamp_source,offset,length,width,height\n";
constexpr const char* kImuHeader =
    "sample_index,runtime_timestamp_ns,timestamp_ns,observed_timestamp_ns,"
    "acceleration_x_mps2,acceleration_y_mps2,acceleration_z_mps2,"
    "angular_velocity_x_rad_s,angular_velocity_y_rad_s,angular_velocity_z_rad_s,"
    "servo_control_p_normalized,servo_control_y_normalized\n";
}  // namespace

void validate_camera_imu_sync_options(const CameraImuSyncOptions& options, const AppConfig& config) {
    if (options.directory.empty() || fs::exists(options.directory))
        throw std::invalid_argument("标定输出目录必须是不存在的新目录");
    if (!std::isfinite(options.seconds) || options.seconds < 10 || options.seconds > 600 ||
        options.fps < 10 || options.fps > 60 || options.fps > config.camera.fps ||
        options.max_bytes == 0 || options.max_bytes > 65536ULL * 1024 * 1024 ||
        options.image_queue_capacity == 0 || options.image_queue_capacity > 64 ||
        options.imu_queue_capacity == 0 || options.imu_queue_capacity > 65536)
        throw std::invalid_argument("标定时长、抽样率、队列容量或数据预算无效");
    if (config.imu.mode != ImuMode::serial ||
        config.camera.capture_mode != runtime::CameraCaptureMode::free_run ||
        config.camera.auto_exposure ||
        (config.camera.frame_timestamp_domain != runtime::CameraTimestampDomain::monotonic &&
         config.camera.frame_timestamp_domain != runtime::CameraTimestampDomain::realtime) ||
        config.camera.width <= 0 || config.camera.height <= 0 ||
        !std::isfinite(config.imu.sample_rate_hz) || config.imu.sample_rate_hz <= 0 ||
        config.imu.measurement_latency_ns < 0)
        throw std::invalid_argument("标定要求真实串口 IMU、free_run 相机、固定曝光及已知时钟域");
    // 保守计入 CSV 及文件头开销；采集时会再次校验 SDK 实际尺寸。
    const long double estimate =
        std::ceil(options.seconds * options.fps + 1) *
            (static_cast<long double>(config.camera.width) * config.camera.height + 512) +
        std::ceil(options.seconds * config.imu.sample_rate_hz + 1) * 512 + 65536;
    if (estimate > options.max_bytes)
        throw std::invalid_argument("预计标定数据超过 sync_calibration.max_mib 预算，请缩短采集或提高预算");
}

const char* camera_imu_sync_error_name(CameraImuSyncError error) noexcept {
    switch (error) {
        case CameraImuSyncError::none: return "none";
        case CameraImuSyncError::invalid_camera: return "invalid_camera_data_or_timestamp";
        case CameraImuSyncError::invalid_imu: return "invalid_imu_data_or_timestamp";
        case CameraImuSyncError::image_queue_full: return "calibration_image_queue_overflow";
        case CameraImuSyncError::imu_queue_full: return "calibration_imu_queue_overflow";
        case CameraImuSyncError::budget_exceeded: return "calibration_storage_budget_exceeded";
        case CameraImuSyncError::write_failure: return "calibration_file_write_or_close_failed";
        case CameraImuSyncError::allocation_failure: return "calibration_image_copy_failed";
        case CameraImuSyncError::insufficient_data: return "insufficient_camera_or_imu_records";
    }
    return "unknown";
}

struct CameraImuSyncSession::Impl {
    struct ImageRecord {
        std::uint64_t index = 0;
        Ns runtime_ns = 0, raw_ns = 0, received_ns = 0, observed_ns = 0;
        runtime::CameraTimestampSource source = runtime::CameraTimestampSource::unknown;
        cv::Mat pixels;
    };
    struct ImuRecord {
        std::uint64_t index = 0;
        Ns raw_ns = 0, observed_ns = 0;
        runtime::ImuAngularVelocitySample sample;
    };

    CameraImuSyncOptions options;
    AppConfig config;
    std::atomic<CameraImuSyncError> failure{CameraImuSyncError::none};
    detail::SyncRecordQueue<ImageRecord> images;
    detail::SyncRecordQueue<ImuRecord> imu;
    std::atomic<bool> accepting{false}, stopping{false};
    std::thread writer;
    std::ofstream pixels, frames, samples;
    bool started = false, finished = false, completed = false;
    Ns previous_camera = -1, previous_imu = -1, previous_observed = -1, last_selected = -1;
    int raw_width = 0, raw_height = 0;
    std::uint64_t seen = 0, selected = 0, image_written = 0, imu_accepted = 0, imu_written = 0;
    std::uint64_t data_bytes = 0, pixel_bytes = 0;
    runtime::TimingDistribution copying, conversion, writing;

    Impl(CameraImuSyncOptions settings, const AppConfig& app)
        : options(std::move(settings)), config(app),
          images(options.image_queue_capacity, failure, CameraImuSyncError::image_queue_full),
          imu(options.imu_queue_capacity, failure, CameraImuSyncError::imu_queue_full) {}

    void fail(CameraImuSyncError error) noexcept {
        auto expected = CameraImuSyncError::none;
        failure.compare_exchange_strong(expected, error, std::memory_order_relaxed);
    }

    /**
     * @brief 仅在生命周期或记录线程写文件，并执行共享数据预算检查。
     * @param output 目标文件流。
     * @param data 待写入的字节。
     * @param size 字节数。
     * @return 写调用成功且未超预算时为 true；不承诺掉电后的介质持久化。
     */
    bool write(std::ofstream& output, const char* data, std::size_t size) {
        if (size > options.max_bytes - data_bytes) {
            fail(CameraImuSyncError::budget_exceeded);
            return false;
        }
        const auto began = Clock::now();
        output.write(data, static_cast<std::streamsize>(size));
        writing.record_ns(std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - began).count());
        if (!output) { fail(CameraImuSyncError::write_failure); return false; }
        data_bytes += size;
        return true;
    }

    bool write_image(const ImageRecord& record) {
        const auto began = Clock::now();
        cv::Mat gray;
        cv::cvtColor(record.pixels, gray, cv::COLOR_BGR2GRAY);
        conversion.record_ns(std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - began).count());
        const auto length = static_cast<std::uint64_t>(gray.cols) * gray.rows;
        if (!write(pixels, reinterpret_cast<const char*>(gray.data), length)) return false;
        std::ostringstream row;
        row.imbue(std::locale::classic());
        row << record.index << ',' << record.runtime_ns << ',' << record.raw_ns << ','
            << record.received_ns << ',' << record.observed_ns << ',' << source_name(record.source)
            << ',' << pixel_bytes << ',' << length << ',' << gray.cols << ',' << gray.rows << '\n';
        const auto value = row.str();
        if (!write(frames, value.data(), value.size())) return false;
        pixel_bytes += length;
        ++image_written;
        return true;
    }

    bool write_imu(const ImuRecord& record) {
        const auto& sample = record.sample;
        std::ostringstream row;
        row.imbue(std::locale::classic());
        row << std::setprecision(std::numeric_limits<double>::max_digits10)
            << record.index << ',' << sample.timestamp_ns << ',' << record.raw_ns << ',' << record.observed_ns
            << ',' << sample.acceleration.x << ',' << sample.acceleration.y << ',' << sample.acceleration.z
            << ',' << sample.angular_velocity.x << ',' << sample.angular_velocity.y << ',' << sample.angular_velocity.z
            << ',' << sample.servo_control_p_normalized << ',' << sample.servo_control_y_normalized << '\n';
        const auto value = row.str();
        if (!write(samples, value.data(), value.size())) return false;
        ++imu_written;
        return true;
    }

    /** @brief 公平消费全速 IMU 及抽样图像；只在独立记录线程执行。 */
    void worker() noexcept {
        try {
            while (!stopping.load(std::memory_order_acquire) || !images.empty() || !imu.empty()) {
                if (failure.load(std::memory_order_relaxed) != CameraImuSyncError::none) break;
                bool worked = false;
                ImuRecord sample;
                for (int batch = 0; batch < 64 && imu.try_pop(&sample); ++batch) {
                    worked = true;
                    if (!write_imu(sample)) return;
                }
                ImageRecord image;
                if (images.try_pop(&image)) {
                    worked = true;
                    if (!write_image(image)) return;
                }
                if (!worked) std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        } catch (...) { fail(CameraImuSyncError::write_failure); }
    }

    /**
     * @brief 原子替换清单；只有数据文件已关闭后才能发布 complete=true。
     * @param complete 是否成功完成采集。
     * @param end 采集结束原因。
     * @param summary 停机后的诊断摘要。
     * @note 直接输出整数时间戳，避免浮点 JSON 中间表示丢失纳秒精度。
     */
    void manifest(bool complete, CameraImuSyncEnd end, const CameraImuSyncSummary& summary) {
        const auto temporary = options.directory / "metadata.json.tmp";
        std::ofstream output(temporary, std::ios::trunc);
        output.imbue(std::locale::classic());
        output << std::setprecision(std::numeric_limits<double>::max_digits10) << std::boolalpha;
        output << "{\n  \"version\": 2,\n  \"complete\": " << complete
               << ",\n  \"end_reason\": \"" << end_name(end)
               << "\",\n  \"recording_error\": \"" << camera_imu_sync_error_name(failure.load()) << "\",\n"
               << "  \"image_orientation\": \"sdk_raw\",\n"
               << "  \"analysis_image_orientation\": \"runtime_prepared_180\",\n"
               << "  \"physical_mount_rotation_deg\": 180,\n  \"prepare_rotation_deg\": 180,\n"
               << "  \"prepared_width\": " << config.camera.width
               << ",\n  \"prepared_height\": " << config.camera.height
               << ",\n  \"raw_width\": " << raw_width << ",\n  \"raw_height\": " << raw_height
               << ",\n  \"camera_timestamp_bias_ns\": " << config.camera.camera_timestamp_bias_ns
               << ",\n  \"measurement_latency_ns\": " << config.imu.measurement_latency_ns
               << ",\n  \"camera_clock_domain\": \"" << domain_name(config.camera.frame_timestamp_domain)
               << "\",\n  \"imu_timestamp_source\": \"imu_uart_receive_estimate\",\n"
               << "  \"imu_timestamp_quality\": \"estimated\",\n"
               << "  \"requested_seconds\": " << options.seconds << ",\n  \"recording_fps\": " << options.fps
               << ",\n  \"configured_camera_fps\": " << config.camera.fps
               << ",\n  \"configured_imu_rate_hz\": " << config.imu.sample_rate_hz
               << ",\n  \"serial_baud_rate\": " << config.serial.baud_rate
               << ",\n  \"requested_receive_realtime_priority\": " << config.imu.receive_thread.realtime_priority
               << ",\n  \"require_realtime_priority\": " << config.imu.receive_thread.require_realtime_priority
               << ",\n  \"max_data_bytes\": " << options.max_bytes
               << ",\n  \"control_output_disabled\": true,\n"
               << "  \"load_scope\": \"Actual bench-scene branches only; TX disabled; recording adds overhead.\",\n"
               << "  \"recording\": {\"camera_seen\":" << seen << ",\"camera_selected\":" << selected
               << ",\"camera_written\":" << image_written << ",\"camera_dropped\":" << images.dropped()
               << ",\"imu_accepted\":" << imu_accepted << ",\"imu_written\":" << imu_written
               << ",\"imu_dropped\":" << imu.dropped() << ",\"data_bytes\":" << data_bytes
               << ",\"pixel_bytes\":" << pixel_bytes << ",\"image_queue_capacity\":" << images.capacity()
               << ",\"imu_queue_capacity\":" << imu.capacity()
               << ",\"image_queue_high_water\":" << images.high_water()
               << ",\"imu_queue_high_water\":" << imu.high_water() << ",\"image_copy\":";
        timing_json(output, copying.summary());
        output << ",\"gray_conversion\":";
        timing_json(output, conversion.summary());
        output << ",\"write_calls\":";
        timing_json(output, writing.summary());
        const auto& imu_status = summary.imu;
        output << "},\n  \"imu_source\": {\"available\":" << imu_status.available
               << ",\"delivered_sample_count\":" << imu_status.delivered_sample_count
               << ",\"latest_sample_timestamp_ns\":" << imu_status.latest_sample_timestamp_ns
               << ",\"queued_chunk_count\":" << imu_status.queued_chunk_count
               << ",\"chunk_queue_capacity\":" << imu_status.chunk_queue_capacity
               << ",\"parse_error_count\":" << imu_status.parse_error_count
               << ",\"backlog_event_count\":" << imu_status.backlog_event_count
               << ",\"received_chunk_count\":" << imu_status.received_chunk_count
               << ",\"dropped_chunk_count\":" << imu_status.dropped_chunk_count
               << ",\"sequence_gap_count\":" << imu_status.sequence_gap_count
               << ",\"timestamp_rollback_count\":" << imu_status.timestamp_rollback_count
               << ",\"dropped_sample_count\":" << imu_status.dropped_sample_count
               << ",\"receive_realtime_priority_applied\":" << imu_status.receive_realtime_priority_applied
               << ",\"receive_priority_error\":" << imu_status.receive_priority_error << "},\n"
               << "  \"camera\": {\"sdk_width\":" << summary.sdk_width << ",\"sdk_height\":" << summary.sdk_height
               << ",\"sdk_fps\":" << (std::isfinite(summary.sdk_fps) ? summary.sdk_fps : 0.0)
               << ",\"exposure_us\":" << summary.exposure_us << ",\"gain\":" << summary.gain
               << ",\"queue_dropped\":" << summary.camera_queue_dropped
               << ",\"timestamp_dropped\":" << summary.camera_timestamp_dropped << ",\"callback_wait\":";
        timing_json(output, summary.camera_callback_wait);
        output << ",\"queue_delay\":";
        timing_json(output, summary.camera_queue_delay);
        output << ",\"decode_convert\":";
        timing_json(output, summary.camera_decode);
        output << "},\n  \"runtime_fps\": {\"capture\":" << summary.fps.capture_fps
               << ",\"processing\":" << summary.fps.processing_fps << ",\"output\":" << summary.fps.output_fps
               << "},\n  \"runtime_timing\":";
        runtime_json(output, summary.timing);
        output << "\n}\n";
        output.close();
        if (!output) throw std::runtime_error("无法写入或关闭标定 metadata");
        fs::rename(temporary, options.directory / "metadata.json");
    }
};

CameraImuSyncSession::CameraImuSyncSession(CameraImuSyncOptions options, const AppConfig& config) {
    validate_camera_imu_sync_options(options, config);
    impl_ = std::make_unique<Impl>(std::move(options), config);
}

CameraImuSyncSession::~CameraImuSyncSession() {
    if (impl_->started && !impl_->finished) finish(CameraImuSyncEnd::startup_failure, {});
}

void CameraImuSyncSession::start() {
    auto& state = *impl_;
    if (state.started) throw std::logic_error("标定会话不可重复启动");
    const auto parent = state.options.directory.parent_path();
    if (!parent.empty()) fs::create_directories(parent);
    if (!fs::create_directory(state.options.directory)) throw std::runtime_error("标定目录已存在，拒绝覆盖");
    state.started = true;
    try {
        state.manifest(false, CameraImuSyncEnd::startup_failure, {});
        state.pixels.open(state.options.directory / "frames.gray8", std::ios::binary);
        state.frames.open(state.options.directory / "frames.csv");
        state.samples.open(state.options.directory / "imu.csv");
        const std::string frame_header(kFrameHeader), imu_header(kImuHeader);
        if (!state.pixels || !state.frames || !state.samples ||
            !state.write(state.frames, frame_header.data(), frame_header.size()) ||
            !state.write(state.samples, imu_header.data(), imu_header.size()))
            throw std::runtime_error("无法创建标定数据文件");
        state.writer = std::thread(&Impl::worker, &state);
        state.accepting.store(true, std::memory_order_release);
    } catch (...) {
        state.fail(CameraImuSyncError::write_failure);
        finish(CameraImuSyncEnd::startup_failure, {});
        throw;
    }
}

bool CameraImuSyncSession::record_camera(const runtime::CameraFrame& frame, Ns observed_ns) noexcept {
    auto& state = *impl_;
    if (!state.accepting.load(std::memory_order_acquire) || error() != CameraImuSyncError::none) return false;
    Ns raw_ns = 0;
    const auto view = frame.image.view();
    if (!frame.image.is_valid() || view.format != domain::PixelFormat::bgr8 || observed_ns < 0 ||
        frame.received_timestamp_ns < 0 || observed_ns < frame.received_timestamp_ns ||
        frame.timestamp_ns <= state.previous_camera || observed_ns < state.previous_observed ||
        !runtime::validate_camera_timestamp(frame.timestamp_ns, frame.received_timestamp_ns,
            state.previous_camera, state.config.guidance.max_camera_future_ns, state.config.guidance.max_camera_age_ns) ||
        !subtract_time(frame.timestamp_ns, state.config.camera.camera_timestamp_bias_ns, raw_ns) ||
        frame.timestamp_source != (state.config.camera.frame_timestamp_domain == runtime::CameraTimestampDomain::monotonic
            ? runtime::CameraTimestampSource::sdk_monotonic : runtime::CameraTimestampSource::sdk_realtime_converted) ||
        (state.seen && (view.width != state.raw_width || view.height != state.raw_height))) {
        state.fail(CameraImuSyncError::invalid_camera);
        return false;
    }
    state.previous_camera = frame.timestamp_ns;
    state.previous_observed = observed_ns;
    state.raw_width = view.width;
    state.raw_height = view.height;
    ++state.seen;
    if (state.last_selected >= 0 && observed_ns - state.last_selected < 1'000'000'000 / state.options.fps) return true;
    state.last_selected = observed_ns;
    if (static_cast<std::uint64_t>(view.width) * view.height > state.options.max_bytes) {
        state.fail(CameraImuSyncError::budget_exceeded);
        return false;
    }
    ++state.selected;
    if (state.images.full()) {
        state.images.reject();
        return false;
    }
    try {
        const auto began = Clock::now();
        Impl::ImageRecord record;
        record.index = state.selected - 1;
        record.runtime_ns = frame.timestamp_ns;
        record.raw_ns = raw_ns;
        record.received_ns = frame.received_timestamp_ns;
        record.observed_ns = observed_ns;
        record.source = frame.timestamp_source;
        record.pixels = vision::make_bgr8_mat_view(view).clone();
        const bool accepted = state.images.try_push(std::move(record));
        state.copying.record_ns(std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - began).count());
        if (accepted) return true;
    } catch (...) { state.fail(CameraImuSyncError::allocation_failure); }
    return false;
}

bool CameraImuSyncSession::record_imu(const runtime::ImuAngularVelocitySample& sample, Ns observed_ns) noexcept {
    auto& state = *impl_;
    if (!state.accepting.load(std::memory_order_acquire) || error() != CameraImuSyncError::none) return false;
    Ns raw_ns = 0;
    if (observed_ns < 0 || sample.timestamp_ns <= state.previous_imu ||
        !add_latency(sample.timestamp_ns, state.config.imu.measurement_latency_ns, raw_ns) ||
        !finite_vector(sample.acceleration) || !finite_vector(sample.angular_velocity) ||
        !std::isfinite(sample.servo_control_p_normalized) || !std::isfinite(sample.servo_control_y_normalized)) {
        state.fail(CameraImuSyncError::invalid_imu);
        return false;
    }
    state.previous_imu = sample.timestamp_ns;
    Impl::ImuRecord record;
    record.index = state.imu_accepted;
    record.raw_ns = raw_ns;
    record.observed_ns = observed_ns;
    record.sample = sample;
    if (state.imu.try_push(std::move(record))) { ++state.imu_accepted; return true; }
    return false;
}

CameraImuSyncError CameraImuSyncSession::error() const noexcept { return impl_->failure.load(); }

bool CameraImuSyncSession::finish(CameraImuSyncEnd end, const CameraImuSyncSummary& summary) noexcept {
    auto& state = *impl_;
    if (state.finished) return state.completed;
    if (!state.started) return false;
    state.accepting.store(false, std::memory_order_release);
    state.stopping.store(true, std::memory_order_release);
    if (state.writer.joinable()) state.writer.join();
    for (auto* file : {&state.pixels, &state.frames, &state.samples}) {
        if (file->is_open()) {
            file->flush();
            file->close();
            if (!*file) state.fail(CameraImuSyncError::write_failure);
        }
    }
    Impl::ImageRecord image;
    while (state.images.try_pop(&image)) image.pixels.release();
    Impl::ImuRecord sample;
    while (state.imu.try_pop(&sample)) {}
    const auto& imu = summary.imu;
    if (imu.parse_error_count || imu.dropped_chunk_count || imu.sequence_gap_count ||
        imu.timestamp_rollback_count || imu.dropped_sample_count) state.fail(CameraImuSyncError::invalid_imu);
    if (summary.camera_timestamp_dropped) state.fail(CameraImuSyncError::invalid_camera);
    if (end == CameraImuSyncEnd::duration_elapsed) {
        if (state.image_written < 2 || state.imu_written < 2) state.fail(CameraImuSyncError::insufficient_data);
        if (!imu.available) state.fail(CameraImuSyncError::invalid_imu);
    }
    state.completed = end == CameraImuSyncEnd::duration_elapsed && error() == CameraImuSyncError::none;
    try { state.manifest(state.completed, end, summary); }
    catch (...) { state.fail(CameraImuSyncError::write_failure); state.completed = false; }
    state.finished = true;
    return state.completed;
}

}  // namespace mosas::app
