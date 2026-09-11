#include <mosas/runtime/guidance_runtime.hpp>
#include <mosas/vision/vision_recognizer.hpp>

#include <algorithm>
#include <cmath>
#include <exception>
#include <iostream>
#include <sstream>
#include <thread>
#include <utility>

#include <opencv2/core.hpp>

namespace mosas::runtime {
namespace {

TimestampNs monotonic_now_ns() noexcept {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

bool is_finite(double value) {
    return std::isfinite(value);
}

bool is_finite(const Vector3& value) {
    return is_finite(value.x) && is_finite(value.y) && is_finite(value.z);
}

bool is_finite(const CameraIntrinsics& value) {
    return is_finite(value.fx) && is_finite(value.fy) && is_finite(value.cx) &&
           is_finite(value.cy);
}

bool is_finite(const RotationMatrix3& value) {
    return is_finite(value.m00) && is_finite(value.m01) &&
           is_finite(value.m02) && is_finite(value.m10) &&
           is_finite(value.m11) && is_finite(value.m12) &&
           is_finite(value.m20) && is_finite(value.m21) &&
           is_finite(value.m22);
}

bool valid_rotation(const RotationMatrix3& value) {
    if (!is_finite(value)) {
        return false;
    }
    const Vector3 column_0{value.m00, value.m10, value.m20};
    const Vector3 column_1{value.m01, value.m11, value.m21};
    const Vector3 column_2{value.m02, value.m12, value.m22};
    const auto norm = [](const Vector3& vector) {
        return std::hypot(std::hypot(vector.x, vector.y), vector.z);
    };
    const auto dot = [](const Vector3& lhs, const Vector3& rhs) {
        return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
    };
    const double determinant =
        value.m00 * (value.m11 * value.m22 - value.m12 * value.m21) -
        value.m01 * (value.m10 * value.m22 - value.m12 * value.m20) +
        value.m02 * (value.m10 * value.m21 - value.m11 * value.m20);
    return std::abs(norm(column_0) - 1.0) <= 1e-6 &&
           std::abs(norm(column_1) - 1.0) <= 1e-6 &&
           std::abs(norm(column_2) - 1.0) <= 1e-6 &&
           std::abs(dot(column_0, column_1)) <= 1e-6 &&
           std::abs(dot(column_0, column_2)) <= 1e-6 &&
           std::abs(dot(column_1, column_2)) <= 1e-6 &&
           std::abs(determinant - 1.0) <= 1e-6;
}

std::string format_source_error(const char* source_name,
                                const char* operation,
                                const SourceResult& result) {
    std::string message = std::string(source_name) + " " + operation + " " +
                          source_status_name(result.status);
    if (!result.message.empty()) {
        message += ": ";
        message += result.message;
    } else if (result.status == SourceStatus::fatal) {
        message += ": source returned fatal without details";
    }
    return message;
}

const char* flight_phase_name(FlightPhase phase) noexcept {
    switch (phase) {
        case FlightPhase::pre_launch:
            return "pre_launch";
        case FlightPhase::ejection:
            return "ejection";
        case FlightPhase::free_flight:
            return "free_flight";
    }
    return "unknown";
}

const char* vision_debug_stage_name(VisionDebugStage stage) noexcept {
    switch (stage) {
        case VisionDebugStage::not_processed:
            return "not_processed";
        case VisionDebugStage::invalid_frame:
            return "invalid_frame";
        case VisionDebugStage::invalid_roi:
            return "invalid_roi";
        case VisionDebugStage::mask_empty:
            return "mask_empty";
        case VisionDebugStage::candidate_rejected:
            return "candidate_rejected";
        case VisionDebugStage::found:
            return "found";
        case VisionDebugStage::processing_error:
            return "processing_error";
    }
    return "unknown";
}

std::string format_imu_source_diagnostics(
    const ImuSourceDiagnostics& diagnostics) {
    if (!diagnostics.available) {
        return "imu_source_diagnostics=unavailable";
    }

    std::ostringstream message;
    message << "imu_source={pending_bytes=";
    if (diagnostics.pending_bytes_valid) {
        message << diagnostics.pending_bytes << ", pending_frames="
                << diagnostics.pending_frame_count;
    } else {
        message << "N/A, pending_frames=N/A";
    }
    message << ", buffered_bytes=" << diagnostics.buffered_bytes
            << ", latest_sample_timestamp_ns="
            << diagnostics.latest_sample_timestamp_ns
            << ", delivered_frames=" << diagnostics.delivered_sample_count
            << ", timeout_count=" << diagnostics.timeout_count
            << ", parse_error_count=" << diagnostics.parse_error_count
            << ", backlog_event_count=" << diagnostics.backlog_event_count
            << "}";
    return message.str();
}

bool valid_detector_config(const FlightPhaseDetectorConfig& config) {
    return config.min_static_samples > 0 &&
           config.min_static_duration_ns >= 0 &&
           is_finite(config.max_static_axis_variation) &&
           config.max_static_axis_variation >= 0.0 &&
           (config.launch_direction == 1 || config.launch_direction == -1) &&
           is_finite(config.ejection_start_threshold) &&
           config.ejection_start_threshold >= 0.0 &&
           config.ejection_confirm_duration_ns >= 0 &&
           is_finite(config.free_flight_release_threshold) &&
           config.free_flight_release_threshold >= 0.0 &&
           config.free_flight_confirm_duration_ns >= 0;
}

bool valid_vision_config(const VisionConfig& config) {
    return config.initial_roi.width > 0 && config.initial_roi.height > 0 &&
           config.threshold.h_min <= config.threshold.h_max &&
           config.threshold.s_min <= config.threshold.s_max &&
           config.threshold.v_min <= config.threshold.v_max &&
           config.found_range_x >= 0 && config.found_range_y >= 0 &&
           config.min_blob_area >= 0 && is_finite(config.min_aspect_ratio) &&
           config.min_aspect_ratio >= 0.0 && is_finite(config.min_fill_ratio) &&
           config.min_fill_ratio >= 0.0;
}

bool valid_los_filter_config(const LineOfSightRateFilterConfig& config) {
    return is_finite(config.process_noise) &&
           is_finite(config.measurement_noise) &&
           is_finite(config.initial_covariance) && config.process_noise >= 0.0 &&
           config.measurement_noise > 0.0 && config.initial_covariance >= 0.0;
}

bool valid_png_config(const PngGuidanceConfig& config) {
    return is_finite(config.navigation_constant_y) &&
           is_finite(config.navigation_constant_z) &&
           is_finite(config.gravity) && config.navigation_constant_y > 0.0 &&
           config.navigation_constant_z > 0.0 && config.gravity > 0.0;
}

}  // namespace

GuidanceRuntime::GuidanceRuntime(
    std::unique_ptr<ImuSource> imu_source,
    std::unique_ptr<CameraSource> camera_source,
    std::unique_ptr<CommandSink> command_sink,
    const GuidanceRuntimeConfig& config, std::unique_ptr<FrameSink> frame_sink)
    : GuidanceRuntime(std::move(imu_source), std::move(camera_source),
                      std::move(command_sink), config, std::move(frame_sink),
                      std::cerr) {}

GuidanceRuntime::GuidanceRuntime(
    std::unique_ptr<ImuSource> imu_source,
    std::unique_ptr<CameraSource> camera_source,
    std::unique_ptr<CommandSink> command_sink,
    const GuidanceRuntimeConfig& config, std::unique_ptr<FrameSink> frame_sink,
    std::ostream& output)
    : imu_source_(std::move(imu_source)),
      camera_source_(std::move(camera_source)),
      command_sink_(std::move(command_sink)),
      frame_sink_(std::move(frame_sink)),
      config_(config),
      output_(&output),
      capture_queue_(config_.capture_queue_capacity),
      output_queue_(config_.output_queue_capacity),
      phase_detector_(config_.phase_detector),
      state_history_(config_.history_capacity),
      dart_condition_(config_.launch_speed_mps) {}

GuidanceRuntime::~GuidanceRuntime() {
    stop();
}

bool GuidanceRuntime::validate_config() const {
    return imu_source_ != nullptr && camera_source_ != nullptr &&
           command_sink_ != nullptr && valid_detector_config(config_.phase_detector) &&
           is_finite(config_.launch_speed_mps) && config_.launch_speed_mps >= 0.0 &&
           config_.history_capacity > 0 && config_.max_imu_age_ns > 0 &&
           config_.max_command_age_ns >= 0 &&
           config_.control_watchdog_timeout_ns >= 0 &&
           config_.processing_deadline_ns >= 0 &&
           valid_vision_config(config_.vision_config) &&
           is_finite(config_.camera_intrinsics) &&
           config_.camera_intrinsics.fx > 0.0 &&
           config_.camera_intrinsics.fy > 0.0 &&
           valid_rotation(config_.camera_to_body) &&
           valid_los_filter_config(config_.los_rate_filter) &&
           valid_png_config(config_.png_guidance) &&
           config_.capture_queue_capacity > 0 &&
           config_.output_queue_capacity > 0;
}

bool GuidanceRuntime::start() {
    if (running_.load()) {
        set_error("guidance runtime is already running");
        return false;
    }
    if (!validate_config()) {
        set_error("invalid guidance runtime configuration");
        return false;
    }
    const SourceResult configure_result =
        camera_source_->configure(config_.camera_capture);
    if (configure_result.status != SourceStatus::ok) {
        set_fault(format_source_error("camera source", "configure",
                                      configure_result));
        return false;
    }

    stop_requested_.store(false);
    faulted_.store(false);
    sources_cancelled_.store(false);
    if (frame_sink_ != nullptr) {
        frame_sink_->reset();
    }
    capture_queue_.reset();
    output_queue_.reset();
    control_mailbox_.reset();
    state_history_.clear();
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        latest_state_.reset();
    }
    {
        std::lock_guard<std::mutex> lock(error_mutex_);
        last_error_.clear();
    }
    phase_detector_ = FlightPhaseDetector(config_.phase_detector);
    dart_condition_.reset();
    capture_timing_.reset();
    prepare_timing_.reset();
    vision_timing_.reset();
    line_of_sight_timing_.reset();
    guidance_timing_.reset();
    command_timing_.reset();
    processing_total_timing_.reset();
    output_timing_.reset();
    control_command_generated_count_.store(0, std::memory_order_relaxed);
    control_command_sent_count_.store(0, std::memory_order_relaxed);
    stale_command_count_.store(0, std::memory_order_relaxed);
    send_failure_count_.store(0, std::memory_order_relaxed);
    control_watchdog_state_.store(ControlWatchdogState::no_command,
                                  std::memory_order_relaxed);
    last_generated_sequence_.store(0, std::memory_order_relaxed);
    last_sent_sequence_.store(0, std::memory_order_relaxed);
    last_generated_time_ns_.store(-1, std::memory_order_relaxed);
    last_sent_time_ns_.store(-1, std::memory_order_relaxed);
    consecutive_send_failures_.store(0, std::memory_order_relaxed);
    consecutive_stale_commands_.store(0, std::memory_order_relaxed);
    last_stale_sequence_.store(0, std::memory_order_relaxed);
    last_stale_age_ns_.store(0, std::memory_order_relaxed);

    running_.store(true);
    try {
        control_thread_ =
            std::thread(&GuidanceRuntime::control_output_worker, this);
        imu_thread_ = std::thread(&GuidanceRuntime::imu_worker, this);
        capture_thread_ = std::thread(&GuidanceRuntime::capture_worker, this);
        processing_thread_ =
            std::thread(&GuidanceRuntime::processing_worker, this);
        output_thread_ = std::thread(&GuidanceRuntime::output_worker, this);
    } catch (const std::exception& exception) {
        set_fault(exception.what());
        stop();
        return false;
    } catch (...) {
        set_fault("failed to start guidance runtime worker");
        stop();
        return false;
    }
    return true;
}

void GuidanceRuntime::stop() noexcept {
    stop_requested_.store(true);
    if (frame_sink_ != nullptr) {
        frame_sink_->cancel();
    }
    cancel_sources();
    control_mailbox_.close();
    if (capture_thread_.joinable()) {
        capture_thread_.join();
    }
    // 采集线程可能仍在执行 SDK 图像转换，必须退出后才能释放相机源。
    if (camera_source_ != nullptr) {
        camera_source_->stop();
    }
    capture_queue_.close();
    if (imu_thread_.joinable()) {
        imu_thread_.join();
    }
    if (processing_thread_.joinable()) {
        processing_thread_.join();
    }
    if (control_thread_.joinable()) {
        control_thread_.join();
    }
    output_queue_.close();
    if (output_thread_.joinable()) {
        output_thread_.join();
    }
    running_.store(false);
}

bool GuidanceRuntime::running() const noexcept {
    return running_.load();
}

bool GuidanceRuntime::faulted() const noexcept {
    return faulted_.load();
}

std::string GuidanceRuntime::last_error() const {
    std::lock_guard<std::mutex> lock(error_mutex_);
    return last_error_;
}

std::optional<ImuStateSnapshot> GuidanceRuntime::latest_state() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return latest_state_;
}

GuidanceRuntimeStatistics GuidanceRuntime::statistics() const {
    double capture_fps = capture_fps_meter_.snapshot();
    if (camera_source_ != nullptr) {
        const double source_capture_fps = camera_source_->capture_fps();
        if (std::isfinite(source_capture_fps) && source_capture_fps > 0.0) {
            capture_fps = source_capture_fps;
        }
    }
    const std::uint64_t runtime_queue_drop_count =
        static_cast<std::uint64_t>(capture_queue_.dropped_count()) +
        static_cast<std::uint64_t>(output_queue_.dropped_count());
    const std::uint64_t camera_source_drop_count =
        camera_source_ == nullptr ? 0 : camera_source_->dropped_frame_count();
    return {capture_fps,
            processing_fps_meter_.snapshot(),
            output_fps_meter_.snapshot(),
            capture_queue_.depth(),
            capture_queue_.high_water_mark(),
            camera_source_drop_count + runtime_queue_drop_count,
            control_command_generated_count_.load(std::memory_order_relaxed),
            control_command_sent_count_.load(std::memory_order_relaxed),
            control_mailbox_.superseded_count(),
            stale_command_count_.load(std::memory_order_relaxed),
            send_failure_count_.load(std::memory_order_relaxed)};
}

GuidanceRuntimeTiming GuidanceRuntime::timing() const {
    const auto to_stage = [](const RuntimeTimingAccumulator& accumulator) {
        const auto snapshot = accumulator.snapshot();
        return GuidanceRuntimeTimingStage{snapshot.samples,
                                          snapshot.average_ms,
                                          snapshot.p95_ms,
                                          snapshot.p99_ms,
                                          snapshot.maximum_ms,
                                          snapshot.deadline_miss_count};
    };
    return {to_stage(capture_timing_), to_stage(prepare_timing_),
            to_stage(vision_timing_), to_stage(line_of_sight_timing_),
            to_stage(guidance_timing_), to_stage(command_timing_),
            to_stage(processing_total_timing_), to_stage(output_timing_)};
}

GuidanceRuntimeControlStatus GuidanceRuntime::control_status() const {
    return {control_watchdog_state_.load(std::memory_order_acquire),
            last_generated_sequence_.load(std::memory_order_relaxed),
            last_sent_sequence_.load(std::memory_order_relaxed),
            last_generated_time_ns_.load(std::memory_order_relaxed),
            last_sent_time_ns_.load(std::memory_order_relaxed),
            consecutive_send_failures_.load(std::memory_order_relaxed),
            consecutive_stale_commands_.load(std::memory_order_relaxed),
            last_stale_sequence_.load(std::memory_order_relaxed),
            last_stale_age_ns_.load(std::memory_order_relaxed)};
}

void GuidanceRuntime::record_timing(
    TimingStage stage, std::chrono::steady_clock::duration duration) noexcept {
    const auto duration_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        duration).count();
    if (duration_ns < 0) {
        return;
    }

    RuntimeTimingAccumulator* accumulator = nullptr;
    switch (stage) {
        case TimingStage::capture:
            accumulator = &capture_timing_;
            break;
        case TimingStage::prepare:
            accumulator = &prepare_timing_;
            break;
        case TimingStage::vision:
            accumulator = &vision_timing_;
            break;
        case TimingStage::line_of_sight:
            accumulator = &line_of_sight_timing_;
            break;
        case TimingStage::guidance:
            accumulator = &guidance_timing_;
            break;
        case TimingStage::command:
            accumulator = &command_timing_;
            break;
        case TimingStage::processing_total:
            accumulator = &processing_total_timing_;
            break;
        case TimingStage::output:
            accumulator = &output_timing_;
            break;
    }
    const std::uint64_t deadline_ns =
        stage == TimingStage::processing_total
            ? static_cast<std::uint64_t>(config_.processing_deadline_ns)
            : 0;
    accumulator->record(static_cast<std::uint64_t>(duration_ns), deadline_ns);
}

void GuidanceRuntime::log(const std::string& message) {
    std::lock_guard<std::mutex> lock(log_mutex_);
    if (output_ != nullptr) {
        *output_ << "[运行时] " << message << '\n';
    }
}

void GuidanceRuntime::set_error(const std::string& message) {
    {
        std::lock_guard<std::mutex> lock(error_mutex_);
        last_error_ = message;
    }
    log("错误: " + message);
}

void GuidanceRuntime::set_fault(const std::string& message) {
    if (!faulted_.exchange(true)) {
        set_error(message);
    }
    stop_requested_.store(true);
    if (frame_sink_ != nullptr) {
        frame_sink_->cancel();
    }
    cancel_sources();
    capture_queue_.close();
    output_queue_.close();
    control_mailbox_.close();
}

void GuidanceRuntime::cancel_sources() noexcept {
    if (sources_cancelled_.exchange(true)) {
        return;
    }
    if (imu_source_ != nullptr) {
        imu_source_->cancel();
    }
    if (camera_source_ != nullptr) {
        camera_source_->cancel();
    }
}

void GuidanceRuntime::imu_worker() {
    try {
    bool initialized = false;
    bool launched = false;
    bool has_attitude_timestamp = false;
    bool has_previous_timestamp = false;
    TimestampNs previous_timestamp_ns = 0;
    TimestampNs previous_attitude_timestamp_ns = 0;
    std::uint64_t timeout_count = 0;
    bool timeout_active = false;
    std::chrono::steady_clock::time_point next_timeout_log{};
    while (!stop_requested_.load()) {
        ImuSample sample{};
        const SourceResult source_result = imu_source_->read(&sample);
        switch (source_result.status) {
            case SourceStatus::ok:
                if (timeout_active) {
                    const auto now = std::chrono::steady_clock::now();
                    if (now >= next_timeout_log) {
                        std::ostringstream message;
                        message << "IMU 数据源读取已恢复: latest_sample_timestamp_ns="
                                << sample.timestamp_ns << ", "
                                << format_imu_source_diagnostics(
                                       imu_source_->diagnostics());
                        log(message.str());
                        next_timeout_log = now + std::chrono::seconds(1);
                    }
                    timeout_active = false;
                }
                break;
            case SourceStatus::timeout: {
                ++timeout_count;
                timeout_active = true;
                const auto now = std::chrono::steady_clock::now();
                if (timeout_count == 1 || now >= next_timeout_log) {
                    std::ostringstream message;
                    message << "警告: IMU 读取超时(累计 " << timeout_count
                            << " 次)";
                    if (!source_result.message.empty()) {
                        message << ": " << source_result.message;
                    }
                    message << ", "
                            << format_imu_source_diagnostics(
                                   imu_source_->diagnostics());
                    log(message.str());
                    next_timeout_log = now + std::chrono::seconds(1);
                }
                if (!stop_requested_.load()) {
                    std::this_thread::yield();
                }
                continue;
            }
            case SourceStatus::cancelled:
                if (stop_requested_.load()) {
                    return;
                }
                set_fault(format_source_error("imu source", "read",
                                              source_result));
                return;
            case SourceStatus::fatal:
                set_fault(format_source_error("imu source", "read",
                                              source_result));
                return;
            default:
                set_fault("imu source read returned unknown status");
                return;
        }
        if (stop_requested_.load()) {
            return;
        }
        if (sample.timestamp_ns < 0 || !is_finite(sample.acceleration) ||
            !is_finite(sample.angular_velocity) ||
            (has_previous_timestamp &&
             sample.timestamp_ns <= previous_timestamp_ns)) {
            set_fault("imu source read returned ok with invalid sample");
            return;
        }

        const FlightPhase phase_before = phase_detector_.phase();
        const FlightPhase phase = config_.skip_imu_self_check
                                      ? FlightPhase::free_flight
                                      : phase_detector_.update(sample);
        has_previous_timestamp = true;
        previous_timestamp_ns = sample.timestamp_ns;

        if (!initialized &&
            (config_.skip_imu_self_check || phase_detector_.initialized())) {
            const Vector3 baseline = config_.skip_imu_self_check
                                         ? Vector3{0.0, 9.80665, 0.0}
                                         : phase_detector_.baseline_acceleration();
            if (!dart_condition_.initialize(
                    baseline)) {
                set_fault("failed to initialize dart condition");
                return;
            }
            initialized = true;
            has_attitude_timestamp = true;
            previous_attitude_timestamp_ns = sample.timestamp_ns;
        }

        if (initialized && has_attitude_timestamp &&
            sample.timestamp_ns > previous_attitude_timestamp_ns) {
            const double dt_seconds = static_cast<double>(
                                         sample.timestamp_ns -
                                         previous_attitude_timestamp_ns) *
                                     1e-9;
            if (!std::isfinite(dt_seconds) || dt_seconds <= 0.0) {
                set_fault("imu attitude update received invalid time step");
                return;
            }
            if (launched) {
                dart_condition_.update(sample.acceleration,
                                       sample.angular_velocity, dt_seconds);
            } else {
                dart_condition_.update_attitude(sample.angular_velocity,
                                                dt_seconds);
            }
            previous_attitude_timestamp_ns = sample.timestamp_ns;
        }

        if (initialized && !launched &&
            (config_.skip_imu_self_check ||
             (phase_before != FlightPhase::ejection &&
              phase == FlightPhase::ejection))) {
            dart_condition_.launch();
            launched = true;
        }

        const ImuStateSnapshot snapshot{
            sample.timestamp_ns,
            phase,
            dart_condition_.attitude(),
            dart_condition_.velocity(),
            sample.acceleration,
            sample.angular_velocity};
        if (!state_history_.push(snapshot)) {
            continue;
        }
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            latest_state_ = snapshot;
        }
    }
    } catch (const std::exception& exception) {
        set_fault(exception.what());
    } catch (...) {
        set_fault("unknown exception in imu worker");
    }
}

void GuidanceRuntime::capture_worker() {
    try {
        while (!stop_requested_.load()) {
            CameraFrame frame{};
            const auto capture_started = std::chrono::steady_clock::now();
            const SourceResult source_result = camera_source_->capture(&frame);
            switch (source_result.status) {
                case SourceStatus::ok:
                    break;
                case SourceStatus::timeout:
                    if (!stop_requested_.load()) {
                        std::this_thread::yield();
                    }
                    continue;
                case SourceStatus::cancelled:
                    if (stop_requested_.load()) {
                        return;
                    }
                    set_fault(format_source_error("camera source", "capture",
                                                  source_result));
                    return;
                case SourceStatus::fatal:
                    set_fault(format_source_error("camera source", "capture",
                                                  source_result));
                    return;
                default:
                    set_fault("camera source capture returned unknown status");
                    return;
            }
            if (stop_requested_.load()) {
                return;
            }
            if (frame.timestamp_ns < 0 || frame.image.empty()) {
                set_fault("camera source capture returned ok with invalid frame");
                return;
            }

            capture_fps_meter_.record();
            record_timing(TimingStage::capture,
                          std::chrono::steady_clock::now() - capture_started);

            if (!capture_queue_.push(std::move(frame))) {
                return;
            }
        }
    } catch (const std::exception& exception) {
        set_fault(exception.what());
    } catch (...) {
        set_fault("unknown exception in capture worker");
    }
}

void GuidanceRuntime::processing_worker() {
    try {
        VisionRecognizer recognizer(config_.vision_config);
        LineOfSightRateEstimator rate_estimator(config_.los_rate_filter);
        bool has_frame_timestamp = false;
        TimestampNs previous_frame_timestamp_ns = 0;
        std::uint64_t imu_match_failure_count = 0;
        bool imu_match_failure_active = false;
        std::chrono::steady_clock::time_point next_imu_match_log{};
        std::uint64_t output_diagnostic_count = 0;
        std::chrono::steady_clock::time_point next_output_diagnostic_log{};
        std::uint64_t next_control_sequence = 0;

        const auto log_output_diagnostic =
            [this, &output_diagnostic_count, &next_output_diagnostic_log](
                const std::string& reason, const std::string& details) {
                ++output_diagnostic_count;
                const auto now = std::chrono::steady_clock::now();
                if (now >= next_output_diagnostic_log) {
                    std::ostringstream message;
                    message << "警告: 视觉/引导输出存在 N/A(累计 "
                            << output_diagnostic_count << " 次): 原因="
                            << reason << ", " << details;
                    log(message.str());
                    next_output_diagnostic_log = now +
                                                  std::chrono::seconds(1);
                }
            };

        const auto enqueue_result = [this](ProcessedFrame result) {
            if (frame_sink_ == nullptr) {
                return true;
            }
            return output_queue_.push(std::move(result));
        };

        ProcessedFrame result{};
        while (capture_queue_.pop(&result.frame)) {
            if (stop_requested_.load()) {
                break;
            }
            const auto processing_started = std::chrono::steady_clock::now();
            const auto prepare_started = std::chrono::steady_clock::now();
            const SourceResult prepare_result =
                camera_source_->prepare(&result.frame);
            if (prepare_result.status == SourceStatus::ok) {
                record_timing(TimingStage::prepare,
                              std::chrono::steady_clock::now() -
                                  prepare_started);
            }
            if (prepare_result.status != SourceStatus::ok) {
                if (prepare_result.status == SourceStatus::timeout) {
                    result = ProcessedFrame{};
                    continue;
                }
                if (prepare_result.status == SourceStatus::cancelled &&
                    stop_requested_.load()) {
                    return;
                }
                set_fault(format_source_error("camera source", "prepare",
                                              prepare_result));
                return;
            }
            const auto enqueue_processed_frame =
                [this, &result, &enqueue_result, processing_started] {
                    processing_fps_meter_.record();
                    record_timing(TimingStage::processing_total,
                                  std::chrono::steady_clock::now() -
                                      processing_started);
                    return enqueue_result(std::move(result));
                };
            const TimestampNs timestamp_ns = result.frame.timestamp_ns;
            result.vision_result = {};
            result.vision_result.next_roi =
                config_.vision_config.initial_roi;
            result.vision_result.minimum_blob_area =
                config_.vision_config.min_blob_area;
            result.imu_state = {
                timestamp_ns, FlightPhase::pre_launch, {}, {}, {}, {}};
            result.guidance = {false, {}, {}, {}, 0.0, 0.0};
            result.overlay = {};

            if ((has_frame_timestamp &&
                 timestamp_ns <= previous_frame_timestamp_ns)) {
                std::ostringstream details;
                details << "camera_timestamp_ns=" << timestamp_ns
                        << ", previous_camera_timestamp_ns="
                        << previous_frame_timestamp_ns;
                log_output_diagnostic("相机帧时间戳不递增", details.str());
                rate_estimator.reset();
                has_frame_timestamp = false;
                if (!enqueue_processed_frame()) {
                    return;
                }
                result = ProcessedFrame{};
                continue;
            }

            const auto state = state_history_.find_at_or_before(timestamp_ns);
            const auto state_age_ns =
                state.has_value() && timestamp_ns >= state->timestamp_ns
                    ? timestamp_ns - state->timestamp_ns
                    : 0;
            const bool state_is_too_old =
                state.has_value() && state_age_ns > config_.max_imu_age_ns;
            const bool state_is_wrong_phase =
                state.has_value() && state->phase != FlightPhase::free_flight;
            if (!state.has_value() || state_is_too_old ||
                state_is_wrong_phase) {
                const auto latest = latest_state();
                std::string reason;
                if (!state.has_value()) {
                    if (!latest.has_value()) {
                        reason = "IMU 状态历史为空";
                    } else if (latest->timestamp_ns > timestamp_ns) {
                        reason = "没有不晚于相机时间戳的 IMU 状态";
                    } else {
                        reason = "IMU 状态历史中没有可匹配状态";
                    }
                } else if (state_is_too_old) {
                    reason = "IMU 状态过旧";
                    if (state_is_wrong_phase) {
                        reason += "且 phase=";
                        reason += flight_phase_name(state->phase);
                        reason += "（需要 free_flight）";
                    }
                } else {
                    reason = std::string("IMU 状态 phase=") +
                             flight_phase_name(state->phase) +
                             "，需要 free_flight";
                }

                ++imu_match_failure_count;
                const auto now = std::chrono::steady_clock::now();
                if (now >= next_imu_match_log) {
                    std::ostringstream message;
                    message << "警告: 相机帧无法匹配可用 IMU 状态(累计 "
                            << imu_match_failure_count << " 次): 原因="
                            << reason << ", camera_timestamp_ns="
                            << timestamp_ns;
                    if (state.has_value()) {
                        message << ", selected_imu_timestamp_ns="
                                << state->timestamp_ns << ", imu_age_ms="
                                << static_cast<double>(state_age_ns) / 1e6;
                    } else if (latest.has_value()) {
                        message << ", latest_imu_timestamp_ns="
                                << latest->timestamp_ns;
                    } else {
                        message << ", latest_imu_timestamp_ns=N/A";
                    }
                    message << ", max_imu_age_ms="
                            << static_cast<double>(config_.max_imu_age_ns) /
                                   1e6
                            << ", history_size=" << state_history_.size()
                            << ", "
                            << format_imu_source_diagnostics(
                                   imu_source_->diagnostics())
                            << "；视觉/引导 overlay 将保持 N/A";
                    log(message.str());
                    next_imu_match_log = now + std::chrono::seconds(1);
                }
                imu_match_failure_active = true;
                if (!enqueue_processed_frame()) {
                    return;
                }
                result = ProcessedFrame{};
                continue;
            }

            if (imu_match_failure_active) {
                const auto now = std::chrono::steady_clock::now();
                if (now >= next_imu_match_log) {
                    std::ostringstream message;
                    message << "IMU 状态匹配已恢复: camera_timestamp_ns="
                            << timestamp_ns << ", imu_timestamp_ns="
                            << state->timestamp_ns << ", imu_age_ms="
                            << static_cast<double>(state_age_ns) / 1e6;
                    log(message.str());
                    next_imu_match_log = now + std::chrono::seconds(1);
                }
                imu_match_failure_active = false;
            }

            result.imu_state = *state;
            result.overlay.attitude_valid = true;
            result.overlay.attitude_pitch_rad = state->attitude.pitch;
            result.overlay.attitude_yaw_rad = state->attitude.yaw;
            result.overlay.attitude_roll_rad = state->attitude.roll;
            result.overlay.velocity_valid = true;
            result.overlay.velocity_x_mps = state->velocity.x;
            result.overlay.velocity_y_mps = state->velocity.y;
            result.overlay.velocity_z_mps = state->velocity.z;

            const auto vision_started = std::chrono::steady_clock::now();
            result.vision_result = recognizer.process(result.frame.image);
            record_timing(TimingStage::vision,
                          std::chrono::steady_clock::now() - vision_started);
            if (!result.vision_result.found) {
                std::ostringstream details;
                details << "camera_timestamp_ns=" << timestamp_ns
                        << ", debug_stage="
                        << vision_debug_stage_name(
                               result.vision_result.debug_stage)
                        << ", mask_pixel_count="
                        << result.vision_result.mask_pixel_count
                        << ", component_count="
                        << result.vision_result.component_count
                        << ", candidate_count="
                        << result.vision_result.candidate_count;
                log_output_diagnostic("视觉未找到目标", details.str());
                rate_estimator.reset();
                has_frame_timestamp = false;
                if (!enqueue_processed_frame()) {
                    return;
                }
                result = ProcessedFrame{};
                continue;
            }

            const auto line_of_sight_started = std::chrono::steady_clock::now();
            const LineOfSight line_of_sight =
                GuidanceEstimator::calculate_compensated(
                    result.vision_result, config_.camera_intrinsics,
                    state->attitude, config_.camera_to_body);
            record_timing(
                TimingStage::line_of_sight,
                std::chrono::steady_clock::now() - line_of_sight_started);
            if (!line_of_sight.valid) {
                std::ostringstream details;
                details << "camera_timestamp_ns=" << timestamp_ns
                        << ", vision_debug_stage="
                        << vision_debug_stage_name(
                               result.vision_result.debug_stage);
                log_output_diagnostic("视线角计算无效", details.str());
                rate_estimator.reset();
                has_frame_timestamp = false;
                if (!enqueue_processed_frame()) {
                    return;
                }
                result = ProcessedFrame{};
                continue;
            }
            result.overlay.line_of_sight_valid = true;
            result.overlay.line_of_sight_q_y_rad = line_of_sight.q_y;
            result.overlay.line_of_sight_q_z_rad = line_of_sight.q_z;

            if (!has_frame_timestamp) {
                rate_estimator.reset();
                const LineOfSightAngularVelocity initial_angular_velocity =
                    rate_estimator.update(line_of_sight, 1.0);
                if (!initial_angular_velocity.valid) {
                    std::ostringstream details;
                    details << "camera_timestamp_ns=" << timestamp_ns
                            << ", dt_seconds=1.0";
                    log_output_diagnostic("首帧视线角速度暂不可用",
                                          details.str());
                }
                previous_frame_timestamp_ns = timestamp_ns;
                has_frame_timestamp = true;
                if (!enqueue_processed_frame()) {
                    return;
                }
                result = ProcessedFrame{};
                continue;
            }

            const TimestampNs previous_timestamp_ns =
                previous_frame_timestamp_ns;
            const double dt_seconds = static_cast<double>(
                                          timestamp_ns -
                                          previous_timestamp_ns) *
                                      1e-9;
            if (!std::isfinite(dt_seconds) || dt_seconds <= 0.0) {
                std::ostringstream details;
                details << "camera_timestamp_ns=" << timestamp_ns
                        << ", previous_camera_timestamp_ns="
                        << previous_timestamp_ns
                        << ", dt_seconds=" << dt_seconds;
                log_output_diagnostic("视线角速度时间步长无效", details.str());
                rate_estimator.reset();
                has_frame_timestamp = false;
                if (!enqueue_processed_frame()) {
                    return;
                }
                result = ProcessedFrame{};
                continue;
            }
            previous_frame_timestamp_ns = timestamp_ns;

            const LineOfSightAngularVelocity angular_velocity =
                rate_estimator.update(line_of_sight, dt_seconds);
            if (!angular_velocity.valid) {
                std::ostringstream details;
                details << "camera_timestamp_ns=" << timestamp_ns
                        << ", dt_seconds=" << dt_seconds;
                log_output_diagnostic("视线角速度暂不可用", details.str());
                if (!enqueue_processed_frame()) {
                    return;
                }
                result = ProcessedFrame{};
                continue;
            }
            result.overlay.line_of_sight_rate_valid = true;
            result.overlay.line_of_sight_rate_q_y_rad_s = angular_velocity.q_y;
            result.overlay.line_of_sight_rate_q_z_rad_s = angular_velocity.q_z;

            const auto guidance_started = std::chrono::steady_clock::now();
            result.guidance = PngGuidance::calculate(
                line_of_sight, angular_velocity, state->velocity,
                state->attitude, config_.png_guidance);
            record_timing(TimingStage::guidance,
                          std::chrono::steady_clock::now() - guidance_started);
            if (!result.guidance.valid) {
                std::ostringstream details;
                details << "camera_timestamp_ns=" << timestamp_ns
                        << ", imu_timestamp_ns=" << state->timestamp_ns
                        << ", imu_age_ms="
                        << static_cast<double>(state_age_ns) / 1e6;
                log_output_diagnostic("PNG 引导输出无效", details.str());
            }
            if (result.guidance.valid && !faulted_.load() &&
                !stop_requested_.load()) {
                result.overlay.body_overload_valid = true;
                result.overlay.body_overload_x_g =
                    result.guidance.body_overload.x;
                result.overlay.body_overload_y_g =
                    result.guidance.body_overload.y;
                result.overlay.body_overload_z_g =
                    result.guidance.body_overload.z;
                const ControlCommand command{
                    ++next_control_sequence,
                    timestamp_ns,
                    static_cast<float>(result.guidance.body_overload.y),
                    static_cast<float>(result.guidance.body_overload.z),
                    kControlCommandValid};
                const TimestampNs generated_time_ns = monotonic_now_ns();
                control_command_generated_count_.fetch_add(
                    1, std::memory_order_relaxed);
                last_generated_sequence_.store(command.sequence,
                                               std::memory_order_relaxed);
                last_generated_time_ns_.store(generated_time_ns,
                                              std::memory_order_relaxed);
                if (!control_mailbox_.submit(command)) {
                    if (!stop_requested_.load()) {
                        set_fault("control command mailbox submit failed");
                    }
                    return;
                }
            }
            if (!enqueue_processed_frame()) {
                return;
            }
            result = ProcessedFrame{};
        }
    } catch (const std::exception& exception) {
        set_fault(exception.what());
    } catch (...) {
        set_fault("unknown exception in processing worker");
    }
}

void GuidanceRuntime::control_output_worker() {
    try {
        constexpr auto kMailboxPollPeriod = std::chrono::milliseconds(1);
        while (!stop_requested_.load(std::memory_order_acquire)) {
            const bool update_available =
                control_mailbox_.wait_for_update(kMailboxPollPeriod);
            if (stop_requested_.load(std::memory_order_acquire)) {
                break;
            }

            if (!update_available) {
                if (config_.control_watchdog_timeout_ns > 0) {
                    const TimestampNs now = monotonic_now_ns();
                    const TimestampNs last_generated =
                        last_generated_time_ns_.load(
                            std::memory_order_relaxed);
                    if (last_generated < 0 ||
                        (now > last_generated &&
                         now - last_generated >
                             config_.control_watchdog_timeout_ns)) {
                        control_watchdog_state_.store(
                            ControlWatchdogState::no_command,
                            std::memory_order_release);
                    }
                }
                continue;
            }

            ControlCommand command{};
            const auto receive_result =
                control_mailbox_.receive_latest(&command);
            if (receive_result == ControlMailbox::ReceiveResult::closed) {
                break;
            }
            if (receive_result != ControlMailbox::ReceiveResult::command ||
                stop_requested_.load(std::memory_order_acquire)) {
                continue;
            }

            const TimestampNs now = monotonic_now_ns();
            const TimestampNs age_ns =
                command.timestamp_ns >= 0 && now > command.timestamp_ns
                    ? now - command.timestamp_ns
                    : 0;
            if (config_.max_command_age_ns > 0 &&
                age_ns > config_.max_command_age_ns) {
                stale_command_count_.fetch_add(1, std::memory_order_relaxed);
                last_stale_sequence_.store(command.sequence,
                                           std::memory_order_relaxed);
                last_stale_age_ns_.store(age_ns, std::memory_order_relaxed);
                consecutive_stale_commands_.fetch_add(
                    1, std::memory_order_relaxed);
                control_watchdog_state_.store(
                    ControlWatchdogState::stale_command,
                    std::memory_order_release);
                continue;
            }

            consecutive_stale_commands_.store(0, std::memory_order_relaxed);
            const auto command_started = std::chrono::steady_clock::now();
            const bool sent = command_sink_->send(command);
            record_timing(TimingStage::command,
                          std::chrono::steady_clock::now() - command_started);
            if (!sent) {
                send_failure_count_.fetch_add(1, std::memory_order_relaxed);
                consecutive_send_failures_.fetch_add(
                    1, std::memory_order_relaxed);
                const bool device_connected =
                    command_sink_->device_connected();
                const auto state = device_connected
                                       ? ControlWatchdogState::send_failure
                                       : ControlWatchdogState::device_disconnected;
                control_watchdog_state_.store(state,
                                              std::memory_order_release);
                std::string message = device_connected
                                          ? "guidance command send failed"
                                          : "control device disconnected";
                const std::string sink_error = command_sink_->last_error();
                if (!sink_error.empty()) {
                    message += ": ";
                    message += sink_error;
                }
                set_fault(message);
                return;
            }

            control_command_sent_count_.fetch_add(1,
                                                  std::memory_order_relaxed);
            last_sent_sequence_.store(command.sequence,
                                      std::memory_order_relaxed);
            last_sent_time_ns_.store(monotonic_now_ns(),
                                     std::memory_order_relaxed);
            consecutive_send_failures_.store(0, std::memory_order_relaxed);
            control_watchdog_state_.store(ControlWatchdogState::healthy,
                                          std::memory_order_release);
        }

        if (!faulted_.load(std::memory_order_acquire)) {
            const auto state =
                control_watchdog_state_.load(std::memory_order_acquire);
            if (state == ControlWatchdogState::no_command ||
                state == ControlWatchdogState::healthy) {
                control_watchdog_state_.store(ControlWatchdogState::shutdown,
                                              std::memory_order_release);
            }
        }
    } catch (const std::exception& exception) {
        control_watchdog_state_.store(ControlWatchdogState::send_failure,
                                      std::memory_order_release);
        set_fault(exception.what());
    } catch (...) {
        control_watchdog_state_.store(ControlWatchdogState::send_failure,
                                      std::memory_order_release);
        set_fault("unknown exception in control output worker");
    }
}

void GuidanceRuntime::output_worker() {
    struct FrameSinkStopGuard {
        FrameSink* sink;

        ~FrameSinkStopGuard() {
            if (sink != nullptr) {
                sink->stop();
            }
        }
    } stop_guard{frame_sink_.get()};

    try {
        ProcessedFrame result{};
        while (output_queue_.pop(&result)) {
            if (stop_requested_.load()) {
                break;
            }
            if (frame_sink_ != nullptr) {
                result.overlay.processing_fps =
                    processing_fps_meter_.snapshot();
                const auto output_started = std::chrono::steady_clock::now();
                if (!frame_sink_->publish(result.frame, result.vision_result,
                                          result.imu_state, result.guidance,
                                          result.overlay)) {
                    set_fault("frame output failed");
                    return;
                }
                record_timing(TimingStage::output,
                              std::chrono::steady_clock::now() - output_started);
                output_fps_meter_.record();
            }
            result = ProcessedFrame{};
        }
    } catch (const std::exception& exception) {
        set_fault(exception.what());
    } catch (...) {
        set_fault("unknown exception in output worker");
    }
}

}  // namespace mosas::runtime
