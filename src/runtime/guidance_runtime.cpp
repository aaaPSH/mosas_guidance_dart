#include <mosas/runtime/guidance_runtime.hpp>
#include <mosas/runtime/clock.hpp>

#include <algorithm>
#include <cmath>
#include <exception>
#include <iostream>
#include <sstream>
#include <thread>
#include <utility>

namespace mosas::runtime {
namespace {

bool is_finite(double value) {
    return std::isfinite(value);
}

const char* camera_timestamp_source_name(CameraTimestampSource source) {
    switch (source) {
        case CameraTimestampSource::sdk_monotonic:
            return "sdk_monotonic";
        case CameraTimestampSource::sdk_realtime_converted:
            return "sdk_realtime_converted";
        case CameraTimestampSource::unknown:
        default:
            return "unknown";
    }
}

bool is_finite(const Vector3& value) {
    return is_finite(value.x) && is_finite(value.y) && is_finite(value.z);
}

bool is_finite(const domain::CameraIntrinsics& value) {
    return is_finite(value.fx) && is_finite(value.fy) && is_finite(value.cx) &&
           is_finite(value.cy);
}

bool is_finite(
    const domain::Rotation3<domain::DartFrameTag,
                            domain::CameraFrameTag>& rotation) {
    for (const double value : rotation.matrix()) {
        if (!is_finite(value)) {
            return false;
        }
    }
    return true;
}

bool valid_rotation(
    const domain::Rotation3<domain::DartFrameTag,
                            domain::CameraFrameTag>& rotation) {
    if (!is_finite(rotation)) {
        return false;
    }
    const auto& value = rotation.matrix();
    const Vector3 column_0{value[0], value[3], value[6]};
    const Vector3 column_1{value[1], value[4], value[7]};
    const Vector3 column_2{value[2], value[5], value[8]};
    const auto norm = [](const Vector3& vector) {
        return std::hypot(std::hypot(vector.x, vector.y), vector.z);
    };
    const auto dot = [](const Vector3& lhs, const Vector3& rhs) {
        return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
    };
    const double determinant =
        value[0] * (value[4] * value[8] - value[5] * value[7]) -
        value[1] * (value[3] * value[8] - value[5] * value[6]) +
        value[2] * (value[3] * value[7] - value[4] * value[6]);
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
            << ", parse_error_count=" << diagnostics.parse_error_count
            << ", backlog_event_count=" << diagnostics.backlog_event_count
            << ", received_chunks=" << diagnostics.received_chunk_count
            << ", dropped_chunks=" << diagnostics.dropped_chunk_count
            << ", sequence_gaps=" << diagnostics.sequence_gap_count
            << ", timestamp_rollbacks="
            << diagnostics.timestamp_rollback_count
            << ", dropped_samples=" << diagnostics.dropped_sample_count
            << ", queued_chunks=" << diagnostics.queued_chunk_count << '/'
            << diagnostics.chunk_queue_capacity
            << ", realtime_priority="
            << (diagnostics.receive_realtime_priority_applied ? "applied"
                                                               : "not_applied")
            << ", priority_error="
            << (diagnostics.receive_priority_error ? "true" : "false")
            << "}";
    return message.str();
}

domain::SensorTime make_monotonic_time(TimestampNs timestamp_ns) {
    domain::SensorTime time{};
    time.monotonic_ns = timestamp_ns;
    time.source_domain = domain::ClockDomain::monotonic;
    return time;
}

Vector3 to_runtime_vector(
    const domain::AngularVelocityDartRadPerSec& value) {
    return {value.x, value.y, value.z};
}

RotationMatrix3 to_runtime_rotation(
    const domain::Rotation3<domain::NavigationFrameTag,
                            domain::DartFrameTag>& rotation) {
    const auto& matrix = rotation.matrix();
    return {matrix[0], matrix[1], matrix[2], matrix[3], matrix[4],
            matrix[5], matrix[6], matrix[7], matrix[8]};
}

domain::Rotation3<domain::NavigationFrameTag, domain::DartFrameTag>
to_domain_rotation(const RotationMatrix3& rotation) {
    return domain::Rotation3<domain::NavigationFrameTag,
                             domain::DartFrameTag>(
        {rotation.m00, rotation.m01, rotation.m02,
         rotation.m10, rotation.m11, rotation.m12,
         rotation.m20, rotation.m21, rotation.m22});
}

domain::GuidanceCommand make_neutral_command(
    TimestampNs timestamp_ns, std::uint64_t generation) {
    domain::GuidanceCommand command{};
    command.generation = generation;
    command.source_time = make_monotonic_time(timestamp_ns);
    command.invalid_reason = domain::GuidanceInvalidReason::stale_input;
    return command;
}

}  // namespace

GuidanceRuntime::GuidanceRuntime(
    std::unique_ptr<ImuSource> imu_source,
    std::unique_ptr<CameraSource> camera_source,
    std::unique_ptr<CommandSink> command_sink,
    const GuidanceRuntimeConfig& config,
    GuidanceRuntimeDependencies dependencies,
    std::unique_ptr<FrameSink> frame_sink)
    : GuidanceRuntime(std::move(imu_source), std::move(camera_source),
                      std::move(command_sink), config,
                      std::move(dependencies), std::move(frame_sink),
                      std::cerr) {}

GuidanceRuntime::GuidanceRuntime(
    std::unique_ptr<ImuSource> imu_source,
    std::unique_ptr<CameraSource> camera_source,
    std::unique_ptr<CommandSink> command_sink,
    const GuidanceRuntimeConfig& config,
    GuidanceRuntimeDependencies dependencies,
    std::unique_ptr<FrameSink> frame_sink, std::ostream& output)
    : imu_source_(std::move(imu_source)),
      camera_source_(std::move(camera_source)),
      command_sink_(std::move(command_sink)),
      frame_sink_(std::move(frame_sink)),
      config_(config),
      dependencies_(std::move(dependencies)),
      output_(&output),
      info_logger_(output, config_.info_log_queue_capacity),
      capture_queue_(config_.capture_queue_capacity),
      output_queue_(config_.output_queue_capacity),
      angular_velocity_history_(config_.history_capacity),
      attitude_history_(config_.history_capacity) {}

GuidanceRuntime::~GuidanceRuntime() {
    stop();
}

bool GuidanceRuntime::validate_config() const {
    return imu_source_ != nullptr && camera_source_ != nullptr &&
           command_sink_ != nullptr &&
           dependencies_.vision_detector != nullptr &&
           dependencies_.flight_state_estimator != nullptr &&
           dependencies_.los_estimator != nullptr &&
           dependencies_.los_rate_filter != nullptr &&
           dependencies_.guidance_law != nullptr &&
           dependencies_.monotonic_clock != nullptr &&
           is_finite(config_.launch_speed_mps) && config_.launch_speed_mps >= 0.0 &&
           config_.history_capacity > 0 && config_.max_imu_age_ns > 0 &&
           config_.max_camera_future_ns >= 0 &&
           config_.max_camera_age_ns > 0 &&
           is_finite(config_.camera_intrinsics) &&
           config_.camera_intrinsics.fx > 0.0 &&
           config_.camera_intrinsics.fy > 0.0 &&
           valid_rotation(config_.dart_from_camera) &&
           config_.capture_queue_capacity > 0 &&
           config_.output_queue_capacity > 0 &&
           config_.info_log_queue_capacity > 0;
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
    if (!info_logger_.start()) {
        set_error("failed to start asynchronous runtime logger");
        return false;
    }
    const SourceResult configure_result =
        camera_source_->configure(config_.camera_capture);
    if (configure_result.status != SourceStatus::ok) {
        set_fault(format_source_error("camera source", "configure",
                                      configure_result));
        info_logger_.stop();
        return false;
    }

    const SourceResult imu_start_result = imu_source_->start();
    if (imu_start_result.status != SourceStatus::ok) {
        set_fault(format_source_error("imu source", "start",
                                      imu_start_result));
        info_logger_.stop();
        return false;
    }
    command_sink_->set_active_generation(0);
    active_launch_generation_.store(0);
    if (!command_sink_->start()) {
        set_fault("command sink start failed");
        imu_source_->stop();
        camera_source_->stop();
        info_logger_.stop();
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
    angular_velocity_history_.reset();
    attitude_history_.reset();
    dependencies_.vision_detector->reset();
    dependencies_.flight_state_estimator->reset();
    dependencies_.los_rate_filter->reset();
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        latest_state_.reset();
    }
    {
        std::lock_guard<std::mutex> lock(error_mutex_);
        last_error_.clear();
    }
    running_.store(true);
    try {
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
    if (imu_source_ != nullptr) {
        imu_source_->stop();
    }
    if (processing_thread_.joinable()) {
        processing_thread_.join();
    }
    output_queue_.close();
    if (output_thread_.joinable()) {
        output_thread_.join();
    }
    if (frame_sink_ != nullptr) {
        frame_sink_->stop();
        const std::string output_error = frame_sink_->last_error();
        if (!output_error.empty() && !faulted_.load()) {
            set_fault("frame output failed during stop: " + output_error);
        }
    }
    if (command_sink_ != nullptr) {
        command_sink_->stop();
    }
    info_logger_.stop();
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
    return {capture_fps, processing_fps_meter_.snapshot(),
            output_fps_meter_.snapshot()};
}

GuidanceRuntimeTiming GuidanceRuntime::timing() const {
    std::lock_guard<std::mutex> lock(timing_mutex_);
    const auto to_stage = [](const TimingAccumulator& accumulator) {
        const auto summary = accumulator.distribution.summary();
        constexpr double kNanosecondsPerMillisecond = 1e6;
        return GuidanceRuntimeTimingStage{
            summary.samples,
            summary.minimum_ns / kNanosecondsPerMillisecond,
            summary.average_ns / kNanosecondsPerMillisecond,
            summary.p50_ns / kNanosecondsPerMillisecond,
            summary.p95_ns / kNanosecondsPerMillisecond,
            summary.p99_ns / kNanosecondsPerMillisecond,
            summary.maximum_ns / kNanosecondsPerMillisecond};
    };
    return {to_stage(capture_timing_),
            to_stage(capture_queue_wait_timing_),
            to_stage(capture_queue_delay_timing_),
            to_stage(prepare_timing_),
            to_stage(vision_timing_),
            to_stage(line_of_sight_timing_),
            to_stage(guidance_timing_),
            to_stage(command_timing_),
            to_stage(processing_total_timing_),
            to_stage(output_queue_wait_timing_),
            to_stage(output_queue_delay_timing_),
            to_stage(output_timing_),
            capture_queue_.dropped_count(),
            output_queue_.dropped_count()};
}

void GuidanceRuntime::record_timing(
    TimingStage stage, std::chrono::steady_clock::duration duration) noexcept {
    const auto duration_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        duration).count();
    if (duration_ns < 0) {
        return;
    }

    std::lock_guard<std::mutex> lock(timing_mutex_);
    TimingAccumulator* accumulator = nullptr;
    switch (stage) {
        case TimingStage::capture:
            accumulator = &capture_timing_;
            break;
        case TimingStage::capture_queue_wait:
            accumulator = &capture_queue_wait_timing_;
            break;
        case TimingStage::capture_queue_delay:
            accumulator = &capture_queue_delay_timing_;
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
        case TimingStage::output_queue_wait:
            accumulator = &output_queue_wait_timing_;
            break;
        case TimingStage::output_queue_delay:
            accumulator = &output_queue_delay_timing_;
            break;
        case TimingStage::output:
            accumulator = &output_timing_;
            break;
    }
    accumulator->distribution.record_ns(
        static_cast<std::uint64_t>(duration_ns));
}

void GuidanceRuntime::log(const std::string& message) {
    const std::string formatted = "[运行时] " + message;
    const TimestampNs now_ns = dependencies_.monotonic_clock != nullptr
                                   ? dependencies_.monotonic_clock->now_ns()
                                   : monotonic_timestamp_ns();
    if (info_logger_.try_submit_text(now_ns, formatted)) {
        return;
    }
    // 日志线程已经启动时，队列满只丢弃，不能把同步输出带回关键路径。
    if (info_logger_.running()) {
        return;
    }
    std::lock_guard<std::mutex> lock(log_mutex_);
    if (output_ != nullptr) {
        *output_ << formatted << '\n';
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
        const auto state = latest_state();
        if (command_sink_ != nullptr && state.has_value() &&
            state->launch.phase == domain::LaunchPhase::launched) {
            std::lock_guard<std::mutex> lock(launch_generation_mutex_);
            if (active_launch_generation_.load() ==
                state->launch.generation) {
                // 运行时故障时尽力清零，避免下位机继续保持上一条非零指令。
                const TimestampNs now_ns =
                    dependencies_.monotonic_clock->now_ns();
                command_sink_->try_submit(make_neutral_command(
                    now_ns, state->launch.generation));
            }
        }
    }
    stop_requested_.store(true);
    if (frame_sink_ != nullptr) {
        frame_sink_->cancel();
    }
    cancel_sources();
    capture_queue_.close();
    output_queue_.close();
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
        std::uint64_t estimator_generation =
            active_launch_generation_.load();
        while (!stop_requested_.load()) {
            ImuAngularVelocitySample sample{};
            if (!imu_source_->try_pop(&sample)) {
                if (!imu_source_->wait_for_data()) {
                    if (stop_requested_.load()) {
                        return;
                    }
                    set_fault("imu source stopped while waiting for data");
                    return;
                }
                continue;
            }
            if (sample.timestamp_ns < 0 || !is_finite(sample.acceleration) ||
                !is_finite(sample.angular_velocity) ||
                !std::isfinite(sample.servo_control_p_normalized) ||
                !std::isfinite(sample.servo_control_y_normalized)) {
                set_fault("imu source returned an invalid kinematic sample");
                return;
            }
            domain::ImuKinematicSample domain_sample{};
            domain_sample.time = make_monotonic_time(sample.timestamp_ns);
            domain_sample.acceleration = {sample.acceleration.x,
                                          sample.acceleration.y,
                                          sample.acceleration.z};
            domain_sample.angular_velocity = {sample.angular_velocity.x,
                                               sample.angular_velocity.y,
                                               sample.angular_velocity.z};
            domain_sample.servo_control_p_normalized =
                sample.servo_control_p_normalized;
            domain_sample.servo_control_y_normalized =
                sample.servo_control_y_normalized;
            domain::FlightStateSnapshot snapshot{};
            const char* history_error = nullptr;
            {
                // 与代际切换线性化，防止新周期样本被写入旧周期历史。
                std::lock_guard<std::mutex> generation_lock(
                    launch_generation_mutex_);
                const std::uint64_t active_generation =
                    active_launch_generation_.load();
                if (active_generation != estimator_generation) {
                    dependencies_.flight_state_estimator->reset();
                    estimator_generation = active_generation;
                }
                snapshot = dependencies_.flight_state_estimator->update(
                    domain_sample);
                snapshot.time = domain_sample.time;
                snapshot.imu_sample.time = domain_sample.time;
                snapshot.attitude.time = domain_sample.time;
                snapshot.launch.generation = estimator_generation;
                snapshot.imu_sample.acceleration = domain_sample.acceleration;
                snapshot.imu_sample.angular_velocity =
                    snapshot.attitude.corrected_angular_velocity;
                snapshot.imu_sample.servo_control_p_normalized =
                    domain_sample.servo_control_p_normalized;
                snapshot.imu_sample.servo_control_y_normalized =
                    domain_sample.servo_control_y_normalized;

                const ImuAngularVelocitySample corrected_sample{
                    sample.timestamp_ns, sample.acceleration,
                    to_runtime_vector(
                        snapshot.attitude.corrected_angular_velocity),
                    sample.servo_control_p_normalized,
                    sample.servo_control_y_normalized};
                if (!angular_velocity_history_.push(corrected_sample)) {
                    history_error =
                        "imu angular velocity timestamp is not increasing";
                }
                if (history_error == nullptr && snapshot.attitude.valid &&
                    !attitude_history_.push(
                        {sample.timestamp_ns,
                         snapshot.attitude.pitch_rad.value,
                         snapshot.attitude.roll_rad.value,
                         to_runtime_rotation(
                             snapshot.attitude.navigation_from_dart),
                         snapshot.launch.phase == domain::LaunchPhase::launched,
                         snapshot.launch.transition_time.monotonic_ns,
                         estimator_generation})) {
                    history_error = "imu attitude timestamp is not increasing";
                }
            }
            if (history_error != nullptr) {
                set_fault(history_error);
                return;
            }
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                latest_state_ = snapshot;
            }
            if (frame_sink_ != nullptr &&
                !frame_sink_->try_record_imu_sample(domain_sample)) {
                set_fault("failed to record IMU sample");
                return;
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
        std::uint64_t next_frame_id = 0;
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
            if (frame.timestamp_ns < 0 || !frame.image.is_valid()) {
                set_fault("camera source capture returned ok with invalid frame");
                return;
            }
            frame.source_frame_id = next_frame_id++;
            if (frame.received_timestamp_ns >= 0 &&
                frame.timestamp_source != CameraTimestampSource::unknown &&
                !validate_camera_timestamp(
                    frame.timestamp_ns, frame.received_timestamp_ns, -1,
                    config_.max_camera_future_ns,
                    config_.max_camera_age_ns)) {
                const bool timestamp_is_future =
                    frame.timestamp_ns > frame.received_timestamp_ns;
                const TimestampNs timestamp_delta_ns =
                    timestamp_is_future
                        ? frame.timestamp_ns - frame.received_timestamp_ns
                        : frame.received_timestamp_ns - frame.timestamp_ns;
                std::ostringstream message;
                message << "camera source returned an invalid capture timestamp"
                        << " (source="
                        << camera_timestamp_source_name(frame.timestamp_source)
                        << ", capture_timestamp_ns=" << frame.timestamp_ns
                        << ", received_timestamp_ns="
                        << frame.received_timestamp_ns << ", "
                        << (timestamp_is_future ? "future_ns=" : "age_ns=")
                        << timestamp_delta_ns
                        << ", max_future_ns=" << config_.max_camera_future_ns
                        << ", max_age_ns=" << config_.max_camera_age_ns << ')';
                set_fault(message.str());
                return;
            }

            capture_fps_meter_.record();
            record_timing(TimingStage::capture,
                          std::chrono::steady_clock::now() - capture_started);

            QueuedCameraFrame queued_frame;
            queued_frame.frame = std::move(frame);
            queued_frame.enqueued_at = std::chrono::steady_clock::now();
            if (!capture_queue_.push(std::move(queued_frame))) {
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
        bool has_frame_timestamp = false;
        TimestampNs previous_frame_timestamp_ns = 0;
        std::uint64_t imu_match_failure_count = 0;
        bool imu_match_failure_active = false;
        std::chrono::steady_clock::time_point next_imu_match_log{};
        std::uint64_t output_diagnostic_count = 0;
        std::chrono::steady_clock::time_point next_output_diagnostic_log{};

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
            result.enqueued_at = std::chrono::steady_clock::now();
            return output_queue_.push(std::move(result));
        };

        const auto submit_neutral_command =
            [this](TimestampNs timestamp_ns,
                   std::optional<std::uint64_t> launch_generation =
                       std::nullopt) {
                if (!launch_generation.has_value()) {
                    const auto state = latest_state();
                    if (!state.has_value() ||
                        state->launch.phase != domain::LaunchPhase::launched) {
                        return true;
                    }
                    launch_generation = state->launch.generation;
                }

                bool generation_is_current = false;
                bool sent = true;
                {
                    std::lock_guard<std::mutex> lock(
                        launch_generation_mutex_);
                    generation_is_current =
                        active_launch_generation_.load() ==
                        *launch_generation;
                    if (generation_is_current && !stop_requested_.load() &&
                        !faulted_.load()) {
                        // 无法形成可靠引导量时主动发送全零，清除下位机保持的旧指令。
                        sent = command_sink_->try_submit(
                            make_neutral_command(timestamp_ns,
                                                 *launch_generation));
                    }
                }
                if (generation_is_current && !sent) {
                    set_fault("guidance command submit failed");
                    return false;
                }
                return true;
            };

        ProcessedFrame result{};
        QueuedCameraFrame queued_frame{};
        while (true) {
            const auto queue_wait_started = std::chrono::steady_clock::now();
            if (!capture_queue_.pop(&queued_frame)) {
                break;
            }
            const auto dequeued_at = std::chrono::steady_clock::now();
            record_timing(TimingStage::capture_queue_wait,
                          dequeued_at - queue_wait_started);
            record_timing(TimingStage::capture_queue_delay,
                          dequeued_at - queued_frame.enqueued_at);
            result.frame = std::move(queued_frame.frame);
            queued_frame = QueuedCameraFrame{};
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
                    if (!submit_neutral_command(result.frame.timestamp_ns)) {
                        return;
                    }
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
            const domain::SensorTime frame_time =
                make_monotonic_time(timestamp_ns);
            result.vision_detection = {};
            result.vision_detection.observation.source_frame_id =
                result.frame.source_frame_id;
            result.vision_detection.observation.time = frame_time;
            result.vision_detection.observation.reason =
                domain::ObservationReason::not_found;
            result.flight_state = {};
            result.flight_state.time = frame_time;
            result.flight_state.imu_sample.time = frame_time;
            result.guidance = {};
            result.guidance.command.source_frame_id =
                result.frame.source_frame_id;
            result.guidance.command.source_time = frame_time;
            const auto mark_guidance_invalid =
                [&result](domain::GuidanceInvalidReason reason) {
                    result.guidance.valid = false;
                    result.guidance.invalid_reason = reason;
                    result.guidance.command.valid = false;
                    result.guidance.command.invalid_reason = reason;
                };
            mark_guidance_invalid(domain::GuidanceInvalidReason::stale_input);
            result.overlay = {};

            if ((has_frame_timestamp &&
                 timestamp_ns <= previous_frame_timestamp_ns)) {
                std::ostringstream details;
                details << "camera_timestamp_ns=" << timestamp_ns
                        << ", previous_camera_timestamp_ns="
                        << previous_frame_timestamp_ns;
                log_output_diagnostic("相机帧时间戳不递增", details.str());
                dependencies_.los_rate_filter->reset();
                has_frame_timestamp = false;
                if (!submit_neutral_command(timestamp_ns)) {
                    return;
                }
                if (!enqueue_processed_frame()) {
                    return;
                }
                result = ProcessedFrame{};
                continue;
            }

            const auto angular_velocity_lookup =
                angular_velocity_history_.find_interpolated(timestamp_ns);
            const TimestampNs bracket_age_ns =
                angular_velocity_lookup.has_value()
                    ? std::max(timestamp_ns -
                                   angular_velocity_lookup->lower_timestamp_ns,
                               angular_velocity_lookup->upper_timestamp_ns -
                                   timestamp_ns)
                    : 0;
            const bool lookup_is_too_old =
                angular_velocity_lookup.has_value() &&
                bracket_age_ns > config_.max_imu_age_ns;
            if (!angular_velocity_lookup.has_value() || lookup_is_too_old) {
                const auto latest = latest_state();
                std::string reason;
                if (!angular_velocity_lookup.has_value()) {
                    if (!latest.has_value()) {
                        reason = "IMU 角速度历史为空或没有完整插值区间";
                    } else if (latest->time.monotonic_ns > timestamp_ns) {
                        reason = "没有覆盖相机曝光时刻的 IMU 角速度前后样本";
                    } else {
                        reason = "IMU 角速度历史中没有可匹配插值区间";
                    }
                } else if (lookup_is_too_old) {
                    reason = "IMU 角速度插值区间过宽";
                } else {
                    reason = "IMU 角速度插值失败";
                }

                ++imu_match_failure_count;
                const auto now = std::chrono::steady_clock::now();
                if (now >= next_imu_match_log) {
                    std::ostringstream message;
                    message << "警告: 相机帧无法匹配可用 IMU 状态(累计 "
                            << imu_match_failure_count << " 次): 原因="
                            << reason << ", camera_timestamp_ns="
                            << timestamp_ns;
                    if (angular_velocity_lookup.has_value()) {
                        message << ", lower_imu_timestamp_ns="
                                << angular_velocity_lookup->lower_timestamp_ns
                                << ", upper_imu_timestamp_ns="
                                << angular_velocity_lookup->upper_timestamp_ns
                                << ", imu_bracket_age_ms="
                                << static_cast<double>(bracket_age_ns) / 1e6;
                    } else if (latest.has_value()) {
                        message << ", latest_imu_timestamp_ns="
                                << latest->time.monotonic_ns;
                    } else {
                        message << ", latest_imu_timestamp_ns=N/A";
                    }
                    message << ", max_imu_age_ms="
                            << static_cast<double>(config_.max_imu_age_ns) /
                                   1e6
                            << ", history_size="
                            << angular_velocity_history_.size()
                            << ", "
                            << format_imu_source_diagnostics(
                                   imu_source_->diagnostics())
                            << "；视觉/引导 overlay 将保持 N/A";
                    log(message.str());
                    next_imu_match_log = now + std::chrono::seconds(1);
                }
                imu_match_failure_active = true;
                mark_guidance_invalid(domain::GuidanceInvalidReason::stale_input);
                if (!submit_neutral_command(timestamp_ns)) {
                    return;
                }
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
                    message << "IMU 角速度插值匹配已恢复: camera_timestamp_ns="
                            << timestamp_ns << ", lower_imu_timestamp_ns="
                            << angular_velocity_lookup->lower_timestamp_ns
                            << ", upper_imu_timestamp_ns="
                            << angular_velocity_lookup->upper_timestamp_ns
                            << ", imu_bracket_age_ms="
                            << static_cast<double>(bracket_age_ns) / 1e6;
                    log(message.str());
                    next_imu_match_log = now + std::chrono::seconds(1);
                }
                imu_match_failure_active = false;
            }

            const auto attitude_lookup =
                attitude_history_.find_interpolated(timestamp_ns);
            if (!attitude_lookup.has_value()) {
                mark_guidance_invalid(
                    domain::GuidanceInvalidReason::invalid_attitude);
                log_output_diagnostic(
                    "姿态历史无法匹配相机曝光时刻",
                    "camera_timestamp_ns=" + std::to_string(timestamp_ns));
                if (!submit_neutral_command(timestamp_ns)) {
                    return;
                }
                if (!enqueue_processed_frame()) {
                    return;
                }
                result = ProcessedFrame{};
                continue;
            }

            result.flight_state.time = frame_time;
            result.flight_state.imu_sample = {
                frame_time,
                {angular_velocity_lookup->sample.acceleration.x,
                 angular_velocity_lookup->sample.acceleration.y,
                 angular_velocity_lookup->sample.acceleration.z},
                {angular_velocity_lookup->sample.angular_velocity.x,
                 angular_velocity_lookup->sample.angular_velocity.y,
                 angular_velocity_lookup->sample.angular_velocity.z},
                angular_velocity_lookup->sample.servo_control_p_normalized,
                angular_velocity_lookup->sample.servo_control_y_normalized};
            result.flight_state.attitude.time = frame_time;
            result.flight_state.attitude.navigation_from_dart =
                to_domain_rotation(attitude_lookup->body_to_navigation);
            result.flight_state.attitude.pitch_rad.value =
                attitude_lookup->pitch_rad;
            result.flight_state.attitude.roll_rad.value =
                attitude_lookup->roll_rad;
            result.flight_state.attitude.corrected_angular_velocity =
                result.flight_state.imu_sample.angular_velocity;
            result.flight_state.attitude.valid = true;
            result.flight_state.launch.phase =
                attitude_lookup->launched ? domain::LaunchPhase::launched
                                          : domain::LaunchPhase::pre_launch;
            result.flight_state.launch.transition_time =
                attitude_lookup->launch_timestamp_ns >= 0
                    ? make_monotonic_time(attitude_lookup->launch_timestamp_ns)
                    : domain::SensorTime{};
            result.flight_state.launch.generation =
                attitude_lookup->launch_generation;
            result.flight_state.launch.reason =
                attitude_lookup->launched
                    ? domain::LaunchReason::threshold_confirmed
                    : domain::LaunchReason::not_started;
            result.overlay.attitude_valid = true;
            result.overlay.attitude_pitch.value = attitude_lookup->pitch_rad;
            result.overlay.attitude_roll.value = attitude_lookup->roll_rad;
            result.overlay.launch_phase = result.flight_state.launch.phase;

            const auto vision_started = std::chrono::steady_clock::now();
            result.vision_detection = dependencies_.vision_detector->detect(
                result.frame.image.view(), result.frame.source_frame_id,
                frame_time);
            result.vision_detection.observation.source_frame_id =
                result.frame.source_frame_id;
            result.vision_detection.observation.time = frame_time;
            record_timing(TimingStage::vision,
                          std::chrono::steady_clock::now() - vision_started);
            if (!result.vision_detection.observation.valid) {
                mark_guidance_invalid(domain::GuidanceInvalidReason::no_target);
                dependencies_.los_rate_filter->reset();
                has_frame_timestamp = false;
                if (attitude_lookup->launched &&
                    !submit_neutral_command(
                        timestamp_ns, attitude_lookup->launch_generation)) {
                    return;
                }
                if (!enqueue_processed_frame()) {
                    return;
                }
                result = ProcessedFrame{};
                continue;
            }

            const auto line_of_sight_started = std::chrono::steady_clock::now();
            const domain::LineOfSight line_of_sight =
                dependencies_.los_estimator->estimate(
                    result.vision_detection, config_.camera_intrinsics,
                    config_.dart_from_camera);
            record_timing(
                TimingStage::line_of_sight,
                std::chrono::steady_clock::now() - line_of_sight_started);
            if (!line_of_sight.valid) {
                mark_guidance_invalid(domain::GuidanceInvalidReason::invalid_los);
                const std::string details =
                    "camera_timestamp_ns=" + std::to_string(timestamp_ns);
                log_output_diagnostic("视线角计算无效", details);
                dependencies_.los_rate_filter->reset();
                has_frame_timestamp = false;
                if (attitude_lookup->launched &&
                    !submit_neutral_command(
                        timestamp_ns, attitude_lookup->launch_generation)) {
                    return;
                }
                if (!enqueue_processed_frame()) {
                    return;
                }
                result = ProcessedFrame{};
                continue;
            }
            result.overlay.line_of_sight_valid = true;
            result.overlay.line_of_sight_q_y = line_of_sight.q_y;
            result.overlay.line_of_sight_q_z = line_of_sight.q_z;

            if (!has_frame_timestamp) {
                dependencies_.los_rate_filter->reset();
                const domain::AngularVelocityDartRadPerSec body_angular_velocity{
                    angular_velocity_lookup->sample.angular_velocity.x,
                    angular_velocity_lookup->sample.angular_velocity.y,
                    angular_velocity_lookup->sample.angular_velocity.z};
                const domain::LineOfSightAngularVelocity initial_angular_velocity =
                    dependencies_.los_rate_filter->update(
                        line_of_sight, body_angular_velocity, 1.0);
                mark_guidance_invalid(
                    domain::GuidanceInvalidReason::invalid_rate);
                if (!initial_angular_velocity.valid) {
                    std::ostringstream details;
                    details << "camera_timestamp_ns=" << timestamp_ns
                            << ", dt_seconds=1.0";
                    log_output_diagnostic("首帧视线角速度暂不可用",
                                          details.str());
                }
                previous_frame_timestamp_ns = timestamp_ns;
                has_frame_timestamp = true;
                if (attitude_lookup->launched &&
                    !submit_neutral_command(
                        timestamp_ns, attitude_lookup->launch_generation)) {
                    return;
                }
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
                mark_guidance_invalid(domain::GuidanceInvalidReason::invalid_rate);
                std::ostringstream details;
                details << "camera_timestamp_ns=" << timestamp_ns
                        << ", previous_camera_timestamp_ns="
                        << previous_timestamp_ns
                        << ", dt_seconds=" << dt_seconds;
                log_output_diagnostic("视线角速度时间步长无效", details.str());
                dependencies_.los_rate_filter->reset();
                has_frame_timestamp = false;
                if (attitude_lookup->launched &&
                    !submit_neutral_command(
                        timestamp_ns, attitude_lookup->launch_generation)) {
                    return;
                }
                if (!enqueue_processed_frame()) {
                    return;
                }
                result = ProcessedFrame{};
                continue;
            }
            previous_frame_timestamp_ns = timestamp_ns;

            const domain::AngularVelocityDartRadPerSec body_angular_velocity{
                angular_velocity_lookup->sample.angular_velocity.x,
                angular_velocity_lookup->sample.angular_velocity.y,
                angular_velocity_lookup->sample.angular_velocity.z};
            const domain::LineOfSightAngularVelocity angular_velocity =
                dependencies_.los_rate_filter->update(
                    line_of_sight, body_angular_velocity, dt_seconds);
            if (!angular_velocity.valid) {
                mark_guidance_invalid(domain::GuidanceInvalidReason::invalid_rate);
                std::ostringstream details;
                details << "camera_timestamp_ns=" << timestamp_ns
                        << ", dt_seconds=" << dt_seconds;
                log_output_diagnostic("视线角速度暂不可用", details.str());
                if (attitude_lookup->launched &&
                    !submit_neutral_command(
                        timestamp_ns, attitude_lookup->launch_generation)) {
                    return;
                }
                if (!enqueue_processed_frame()) {
                    return;
                }
                result = ProcessedFrame{};
                continue;
            }
            result.overlay.line_of_sight_rate_valid = true;
            result.overlay.line_of_sight_rate_q_y = angular_velocity.q_y;
            result.overlay.line_of_sight_rate_q_z = angular_velocity.q_z;

            domain::GuidanceInput guidance_input{};
            guidance_input.source_frame_id = result.frame.source_frame_id;
            guidance_input.time = frame_time;
            guidance_input.line_of_sight = line_of_sight;
            guidance_input.angular_velocity = angular_velocity;
            guidance_input.speed_mps.value = config_.launch_speed_mps;
            guidance_input.attitude = result.flight_state.attitude;
            const auto guidance_started = std::chrono::steady_clock::now();
            result.guidance = dependencies_.guidance_law->calculate(
                guidance_input);
            result.guidance.command.generation =
                attitude_lookup->launch_generation;
            result.guidance.command.source_frame_id =
                result.frame.source_frame_id;
            result.guidance.command.source_time = frame_time;
            if (result.guidance.valid) {
                result.guidance.invalid_reason =
                    domain::GuidanceInvalidReason::none;
            } else if (result.guidance.invalid_reason ==
                       domain::GuidanceInvalidReason::none) {
                result.guidance.invalid_reason =
                    domain::GuidanceInvalidReason::algorithm_rejected;
            }
            result.guidance.command.valid = result.guidance.valid;
            result.guidance.command.invalid_reason =
                result.guidance.invalid_reason;
            record_timing(TimingStage::guidance,
                          std::chrono::steady_clock::now() - guidance_started);
            if (!result.guidance.valid) {
                std::ostringstream details;
                details << "camera_timestamp_ns=" << timestamp_ns
                        << ", imu_lower_timestamp_ns="
                        << angular_velocity_lookup->lower_timestamp_ns
                        << ", imu_upper_timestamp_ns="
                        << angular_velocity_lookup->upper_timestamp_ns
                        << ", imu_bracket_age_ms="
                        << static_cast<double>(bracket_age_ns) / 1e6;
                log_output_diagnostic("PNG 引导输出无效", details.str());
            }
            if (attitude_lookup->launched && !result.guidance.valid &&
                !submit_neutral_command(
                    timestamp_ns, attitude_lookup->launch_generation)) {
                return;
            }
            if (attitude_lookup->launched && result.guidance.valid &&
                !faulted_.load() && !stop_requested_.load()) {
                result.overlay.body_overload_valid = true;
                result.overlay.body_overload =
                    result.guidance.command.overload_g;
                const auto command_started = std::chrono::steady_clock::now();
                bool generation_is_current = false;
                bool sent = true;
                {
                    std::lock_guard<std::mutex> lock(
                        launch_generation_mutex_);
                    generation_is_current =
                        active_launch_generation_.load() ==
                        attitude_lookup->launch_generation;
                    if (generation_is_current && !stop_requested_.load()) {
                        sent = command_sink_->try_submit(
                            result.guidance.command);
                    }
                }
                record_timing(TimingStage::command,
                              std::chrono::steady_clock::now() - command_started);
                if (generation_is_current && !sent) {
                    set_fault("guidance command submit failed");
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

void GuidanceRuntime::output_worker() {
    try {
        ProcessedFrame result{};
        while (true) {
            const auto queue_wait_started = std::chrono::steady_clock::now();
            if (!output_queue_.pop(&result)) {
                break;
            }
            const auto dequeued_at = std::chrono::steady_clock::now();
            record_timing(TimingStage::output_queue_wait,
                          dequeued_at - queue_wait_started);
            record_timing(TimingStage::output_queue_delay,
                          dequeued_at - result.enqueued_at);
            if (stop_requested_.load()) {
                break;
            }
            if (result.flight_state.launch.generation !=
                active_launch_generation_.load()) {
                result = ProcessedFrame{};
                continue;
            }
            if (frame_sink_ != nullptr) {
                result.overlay.processing_fps =
                    processing_fps_meter_.snapshot();
                const auto output_started = std::chrono::steady_clock::now();
                if (!frame_sink_->publish(
                        result.frame, result.vision_detection,
                        result.flight_state, result.guidance, result.overlay)) {
                    set_fault("frame output failed");
                    return;
                }
                if (frame_sink_->take_rearm_request()) {
                    std::lock_guard<std::mutex> lock(
                        launch_generation_mutex_);
                    const std::uint64_t next_generation =
                        active_launch_generation_.load() + 1;
                    command_sink_->set_active_generation(next_generation);
                    active_launch_generation_.store(next_generation);
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
