#include "frame_pipeline.hpp"

#include "imu_pipeline.hpp"
#include "output_dispatcher.hpp"
#include "launch_generation_coordinator.hpp"
#include "runtime_value_helpers.hpp"

#include <mosas/runtime/clock.hpp>
#include <mosas/runtime/rotation_matrix3.hpp>

#include <algorithm>
#include <cmath>
#include <exception>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>

namespace mosas::runtime {
namespace {

const char* camera_timestamp_source_name(CameraTimestampSource source) {
    switch (source) {
        case CameraTimestampSource::sdk_monotonic:
            return "sdk_monotonic";
        case CameraTimestampSource::sdk_realtime_converted:
            return "sdk_realtime_converted";
        case CameraTimestampSource::simulation:
            return "simulation";
        case CameraTimestampSource::unknown:
        default:
            return "unknown";
    }
}

std::string
format_imu_source_diagnostics(const ImuSourceDiagnostics& diagnostics) {
    if (!diagnostics.available) {
        return "imu_source_diagnostics=unavailable";
    }

    std::ostringstream message;
    message << "imu_source={pending_bytes=";
    if (diagnostics.pending_bytes_valid) {
        message << diagnostics.pending_bytes
                << ", pending_frames=" << diagnostics.pending_frame_count;
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
            << ", timestamp_rollbacks=" << diagnostics.timestamp_rollback_count
            << ", dropped_samples=" << diagnostics.dropped_sample_count
            << ", queued_chunks=" << diagnostics.queued_chunk_count << '/'
            << diagnostics.chunk_queue_capacity << ", realtime_priority="
            << (diagnostics.receive_realtime_priority_applied ? "applied"
                                                              : "not_applied")
            << ", priority_error="
            << (diagnostics.receive_priority_error ? "true" : "false") << "}";
    return message.str();
}

domain::Rotation3<domain::NavigationFrameTag, domain::DartFrameTag>
to_domain_rotation(const RotationMatrix3& rotation) {
    return domain::Rotation3<domain::NavigationFrameTag, domain::DartFrameTag>(
        {rotation.m00, rotation.m01, rotation.m02, rotation.m10, rotation.m11,
         rotation.m12, rotation.m20, rotation.m21, rotation.m22});
}

} // namespace

FramePipeline::FramePipeline(
    std::unique_ptr<CameraSource> camera_source,
    std::unique_ptr<ports::IVisionDetector> vision_detector,
    std::unique_ptr<ports::ILosEstimator> los_estimator,
    std::unique_ptr<ports::ILosRateFilter> los_rate_filter,
    std::unique_ptr<ports::IGuidanceLaw> guidance_law,
    RuntimeSupervisorConfig config, ImuPipeline& imu_pipeline,
    CommandSink& command_sink,
    LaunchGenerationCoordinator& generation_coordinator,
    OutputDispatcher& output_dispatcher, FramePipelineCallbacks callbacks)
    : camera_source_(std::move(camera_source)),
      vision_detector_(std::move(vision_detector)),
      los_estimator_(std::move(los_estimator)),
      los_rate_filter_(std::move(los_rate_filter)),
      guidance_law_(std::move(guidance_law)), config_(std::move(config)),
      imu_pipeline_(imu_pipeline), command_sink_(command_sink),
      generation_coordinator_(generation_coordinator),
      output_dispatcher_(output_dispatcher), callbacks_(std::move(callbacks)),
      capture_queue_(config_.capture_queue_capacity) {}

FramePipeline::~FramePipeline() = default;

bool FramePipeline::valid() const noexcept {
    return camera_source_ != nullptr && vision_detector_ != nullptr &&
           los_estimator_ != nullptr && los_rate_filter_ != nullptr &&
           guidance_law_ != nullptr;
}

SourceResult FramePipeline::configure_source() {
    if (camera_source_ == nullptr) {
        return {SourceStatus::fatal, "camera source is null"};
    }
    return camera_source_->configure(config_.camera_capture);
}

void FramePipeline::reset() noexcept {
    capture_queue_.reset();
    if (vision_detector_ != nullptr) {
        vision_detector_->reset();
    }
    if (los_rate_filter_ != nullptr) {
        los_rate_filter_->reset();
    }
}

void FramePipeline::start_workers() {
    if (capture_thread_.joinable() || processing_thread_.joinable()) {
        throw std::logic_error("frame pipeline workers are already running");
    }
    capture_thread_ = std::thread(&FramePipeline::capture_worker, this);
    processing_thread_ = std::thread(&FramePipeline::processing_worker, this);
}

void FramePipeline::cancel_source() noexcept {
    if (camera_source_ != nullptr) {
        camera_source_->cancel();
    }
}

void FramePipeline::join_capture_worker() noexcept {
    if (capture_thread_.joinable()) {
        capture_thread_.join();
    }
}

void FramePipeline::stop_source() noexcept {
    if (camera_source_ != nullptr) {
        camera_source_->stop();
    }
}

void FramePipeline::close_queue() noexcept { capture_queue_.close(); }

void FramePipeline::join_processing_worker() noexcept {
    if (processing_thread_.joinable()) {
        processing_thread_.join();
    }
}

double FramePipeline::processing_fps() const {
    return processing_fps_meter_.snapshot();
}

RuntimeStatistics FramePipeline::statistics() const {
    double capture_fps = capture_fps_meter_.snapshot();
    if (camera_source_ != nullptr) {
        const double source_capture_fps = camera_source_->capture_fps();
        if (std::isfinite(source_capture_fps) && source_capture_fps > 0.0) {
            capture_fps = source_capture_fps;
        }
    }
    return {capture_fps, processing_fps_meter_.snapshot(), 0.0};
}

RuntimeTiming FramePipeline::timing() const {
    std::lock_guard<std::mutex> lock(timing_mutex_);
    const auto to_stage = [](const TimingAccumulator& accumulator) {
        const auto summary = accumulator.distribution.summary();
        constexpr double kNanosecondsPerMillisecond = 1e6;
        return RuntimeTimingStage{
            summary.samples,
            summary.minimum_ns / kNanosecondsPerMillisecond,
            summary.average_ns / kNanosecondsPerMillisecond,
            summary.p50_ns / kNanosecondsPerMillisecond,
            summary.p95_ns / kNanosecondsPerMillisecond,
            summary.p99_ns / kNanosecondsPerMillisecond,
            summary.maximum_ns / kNanosecondsPerMillisecond};
    };
    RuntimeTiming result{};
    result.capture = to_stage(capture_timing_);
    result.capture_queue_wait = to_stage(capture_queue_wait_timing_);
    result.capture_queue_delay = to_stage(capture_queue_delay_timing_);
    result.prepare = to_stage(prepare_timing_);
    result.vision = to_stage(vision_timing_);
    result.line_of_sight = to_stage(line_of_sight_timing_);
    result.guidance = to_stage(guidance_timing_);
    result.command = to_stage(command_timing_);
    result.processing_total = to_stage(processing_total_timing_);
    result.capture_queue_dropped = capture_queue_.dropped_count();
    return result;
}

void FramePipeline::record_timing(
    TimingStage stage, std::chrono::steady_clock::duration duration) noexcept {
    const auto duration_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
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
    }
    accumulator->distribution.record_ns(
        static_cast<std::uint64_t>(duration_ns));
}

void FramePipeline::report_fault(std::string message) noexcept {
    if (!callbacks_.report_fault) {
        return;
    }
    try {
        callbacks_.report_fault(std::move(message));
    } catch (...) {
    }
}

void FramePipeline::log(const std::string& message) const {
    if (callbacks_.log) {
        callbacks_.log(message);
    }
}

void FramePipeline::capture_worker() noexcept {
    try {
        std::uint64_t next_frame_id = 0;
        while (!callbacks_.stop_requested()) {
            CameraFrame frame{};
            const auto capture_started = std::chrono::steady_clock::now();
            const SourceResult source_result = camera_source_->capture(&frame);
            switch (source_result.status) {
                case SourceStatus::ok:
                    break;
                case SourceStatus::timeout:
                    if (!callbacks_.stop_requested()) {
                        std::this_thread::yield();
                    }
                    continue;
                case SourceStatus::cancelled:
                    if (callbacks_.stop_requested()) {
                        return;
                    }
                    report_fault(detail::format_source_error(
                        "camera source", "capture", source_result));
                    return;
                case SourceStatus::fatal:
                    report_fault(detail::format_source_error(
                        "camera source", "capture", source_result));
                    return;
                default:
                    report_fault(
                        "camera source capture returned unknown status");
                    return;
            }
            if (callbacks_.stop_requested()) {
                return;
            }
            if (frame.timestamp_ns < 0 || !frame.image.is_valid()) {
                report_fault(
                    "camera source capture returned ok with invalid frame");
                return;
            }
            const auto capture_time = detail::resolve_sensor_time(frame.timestamp_ns, frame.sensor_time);
            if ((frame.timestamp_source == CameraTimestampSource::simulation) !=
                (capture_time.source == domain::TimestampSource::simulation)) {
                report_fault("camera simulation label and sensor metadata disagree");
                return;
            }
            frame.source_frame_id = next_frame_id++;
            if (frame.received_timestamp_ns >= 0 &&
                frame.timestamp_source != CameraTimestampSource::unknown &&
                !validate_camera_timestamp(
                    frame.timestamp_ns, frame.received_timestamp_ns, -1,
                    config_.max_camera_future_ns, config_.max_camera_age_ns)) {
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
                report_fault(message.str());
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
        report_fault(exception.what());
    } catch (...) {
        report_fault("unknown exception in capture worker");
    }
}

void FramePipeline::processing_worker() noexcept {
    try {
        bool has_frame_timestamp = false;
        TimestampNs previous_frame_timestamp_ns = 0;
        std::optional<std::uint64_t> rate_generation;
        const auto reset_rate_segment = [this, &has_frame_timestamp] {
            los_rate_filter_->reset();
            has_frame_timestamp = false;
        };
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
                            << output_diagnostic_count
                            << " 次): 原因=" << reason << ", " << details;
                    log(message.str());
                    next_output_diagnostic_log = now + std::chrono::seconds(1);
                }
            };

        const auto enqueue_result = [this](ProcessedFrame result) {
            return output_dispatcher_.enqueue(std::move(result));
        };

        ProcessedFrame result{};
        const auto submit_neutral_command =
            [this, &result](TimestampNs timestamp_ns,
                   std::optional<std::uint64_t> launch_generation =
                       std::nullopt) {
                if (!launch_generation.has_value()) {
                    const auto state = imu_pipeline_.latest_state();
                    if (!state.has_value() ||
                        state->launch.phase != domain::LaunchPhase::launched) {
                        return true;
                    }
                    launch_generation = state->launch.generation;
                }

                bool generation_is_current = false;
                bool sent = true;
                generation_coordinator_.with_generation(
                    [&](std::uint64_t active_generation) {
                        generation_is_current =
                            active_generation == *launch_generation;
                        if (generation_is_current &&
                            !callbacks_.stop_requested() &&
                            !callbacks_.faulted()) {
                            // 无法形成可靠引导量时主动发送全零，清除下位机保持的旧指令。
                            sent = command_sink_.try_submit(
                                detail::make_neutral_command(
                                    detail::resolve_sensor_time(timestamp_ns, result.frame.sensor_time), *launch_generation));
                        }
                    });
                if (generation_is_current && !sent) {
                    report_fault("guidance command submit failed");
                    return false;
                }
                return true;
            };

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
            if (callbacks_.stop_requested()) {
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
                    reset_rate_segment();
                    if (!submit_neutral_command(result.frame.timestamp_ns)) {
                        return;
                    }
                    result = ProcessedFrame{};
                    continue;
                }
                if (prepare_result.status == SourceStatus::cancelled &&
                    callbacks_.stop_requested()) {
                    return;
                }
                report_fault(detail::format_source_error(
                    "camera source", "prepare", prepare_result));
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
                detail::resolve_sensor_time(timestamp_ns, result.frame.sensor_time);
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
                reset_rate_segment();
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
                imu_pipeline_.angular_velocity_at(timestamp_ns);
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
                const auto latest = imu_pipeline_.latest_state();
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
                            << imu_match_failure_count
                            << " 次): 原因=" << reason
                            << ", camera_timestamp_ns=" << timestamp_ns;
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
                            << static_cast<double>(config_.max_imu_age_ns) / 1e6
                            << ", history_size="
                            << imu_pipeline_.angular_velocity_history_size()
                            << ", "
                            << format_imu_source_diagnostics(
                                   imu_pipeline_.diagnostics())
                            << "；视觉/引导 overlay 将保持 N/A";
                    log(message.str());
                    next_imu_match_log = now + std::chrono::seconds(1);
                }
                imu_match_failure_active = true;
                reset_rate_segment();
                mark_guidance_invalid(
                    domain::GuidanceInvalidReason::stale_input);
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
                imu_pipeline_.attitude_at(timestamp_ns);
            if (!attitude_lookup.has_value()) {
                reset_rate_segment();
                mark_guidance_invalid(
                    domain::GuidanceInvalidReason::invalid_attitude);
                log_output_diagnostic("姿态历史无法匹配相机曝光时刻",
                                      "camera_timestamp_ns=" +
                                          std::to_string(timestamp_ns));
                if (!submit_neutral_command(timestamp_ns)) {
                    return;
                }
                if (!enqueue_processed_frame()) {
                    return;
                }
                result = ProcessedFrame{};
                continue;
            }

            if (rate_generation.has_value() &&
                *rate_generation != attitude_lookup->launch_generation) {
                reset_rate_segment();
            }
            rate_generation = attitude_lookup->launch_generation;

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
                    ? (frame_time.source == domain::TimestampSource::simulation
                           ? domain::make_simulation_time(attitude_lookup->launch_timestamp_ns)
                           : detail::make_monotonic_time(attitude_lookup->launch_timestamp_ns))
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
            result.vision_detection = vision_detector_->detect(
                result.frame.image.view(), result.frame.source_frame_id,
                frame_time);
            result.vision_detection.observation.source_frame_id =
                result.frame.source_frame_id;
            result.vision_detection.observation.time = frame_time;
            record_timing(TimingStage::vision,
                          std::chrono::steady_clock::now() - vision_started);
            if (!result.vision_detection.observation.valid) {
                mark_guidance_invalid(domain::GuidanceInvalidReason::no_target);
                reset_rate_segment();
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
            const domain::LineOfSight line_of_sight = los_estimator_->estimate(
                result.vision_detection, config_.camera_intrinsics,
                config_.dart_from_camera);
            record_timing(TimingStage::line_of_sight,
                          std::chrono::steady_clock::now() -
                              line_of_sight_started);
            if (!line_of_sight.valid) {
                mark_guidance_invalid(
                    domain::GuidanceInvalidReason::invalid_los);
                const std::string details =
                    "camera_timestamp_ns=" + std::to_string(timestamp_ns);
                log_output_diagnostic("视线角计算无效", details);
                reset_rate_segment();
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
                los_rate_filter_->reset();
                // 首帧仅建立基准；占位 dt 不用于矢量状态的时间传播。
                const auto initial_rate = los_rate_filter_->update(
                    {line_of_sight, result.flight_state.attitude, 1.0,
                     attitude_lookup->launch_generation});
                mark_guidance_invalid(
                    domain::GuidanceInvalidReason::invalid_rate);
                if (!initial_rate.valid) {
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
            const double dt_seconds =
                static_cast<double>(timestamp_ns - previous_timestamp_ns) *
                1e-9;
            if (!std::isfinite(dt_seconds) || dt_seconds <= 0.0) {
                mark_guidance_invalid(
                    domain::GuidanceInvalidReason::invalid_rate);
                std::ostringstream details;
                details << "camera_timestamp_ns=" << timestamp_ns
                        << ", previous_camera_timestamp_ns="
                        << previous_timestamp_ns
                        << ", dt_seconds=" << dt_seconds;
                log_output_diagnostic("视线角速度时间步长无效", details.str());
                reset_rate_segment();
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

            const auto rate = los_rate_filter_->update(
                {line_of_sight, result.flight_state.attitude, dt_seconds,
                 attitude_lookup->launch_generation});
            if (!rate.valid) {
                mark_guidance_invalid(
                    domain::GuidanceInvalidReason::invalid_rate);
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
            result.overlay.line_of_sight_rate_valid = rate.angular_velocity.valid;
            result.overlay.line_of_sight_rate_q_y = rate.angular_velocity.q_y;
            result.overlay.line_of_sight_rate_q_z = rate.angular_velocity.q_z;

            domain::GuidanceInput guidance_input{};
            guidance_input.source_frame_id = result.frame.source_frame_id;
            guidance_input.time = frame_time;
            guidance_input.line_of_sight = line_of_sight;
            guidance_input.angular_velocity = rate.angular_velocity;
            guidance_input.vector_los = rate.vector;
            guidance_input.speed_mps.value = config_.launch_speed_mps;
            guidance_input.attitude = result.flight_state.attitude;
            const auto guidance_started = std::chrono::steady_clock::now();
            result.guidance = guidance_law_->calculate(guidance_input);
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
                log_output_diagnostic("制导算法输出无效", details.str());
            }
            if (attitude_lookup->launched && !result.guidance.valid &&
                !submit_neutral_command(timestamp_ns,
                                        attitude_lookup->launch_generation)) {
                return;
            }
            if (attitude_lookup->launched && result.guidance.valid &&
                !callbacks_.faulted() && !callbacks_.stop_requested()) {
                result.overlay.body_overload_valid = true;
                result.overlay.body_overload =
                    result.guidance.command.overload_g;
                const auto command_started = std::chrono::steady_clock::now();
                bool generation_is_current = false;
                bool sent = true;
                generation_coordinator_.with_generation(
                    [&](std::uint64_t active_generation) {
                        generation_is_current =
                            active_generation ==
                            attitude_lookup->launch_generation;
                        if (generation_is_current &&
                            !callbacks_.stop_requested()) {
                            sent = command_sink_.try_submit(
                                result.guidance.command);
                        }
                    });
                record_timing(TimingStage::command,
                              std::chrono::steady_clock::now() -
                                  command_started);
                if (generation_is_current && !sent) {
                    report_fault("guidance command submit failed");
                    return;
                }
            }
            if (!enqueue_processed_frame()) {
                return;
            }
            result = ProcessedFrame{};
        }
    } catch (const std::exception& exception) {
        report_fault(exception.what());
    } catch (...) {
        report_fault("unknown exception in processing worker");
    }
}

} // namespace mosas::runtime
