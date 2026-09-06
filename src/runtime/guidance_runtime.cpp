#include <mosas/runtime/guidance_runtime.hpp>
#include <mosas/vision/vision_recognizer.hpp>

#include <algorithm>
#include <cmath>
#include <exception>
#include <thread>
#include <utility>

#include <opencv2/core.hpp>

namespace mosas::runtime {
namespace {

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
    : imu_source_(std::move(imu_source)),
      camera_source_(std::move(camera_source)),
      command_sink_(std::move(command_sink)),
      frame_sink_(std::move(frame_sink)),
      config_(config),
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
    if (processing_thread_.joinable()) {
        processing_thread_.join();
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
    return {capture_fps, processing_fps_meter_.snapshot(),
            output_fps_meter_.snapshot()};
}

GuidanceRuntimeTiming GuidanceRuntime::timing() const {
    std::lock_guard<std::mutex> lock(timing_mutex_);
    const auto to_stage = [](const TimingAccumulator& accumulator) {
        if (accumulator.samples == 0) {
            return GuidanceRuntimeTimingStage{};
        }
        return GuidanceRuntimeTimingStage{
            accumulator.samples,
            static_cast<double>(accumulator.total_ns) /
                static_cast<double>(accumulator.samples) / 1e6,
            static_cast<double>(accumulator.maximum_ns) / 1e6};
    };
    return {to_stage(capture_timing_), to_stage(prepare_timing_),
            to_stage(vision_timing_), to_stage(line_of_sight_timing_),
            to_stage(guidance_timing_), to_stage(command_timing_),
            to_stage(processing_total_timing_), to_stage(output_timing_)};
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
    ++accumulator->samples;
    accumulator->total_ns += static_cast<std::uint64_t>(duration_ns);
    accumulator->maximum_ns = std::max(
        accumulator->maximum_ns, static_cast<std::uint64_t>(duration_ns));
}

void GuidanceRuntime::set_error(const std::string& message) {
    std::lock_guard<std::mutex> lock(error_mutex_);
    last_error_ = message;
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
    while (!stop_requested_.load()) {
        ImuSample sample{};
        const SourceResult source_result = imu_source_->read(&sample);
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
                rate_estimator.reset();
                has_frame_timestamp = false;
                if (!enqueue_processed_frame()) {
                    return;
                }
                result = ProcessedFrame{};
                continue;
            }

            const auto state = state_history_.find_at_or_before(timestamp_ns);
            if (!state.has_value() ||
                timestamp_ns - state->timestamp_ns > config_.max_imu_age_ns ||
                state->phase != FlightPhase::free_flight) {
                if (!enqueue_processed_frame()) {
                    return;
                }
                result = ProcessedFrame{};
                continue;
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
                rate_estimator.update(line_of_sight, 1.0);
                previous_frame_timestamp_ns = timestamp_ns;
                has_frame_timestamp = true;
                if (!enqueue_processed_frame()) {
                    return;
                }
                result = ProcessedFrame{};
                continue;
            }

            const double dt_seconds = static_cast<double>(
                                          timestamp_ns -
                                          previous_frame_timestamp_ns) *
                                      1e-9;
            previous_frame_timestamp_ns = timestamp_ns;
            if (!std::isfinite(dt_seconds) || dt_seconds <= 0.0) {
                rate_estimator.reset();
                has_frame_timestamp = false;
                if (!enqueue_processed_frame()) {
                    return;
                }
                result = ProcessedFrame{};
                continue;
            }

            const LineOfSightAngularVelocity angular_velocity =
                rate_estimator.update(line_of_sight, dt_seconds);
            if (!angular_velocity.valid) {
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
            if (result.guidance.valid && !faulted_.load() &&
                !stop_requested_.load()) {
                result.overlay.body_overload_valid = true;
                result.overlay.body_overload_x_g =
                    result.guidance.body_overload.x;
                result.overlay.body_overload_y_g =
                    result.guidance.body_overload.y;
                result.overlay.body_overload_z_g =
                    result.guidance.body_overload.z;
                const auto command_started = std::chrono::steady_clock::now();
                const bool sent =
                    command_sink_->send({timestamp_ns, result.guidance});
                record_timing(TimingStage::command,
                              std::chrono::steady_clock::now() - command_started);
                if (!sent) {
                    set_fault("guidance command send failed");
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
