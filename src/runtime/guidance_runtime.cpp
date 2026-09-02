#include <mosas/runtime/guidance_runtime.hpp>
#include <mosas/vision/vision_recognizer.hpp>

#include <cmath>
#include <exception>
#include <thread>
#include <utility>

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
    return is_finite(config.navigation_constant) &&
           is_finite(config.gravity) && config.navigation_constant > 0.0 &&
           config.gravity > 0.0;
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
           valid_png_config(config_.png_guidance);
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
    if (!camera_source_->configure(config_.camera_capture)) {
        set_fault("camera configuration failed");
        return false;
    }

    stop_requested_.store(false);
    faulted_.store(false);
    sources_cancelled_.store(false);
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
        vision_thread_ = std::thread(&GuidanceRuntime::vision_worker, this);
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
    cancel_sources();
    if (imu_thread_.joinable()) {
        imu_thread_.join();
    }
    if (vision_thread_.joinable()) {
        vision_thread_.join();
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

void GuidanceRuntime::set_error(const std::string& message) {
    std::lock_guard<std::mutex> lock(error_mutex_);
    last_error_ = message;
}

void GuidanceRuntime::set_fault(const std::string& message) {
    if (!faulted_.exchange(true)) {
        set_error(message);
    }
    stop_requested_.store(true);
    cancel_sources();
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
    bool has_update_timestamp = false;
    bool has_previous_timestamp = false;
    TimestampNs previous_timestamp_ns = 0;
    TimestampNs previous_update_timestamp_ns = 0;
    while (!stop_requested_.load()) {
        ImuSample sample{};
        if (!imu_source_->read(&sample)) {
            if (!stop_requested_.load()) {
                std::this_thread::yield();
            }
            continue;
        }
        if (stop_requested_.load() || sample.timestamp_ns < 0 ||
            !is_finite(sample.acceleration) ||
            !is_finite(sample.angular_velocity) ||
            (has_previous_timestamp &&
             sample.timestamp_ns <= previous_timestamp_ns)) {
            continue;
        }

        const FlightPhase phase_before = phase_detector_.phase();
        const FlightPhase phase = phase_detector_.update(sample);
        has_previous_timestamp = true;
        previous_timestamp_ns = sample.timestamp_ns;

        if (!initialized && phase_detector_.initialized()) {
            if (!dart_condition_.initialize(
                    phase_detector_.baseline_acceleration())) {
                set_fault("failed to initialize dart condition");
                return;
            }
            initialized = true;
        }

        if (initialized && !launched && phase_before != FlightPhase::ejection &&
            phase == FlightPhase::ejection) {
            dart_condition_.launch();
            launched = true;
            has_update_timestamp = true;
            previous_update_timestamp_ns = sample.timestamp_ns;
        } else if (launched && has_update_timestamp &&
                   sample.timestamp_ns > previous_update_timestamp_ns) {
            const double dt_seconds = static_cast<double>(
                                         sample.timestamp_ns -
                                         previous_update_timestamp_ns) *
                                     1e-9;
            dart_condition_.update(sample.acceleration, sample.angular_velocity,
                                   dt_seconds);
            previous_update_timestamp_ns = sample.timestamp_ns;
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

void GuidanceRuntime::vision_worker() {
    try {
        VisionRecognizer recognizer(config_.vision_config);
        LineOfSightRateEstimator rate_estimator(config_.los_rate_filter);
        bool has_frame_timestamp = false;
        TimestampNs previous_frame_timestamp_ns = 0;

        while (!stop_requested_.load()) {
            CameraFrame frame{};
            if (!camera_source_->capture(&frame)) {
                if (!stop_requested_.load()) {
                    std::this_thread::yield();
                }
                continue;
            }

            if (frame.timestamp_ns < 0 || frame.image.empty() ||
                frame.image.type() != CV_8UC3 ||
                (has_frame_timestamp &&
                 frame.timestamp_ns <= previous_frame_timestamp_ns)) {
                rate_estimator.reset();
                has_frame_timestamp = false;
                continue;
            }

            const auto state =
                state_history_.find_at_or_before(frame.timestamp_ns);
            if (!state.has_value() ||
                frame.timestamp_ns - state->timestamp_ns >
                    config_.max_imu_age_ns ||
                state->phase != FlightPhase::free_flight) {
                continue;
            }

            const VisionResult vision_result = recognizer.process(frame.image);
            if (!vision_result.found) {
                rate_estimator.reset();
                has_frame_timestamp = false;
                continue;
            }

            const LineOfSight line_of_sight =
                GuidanceEstimator::calculate_compensated(
                    vision_result, config_.camera_intrinsics, state->attitude,
                    config_.camera_to_body);
            if (!line_of_sight.valid) {
                rate_estimator.reset();
                has_frame_timestamp = false;
                continue;
            }

            if (!has_frame_timestamp) {
                rate_estimator.reset();
                rate_estimator.update(line_of_sight, 1.0);
                previous_frame_timestamp_ns = frame.timestamp_ns;
                has_frame_timestamp = true;
                continue;
            }

            const double dt_seconds = static_cast<double>(
                                          frame.timestamp_ns -
                                          previous_frame_timestamp_ns) *
                                      1e-9;
            previous_frame_timestamp_ns = frame.timestamp_ns;
            if (!std::isfinite(dt_seconds) || dt_seconds <= 0.0) {
                rate_estimator.reset();
                has_frame_timestamp = false;
                continue;
            }

            const LineOfSightAngularVelocity angular_velocity =
                rate_estimator.update(line_of_sight, dt_seconds);
            if (!angular_velocity.valid) {
                continue;
            }

            const PngGuidanceOutput guidance = PngGuidance::calculate(
                line_of_sight, angular_velocity, state->velocity,
                state->attitude, config_.png_guidance);
            if (!guidance.valid || faulted_.load()) {
                continue;
            }

            if (!command_sink_->send({frame.timestamp_ns, guidance})) {
                set_fault("guidance command send failed");
                return;
            }
            if (frame_sink_ != nullptr) {
                frame_sink_->publish(frame, vision_result, *state, guidance);
            }
        }
    } catch (const std::exception& exception) {
        set_fault(exception.what());
    } catch (...) {
        set_fault("unknown exception in vision worker");
    }
}

}  // namespace mosas::runtime
