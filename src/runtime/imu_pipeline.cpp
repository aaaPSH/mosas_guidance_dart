#include "imu_pipeline.hpp"

#include "launch_generation_coordinator.hpp"
#include "runtime_value_helpers.hpp"

#include <mosas/runtime/rotation_matrix3.hpp>
#include <mosas/runtime/vector3.hpp>

#include <cmath>
#include <exception>
#include <stdexcept>
#include <utility>

namespace mosas::runtime {
namespace {

bool is_finite(double value) noexcept { return std::isfinite(value); }

bool is_finite(const Vector3& value) noexcept {
    return is_finite(value.x) && is_finite(value.y) && is_finite(value.z);
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

}  // namespace

ImuPipeline::ImuPipeline(
    std::unique_ptr<ImuSource> source,
    std::unique_ptr<ports::IFlightStateEstimator> estimator,
    std::size_t history_capacity,
    LaunchGenerationCoordinator& generation_coordinator,
    ImuPipelineCallbacks callbacks)
    : source_(std::move(source)),
      estimator_(std::move(estimator)),
      generation_coordinator_(generation_coordinator),
      callbacks_(std::move(callbacks)),
      angular_velocity_history_(history_capacity),
      attitude_history_(history_capacity) {}

ImuPipeline::~ImuPipeline() = default;

bool ImuPipeline::valid() const noexcept {
    return source_ != nullptr && estimator_ != nullptr;
}

SourceResult ImuPipeline::start_source() {
    if (source_ == nullptr) {
        return {SourceStatus::fatal, "imu source is null"};
    }
    return source_->start();
}

void ImuPipeline::reset() noexcept {
    angular_velocity_history_.reset();
    attitude_history_.reset();
    if (estimator_ != nullptr) {
        estimator_->reset();
    }
    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_state_.reset();
}

void ImuPipeline::start_worker() {
    if (worker_.joinable()) {
        throw std::logic_error("IMU pipeline worker is already running");
    }
    worker_ = std::thread(&ImuPipeline::worker_loop, this);
}

void ImuPipeline::cancel_source() noexcept {
    if (source_ != nullptr) {
        source_->cancel();
    }
}

void ImuPipeline::join_worker() noexcept {
    if (worker_.joinable()) {
        worker_.join();
    }
}

void ImuPipeline::stop_source() noexcept {
    if (source_ != nullptr) {
        source_->stop();
    }
}

std::optional<ImuStateSnapshot> ImuPipeline::latest_state() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return latest_state_;
}

std::optional<ImuAngularVelocityLookup> ImuPipeline::angular_velocity_at(
    TimestampNs timestamp_ns) const noexcept {
    return angular_velocity_history_.find_interpolated(timestamp_ns);
}

std::optional<AttitudeLookup> ImuPipeline::attitude_at(
    TimestampNs timestamp_ns) const noexcept {
    return attitude_history_.find_interpolated(timestamp_ns);
}

std::size_t ImuPipeline::angular_velocity_history_size() const noexcept {
    return angular_velocity_history_.size();
}

ImuSourceDiagnostics ImuPipeline::diagnostics() const noexcept {
    return source_ == nullptr ? ImuSourceDiagnostics{} : source_->diagnostics();
}

void ImuPipeline::report_fault(std::string message) noexcept {
    if (!callbacks_.report_fault) {
        return;
    }
    try {
        callbacks_.report_fault(std::move(message));
    } catch (...) {
    }
}

void ImuPipeline::worker_loop() noexcept {
    try {
        std::uint64_t estimator_generation =
            generation_coordinator_.current();
        while (!callbacks_.stop_requested ||
               !callbacks_.stop_requested()) {
            ImuAngularVelocitySample sample{};
            if (!source_->try_pop(&sample)) {
                if (!source_->wait_for_data()) {
                    if (callbacks_.stop_requested &&
                        callbacks_.stop_requested()) {
                        return;
                    }
                    report_fault("imu source stopped while waiting for data");
                    return;
                }
                continue;
            }
            if (sample.timestamp_ns < 0 || !is_finite(sample.acceleration) ||
                !is_finite(sample.angular_velocity) ||
                !std::isfinite(sample.servo_control_p_normalized) ||
                !std::isfinite(sample.servo_control_y_normalized)) {
                report_fault("imu source returned an invalid kinematic sample");
                return;
            }

            domain::ImuKinematicSample domain_sample{};
            domain_sample.time = detail::make_monotonic_time(sample.timestamp_ns);
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
            generation_coordinator_.with_generation(
                [&](std::uint64_t active_generation) {
                    if (active_generation != estimator_generation) {
                        estimator_->reset();
                        estimator_generation = active_generation;
                    }
                    snapshot = estimator_->update(domain_sample);
                    snapshot.time = domain_sample.time;
                    snapshot.imu_sample.time = domain_sample.time;
                    snapshot.attitude.time = domain_sample.time;
                    snapshot.launch.generation = estimator_generation;
                    snapshot.imu_sample.acceleration =
                        domain_sample.acceleration;
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
                             snapshot.launch.phase ==
                                 domain::LaunchPhase::launched,
                             snapshot.launch.transition_time.monotonic_ns,
                             estimator_generation})) {
                        history_error =
                            "imu attitude timestamp is not increasing";
                    }
                });
            if (history_error != nullptr) {
                report_fault(history_error);
                return;
            }
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                latest_state_ = snapshot;
            }
            if (callbacks_.try_record_sample &&
                !callbacks_.try_record_sample(domain_sample)) {
                report_fault("failed to record IMU sample");
                return;
            }
        }
    } catch (const std::exception& exception) {
        report_fault(exception.what());
    } catch (...) {
        report_fault("unknown exception in imu worker");
    }
}

}  // namespace mosas::runtime
