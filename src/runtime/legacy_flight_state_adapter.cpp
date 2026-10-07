#include <mosas/runtime/legacy_flight_state_adapter.hpp>

namespace mosas::runtime {
namespace {

Vector3 to_runtime_vector(const domain::AccelerationDartMps2& value) {
    return {value.x, value.y, value.z};
}

Vector3 to_runtime_vector(const domain::AngularVelocityDartRadPerSec& value) {
    return {value.x, value.y, value.z};
}

domain::SensorTime transition_time(const domain::SensorTime& sample_time,
                                   TimestampNs transition_ns) {
    if (transition_ns < 0) {
        return {};
    }
    domain::SensorTime result = sample_time;
    result.monotonic_ns = transition_ns;
    return result;
}

}  // namespace

LegacyFlightStateEstimatorAdapter::LegacyFlightStateEstimatorAdapter(
    const AttitudeEstimatorConfig& config)
    : estimator_(config) {}

domain::FlightStateSnapshot LegacyFlightStateEstimatorAdapter::update(
    const domain::ImuKinematicSample& sample) {
    const TimestampNs timestamp_ns = sample.time.monotonic_ns;
    const ImuAngularVelocitySample legacy_sample{
        timestamp_ns, to_runtime_vector(sample.acceleration),
        to_runtime_vector(sample.angular_velocity),
        sample.servo_control_p_normalized,
        sample.servo_control_y_normalized};
    const AttitudeEstimate estimate = estimator_.update(legacy_sample);

    domain::FlightStateSnapshot snapshot{};
    snapshot.time = sample.time;
    snapshot.imu_sample = sample;
    snapshot.imu_sample.angular_velocity = {
        estimate.corrected_angular_velocity.x,
        estimate.corrected_angular_velocity.y,
        estimate.corrected_angular_velocity.z};
    snapshot.attitude.time = sample.time;
    snapshot.attitude.time.monotonic_ns = estimate.timestamp_ns;
    const RotationMatrix3& matrix = estimate.body_to_navigation;
    snapshot.attitude.navigation_from_dart =
        domain::Rotation3<domain::NavigationFrameTag,
                          domain::DartFrameTag>(
            {matrix.m00, matrix.m01, matrix.m02,
             matrix.m10, matrix.m11, matrix.m12,
             matrix.m20, matrix.m21, matrix.m22});
    snapshot.attitude.pitch_rad.value = estimate.pitch_rad;
    snapshot.attitude.roll_rad.value = estimate.roll_rad;
    snapshot.attitude.corrected_angular_velocity = {
        estimate.corrected_angular_velocity.x,
        estimate.corrected_angular_velocity.y,
        estimate.corrected_angular_velocity.z};
    snapshot.attitude.valid = estimate.valid;

    snapshot.launch.phase = estimate.launched ? domain::LaunchPhase::launched
                                              : domain::LaunchPhase::pre_launch;
    snapshot.launch.transition_time =
        transition_time(sample.time, estimate.launch_timestamp_ns);
    snapshot.launch.reason = estimate.launched
                                 ? domain::LaunchReason::threshold_confirmed
                                 : domain::LaunchReason::not_started;
    return snapshot;
}

void LegacyFlightStateEstimatorAdapter::reset() noexcept {
    estimator_.reset();
}

}  // namespace mosas::runtime
