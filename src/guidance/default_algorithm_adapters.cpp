#include <mosas/guidance/default_algorithm_adapters.hpp>
#include <utility>

namespace mosas::guidance {
namespace {

domain::LineOfSight convert(const LineOfSight& value) {
    return {value.valid, {value.q_y}, {value.q_z}};
}

domain::LineOfSightAngularVelocity convert(
    const LineOfSightAngularVelocity& value) {
    return {value.valid, {value.q_y}, {value.q_z}};
}

LineOfSight convert(const domain::LineOfSight& value) {
    return {value.valid, value.q_y.value, value.q_z.value};
}

LineOfSightAngularVelocity convert(
    const domain::LineOfSightAngularVelocity& value) {
    return {value.valid, value.q_y.value, value.q_z.value};
}

Vector3 convert(const domain::AngularVelocityDartRadPerSec& value) {
    return {value.x, value.y, value.z};
}

}  // namespace

domain::LineOfSight ExistingLosEstimatorAdapter::estimate(
    const domain::VisionDetection& observation,
    const domain::CameraIntrinsics& intrinsics,
    const domain::Rotation3<domain::DartFrameTag,
                            domain::CameraFrameTag>& dart_from_camera) {
    VisionResult legacy_observation{};
    legacy_observation.found = observation.observation.valid;
    legacy_observation.blob.center_x = observation.observation.center.u;
    legacy_observation.blob.center_y = observation.observation.center.v;
    legacy_observation.blob.x = observation.diagnostics.bounding_box.x;
    legacy_observation.blob.y = observation.diagnostics.bounding_box.y;
    legacy_observation.blob.width =
        observation.diagnostics.bounding_box.width;
    legacy_observation.blob.height =
        observation.diagnostics.bounding_box.height;
    legacy_observation.blob.area = observation.diagnostics.area_pixels;

    const auto& matrix = dart_from_camera.matrix();
    const RotationMatrix3 legacy_rotation{
        matrix[0], matrix[1], matrix[2],
        matrix[3], matrix[4], matrix[5],
        matrix[6], matrix[7], matrix[8],
    };
    return convert(GuidanceEstimator::calculate_body(
        legacy_observation,
        {intrinsics.fx, intrinsics.fy, intrinsics.cx, intrinsics.cy},
        legacy_rotation));
}

ExistingLosRateFilterAdapter::ExistingLosRateFilterAdapter(
    const LineOfSightRateFilterConfig& config)
    : estimator_(config) {}

domain::LosRateEstimate ExistingLosRateFilterAdapter::update(
    const domain::LosRateFilterInput& input) {
    const auto rate = convert(estimator_.update(
        convert(input.line_of_sight),
        convert(input.attitude.corrected_angular_velocity), input.dt_seconds));
    return {rate.valid, rate, {}};
}

domain::LineOfSightRateSample ExistingLosRateFilterAdapter::sample() const {
    const LineOfSightRateSample legacy = estimator_.sample();
    return {convert(legacy.line_of_sight),
            convert(legacy.raw_angular_velocity),
            convert(legacy.filtered_angular_velocity)};
}

void ExistingLosRateFilterAdapter::reset() noexcept { estimator_.reset(); }

ExistingPngGuidanceLawAdapter::ExistingPngGuidanceLawAdapter(
    const PngGuidanceConfig& config, PngTraceObserver observer)
    : config_(config), observer_(std::move(observer)) {}

domain::GuidanceOutput ExistingPngGuidanceLawAdapter::calculate(
    const domain::GuidanceInput& input) {
    domain::GuidanceOutput output{};
    output.command.source_frame_id = input.source_frame_id;
    output.command.source_time = input.time;
    output.command.invalid_reason = domain::GuidanceInvalidReason::algorithm_rejected;
    if (!input.line_of_sight.valid) {
        output.command.invalid_reason = domain::GuidanceInvalidReason::invalid_los;
        return output;
    }
    if (!input.angular_velocity.valid) {
        output.command.invalid_reason = domain::GuidanceInvalidReason::invalid_rate;
        return output;
    }
    if (!input.attitude.valid) {
        output.command.invalid_reason =
            domain::GuidanceInvalidReason::invalid_attitude;
        return output;
    }

    PngCalculationTrace trace;
    const PngGuidanceOutput legacy = PngGuidance::calculate(
        convert(input.line_of_sight), convert(input.angular_velocity),
        input.speed_mps.value, input.attitude.pitch_rad.value,
        input.attitude.roll_rad.value, config_, observer_ ? &trace : nullptr);
    if (observer_) observer_(input, trace);
    output.valid = legacy.valid;
    output.command.valid = legacy.valid;
    if (!legacy.valid) {
        return output;
    }

    output.command.invalid_reason = domain::GuidanceInvalidReason::none;
    output.command.overload_g = {
        legacy.body_overload.x, legacy.body_overload.y,
        legacy.body_overload.z};
    output.diagnostics.navigation_acceleration = {
        legacy.navigation_acceleration.x, legacy.navigation_acceleration.y,
        legacy.navigation_acceleration.z};
    output.diagnostics.body_acceleration = {
        legacy.body_acceleration.x, legacy.body_acceleration.y,
        legacy.body_acceleration.z};
    output.diagnostics.body_overload = {
        legacy.body_overload.x, legacy.body_overload.y,
        legacy.body_overload.z};
    output.diagnostics.command_overload.value = legacy.command_overload;
    output.diagnostics.command_phase.value = legacy.command_phase;
    return output;
}

}  // namespace mosas::guidance
