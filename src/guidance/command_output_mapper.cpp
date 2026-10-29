#include <mosas/guidance/command_output_mapper.hpp>
#include "vector_los_math.hpp"
#include <cmath>

namespace mosas::guidance {
namespace {
/** @return 时间元数据属于所选运行时间轴时为 true，不混淆原始硬件时钟域。 */
bool matching_time(const domain::SensorTime& time, domain::ClockDomain domain) noexcept {
    if (!time.is_valid()) return false;
    const bool simulated = time.source == domain::TimestampSource::simulation ||
                           time.source_domain == domain::ClockDomain::simulation;
    if (domain == domain::ClockDomain::monotonic) return !simulated;
    return domain == domain::ClockDomain::simulation && simulated &&
           time.source == domain::TimestampSource::simulation &&
           time.source_domain == domain::ClockDomain::simulation &&
           time.quality == domain::TimestampQuality::simulation &&
           time.source_time_ns == time.monotonic_ns;
}
}

bool CommandOutputMapper::valid_config(const CommandMappingConfig& c) noexcept {
    return valid_control_channel(c.control_channel) && std::isfinite(c.gravity) && c.gravity > 0 &&
           std::isfinite(c.maximum_command_overload) && c.maximum_command_overload >= 0 &&
           !std::isnan(c.maximum_command_acceleration) && c.maximum_command_acceleration >= 0;
}

domain::CommandMappingResult CommandOutputMapper::map(const domain::BallisticOverloadG& request,
    const domain::CommandOutputContext& c) const noexcept {
    using Reason = domain::GuidanceInvalidReason;
    domain::CommandMappingResult result;
    auto& command = result.command;
    auto& trace = result.trace;
    command.source_frame_id = c.source_frame_id;
    command.generation = c.generation;
    command.source_time = c.source_time;
    command.launched = c.launched && c.generation == c.active_generation;
    command.target_found = c.target_found && !c.expired;
    trace.emit_time_ns = c.emit_time_ns;
    const auto reject = [&](Reason reason) {
        command.valid = false;
        command.invalid_reason = reason;
        command.overload_g = {};
        trace.valid = false;
        trace.body_overload = {};
        trace.body_acceleration = {};
        trace.ballistic_acceleration = {};
        trace.command_overload = {};
        trace.command_phase = {};
        return result;
    };
    if (c.generation != c.active_generation) {
        command.launched = command.target_found = false;
        return reject(Reason::stale_input);
    }
    if (c.latest_state && (c.latest_state->launch.generation != c.active_generation ||
        c.latest_state->launch.phase != domain::LaunchPhase::launched)) {
        command.launched = command.target_found = false;
        return reject(Reason::launch_not_confirmed);
    }
    if (!c.launched) {
        command.launched = command.target_found = false;
        return reject(Reason::launch_not_confirmed);
    }
    if (c.expired || !c.output_enabled) return reject(Reason::stale_input);
    if (!c.request_valid) return reject(c.invalid_reason == Reason::none ? Reason::algorithm_rejected : c.invalid_reason);
    if (!valid() || request.x != 0 || !std::isfinite(request.y) || !std::isfinite(request.z))
        return reject(Reason::algorithm_rejected);
    trace.raw_navigation_overload = request;
    if (!c.latest_state || !c.latest_state->attitude.valid) return reject(Reason::invalid_attitude);
    trace.attitude = c.latest_state->attitude;
    const auto& time = trace.attitude.time;
    if (!matching_time(c.source_time, c.clock_domain) || !matching_time(time, c.clock_domain) ||
        !matching_time(c.latest_state->time, c.clock_domain) ||
        c.latest_state->time.monotonic_ns != time.monotonic_ns || c.emit_time_ns < time.monotonic_ns ||
        c.maximum_attitude_age_ns <= 0 || c.emit_time_ns - time.monotonic_ns > c.maximum_attitude_age_ns)
        return reject(Reason::stale_input);
    const auto& m = trace.attitude.navigation_from_dart.matrix();
    if (!detail::valid_rotation(m)) return reject(Reason::invalid_attitude);
    const double horizontal = std::hypot(m[4], m[5]);
    const double cosine = horizontal > 1e-6 ? m[4] / horizontal : 1.0;
    const double sine = horizontal > 1e-6 ? -m[5] / horizontal : 0.0;
    double y = cosine * request.y + sine * request.z;
    double z = -sine * request.y + cosine * request.z;
    select_control_channel(config_.control_channel, y, z);
    if (!std::isfinite(std::hypot(y, z))) return reject(Reason::algorithm_rejected);
    const auto limit = [&](double maximum) {
        const double magnitude = std::hypot(y, z);
        if (magnitude <= maximum) return false;
        if (maximum == 0) y = z = 0;
        else { const double scale = maximum / magnitude; y *= scale; z *= scale; }
        return true;
    };
    trace.overload_limited = limit(config_.maximum_command_overload);
    trace.after_overload_limit = {0, cosine * y - sine * z, sine * y + cosine * z};
    trace.acceleration_limited = limit(config_.maximum_command_acceleration / config_.gravity);
    trace.body_overload = {0, y, z};
    trace.body_acceleration = {0, y * config_.gravity, z * config_.gravity};
    const double by = cosine * y - sine * z;
    const double bz = sine * y + cosine * z;
    trace.ballistic_acceleration = {0, by * config_.gravity, bz * config_.gravity};
    if (!std::isfinite(trace.body_acceleration.y) || !std::isfinite(trace.body_acceleration.z) ||
        !std::isfinite(trace.ballistic_acceleration.y) || !std::isfinite(trace.ballistic_acceleration.z))
        return reject(Reason::algorithm_rejected);
    trace.command_overload.value = std::hypot(y, z);
    trace.command_phase.value = trace.command_overload.value == 0 ? 0 : std::atan2(bz, by);
    trace.valid = command.valid = true;
    command.invalid_reason = Reason::none;
    command.overload_g = trace.body_overload;
    return result;
}
}
