#include <mosas/guidance/command_output_mapper.hpp>
#include "rotation_validation.hpp"
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
           !std::isnan(c.maximum_command_acceleration) && c.maximum_command_acceleration >= 0 &&
           valid_debug_config(c.debug);
}

void CommandOutputMapper::reset_debug_state_locked() const noexcept {
    last_generation_.reset();
    first_target_detected_time_ns_.reset();
}

void CommandOutputMapper::reset() noexcept {
    std::lock_guard<std::mutex> lock(debug_mutex_);
    reset_debug_state_locked();
}

domain::CommandMappingResult CommandOutputMapper::map(const domain::BallisticOverloadG& request,
    const domain::CommandOutputContext& c) const noexcept {
    using Reason = domain::GuidanceInvalidReason;
    domain::CommandMappingResult result;
    auto& command = result.command;
    auto& trace = result.trace;
    command.source_frame_id = c.source_frame_id;
    command.generation = c.generation;
    const bool debug = config_.debug.mode != GuidanceDebugMode::disabled;
    const bool launch_delay = config_.debug.mode == GuidanceDebugMode::launch_delay;
    command.source_time = launch_delay && c.latest_state ? c.latest_state->time : c.source_time;
    // 固定模式即使没有视觉命令，也要向下位机发布 IMU 确认的上升状态。
    const bool launched = debug
        ? c.latest_state && c.latest_state->launch.phase == domain::LaunchPhase::launched
        : c.launched;
    command.launched = launched && c.generation == c.active_generation;
    command.target_found = !debug && c.target_found && !c.expired;
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
    if (!command.launched) {
        command.launched = command.target_found = false;
        return reject(Reason::launch_not_confirmed);
    }
    if (!c.output_enabled) return reject(Reason::stale_input);
    if (!valid()) return reject(Reason::algorithm_rejected);

    // ==========================================
    // 调试模式分支（发射后延时或识别后发送固定指令）
    // ==========================================
    if (debug) {
        if (c.emit_time_ns < 0) return reject(Reason::stale_input);
        std::lock_guard<std::mutex> lock(debug_mutex_);
        if (last_generation_ != c.generation) {
            reset_debug_state_locked();
            last_generation_ = c.generation;
        }

        bool triggered = false;
        if (launch_delay) {
            const auto& launch_time = c.latest_state->launch.transition_time;
            if (!matching_time(launch_time, c.clock_domain) || c.emit_time_ns < launch_time.monotonic_ns) {
                return reject(Reason::stale_input);
            }
            const int64_t delay_ns = static_cast<int64_t>(config_.debug.trigger_delay_ms) * 1'000'000LL;
            const int64_t elapsed_ns = c.emit_time_ns - launch_time.monotonic_ns;
            if (elapsed_ns >= delay_ns) {
                if (config_.debug.duration_ms > 0) {
                    const int64_t duration_ns = static_cast<int64_t>(config_.debug.duration_ms) * 1'000'000LL;
                    triggered = (elapsed_ns < delay_ns + duration_ns);
                } else {
                    triggered = true;
                }
            }
        } else if (config_.debug.mode == GuidanceDebugMode::target_detected) {
            // 识别须来自未过期的发射后图像；首帧速率无效不影响识别触发。
            if (c.launched && c.target_found && !c.expired && !first_target_detected_time_ns_) {
                first_target_detected_time_ns_ = c.emit_time_ns;
            }
            if (first_target_detected_time_ns_) {
                if (c.emit_time_ns < *first_target_detected_time_ns_) return reject(Reason::stale_input);
                const int64_t elapsed_ns = c.emit_time_ns - *first_target_detected_time_ns_;
                if (config_.debug.duration_ms > 0) {
                    const int64_t duration_ns = static_cast<int64_t>(config_.debug.duration_ms) * 1'000'000LL;
                    triggered = (elapsed_ns < duration_ns);
                } else {
                    triggered = true;
                }
            }
        }

        if (c.latest_state && c.latest_state->attitude.valid) {
            trace.attitude = c.latest_state->attitude;
        }

        if (!triggered) {
            // 延迟等待不受视觉影响；识别模式触发后保留原有识别锁存语义。
            command.target_found = first_target_detected_time_ns_.has_value();
            return reject(Reason::none);
        }

        // 定时触发或识别触发时，识别标志位置 1，确保下位机正确执行指令
        command.target_found = true;

        double y = config_.debug.pitch_overload_g;
        double z = config_.debug.yaw_overload_g;
        select_control_channel(config_.control_channel, y, z);

        const auto limit = [&](double maximum) {
            const double magnitude = std::hypot(y, z);
            if (magnitude <= maximum) return false;
            if (maximum == 0) y = z = 0;
            else { const double scale = maximum / magnitude; y *= scale; z *= scale; }
            return true;
        };

        trace.overload_limited = limit(config_.maximum_command_overload);
        trace.acceleration_limited = limit(config_.maximum_command_acceleration / config_.gravity);
        trace.body_overload = {0, y, z};
        trace.body_acceleration = {0, y * config_.gravity, z * config_.gravity};
        trace.command_overload.value = std::hypot(y, z);
        trace.command_phase.value = trace.command_overload.value == 0 ? 0 : std::atan2(z, y);
        trace.valid = command.valid = true;
        command.invalid_reason = Reason::none;
        command.overload_g = trace.body_overload;
        return result;
    }

    // ==========================================
    // 正常 PNG 制导分支
    // ==========================================
    if (c.expired) return reject(Reason::stale_input);
    if (!c.request_valid) return reject(c.invalid_reason == Reason::none ? Reason::algorithm_rejected : c.invalid_reason);
    if (request.x != 0 || !std::isfinite(request.y) || !std::isfinite(request.z))
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
