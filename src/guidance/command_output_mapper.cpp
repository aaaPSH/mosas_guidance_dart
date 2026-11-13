#include <mosas/guidance/command_output_mapper.hpp>
#include "rotation_validation.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

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

/**
 * @brief 按合成过载上限同比缩放两轴，不改变指令方向。
 * @param y 镖体系 Y 过载，必要时原地限幅，单位 g。
 * @param z 镖体系 Z 过载，必要时原地限幅，单位 g。
 * @param maximum 已校验的非负合成上限，允许 +inf，单位 g。
 * @return 指令因超过上限而被修改时为 true。
 */
bool limit_overload_magnitude(double& y, double& z, double maximum) noexcept {
    const double magnitude = std::hypot(y, z);
    if (magnitude <= maximum) return false;
    if (maximum == 0) {
        y = 0;
        z = 0;
        return true;
    }
    const double scale = maximum / magnitude;
    y *= scale;
    z *= scale;
    return true;
}

/**
 * @brief 在离散过载网格上选择满足物理上限、最接近连续指令的向内量化结果。
 * @param y 镖体系 Y 指令，成功时写回量化值。
 * @param z 镖体系 Z 指令，成功时写回量化值。
 * @param step 量化步长，单位 g；0 表示不量化。
 * @param config 两级物理上限和重力配置。
 * @param trace 保留连续限幅标志，并记录量化导致的限幅。
 * @return 步长合法且存在满足上限的有限量化值时为 true。
 * @note 最近舍入超限时，每轴最多向零减一个 LSB 即覆盖必要的向内候选，工作量固定。
 */
bool quantize_overload(double& y, double& z, double step,
    const CommandMappingConfig& config, domain::CommandMappingTrace& trace) noexcept {
    if (step == 0.0) return true;
    if (step < 0.0 || !std::isfinite(step)) return false;
    const double scale = 1.0 / step;
    const double scaled_y = std::abs(y) * scale, scaled_z = std::abs(z) * scale;
    if (!std::isfinite(scale) || !std::isfinite(scaled_y) || !std::isfinite(scaled_z))
        return false;
    const double rounded_y = std::round(scaled_y), rounded_z = std::round(scaled_z);
    const auto signed_value = [scale](double index, double original) {
        return index == 0.0 ? 0.0 : std::copysign(index / scale, original); // 协议整数没有负零。
    };
    const double nearest_magnitude = std::hypot(rounded_y / scale, rounded_z / scale);
    const bool overload_limited = nearest_magnitude > config.maximum_command_overload;
    const bool acceleration_limited = nearest_magnitude * config.gravity > config.maximum_command_acceleration;
    trace.overload_limited |= overload_limited;
    trace.acceleration_limited |= acceleration_limited;
    if (!overload_limited && !acceleration_limited) {
        // 保留协议的半 LSB 舍入语义，不让浮点误差在等距候选间改选。
        y = signed_value(rounded_y, y);
        z = signed_value(rounded_z, z);
        return true;
    }
    double best_error = std::numeric_limits<double>::infinity();
    double best_y = 0.0, best_z = 0.0;
    for (int inward_y = 0; inward_y <= 1; ++inward_y) {
        for (int inward_z = 0; inward_z <= 1; ++inward_z) {
            const double candidate_y = signed_value(std::max(0.0, rounded_y - inward_y), y);
            const double candidate_z = signed_value(std::max(0.0, rounded_z - inward_z), z);
            const double magnitude = std::hypot(candidate_y, candidate_z);
            if (magnitude > config.maximum_command_overload ||
                magnitude * config.gravity > config.maximum_command_acceleration) continue;
            const double error = std::hypot(candidate_y - y, candidate_z - z);
            if (error < best_error) {
                best_error = error;
                best_y = candidate_y;
                best_z = candidate_z;
            }
        }
    }
    if (!std::isfinite(best_error)) return false;
    y = best_y;
    z = best_z;
    return true;
}
}

bool CommandOutputMapper::valid_config(const CommandMappingConfig& config) noexcept {
    return valid_control_channel(config.control_channel) &&
           std::isfinite(config.gravity) && config.gravity > 0 &&
           std::isfinite(config.maximum_command_overload) &&
           config.maximum_command_overload >= 0 &&
           !std::isnan(config.maximum_command_acceleration) &&
           config.maximum_command_acceleration >= 0 &&
           valid_debug_config(config.debug);
}

void CommandOutputMapper::reset_debug_state_locked() const noexcept {
    last_generation_.reset();
    first_target_detected_time_ns_.reset();
}

void CommandOutputMapper::reset() noexcept {
    std::lock_guard<std::mutex> lock(debug_mutex_);
    reset_debug_state_locked();
}

domain::CommandMappingResult CommandOutputMapper::map(
    const domain::BallisticOverloadG& request,
    const domain::CommandOutputContext& context) const noexcept {
    using Reason = domain::GuidanceInvalidReason;
    domain::CommandMappingResult result;
    auto& command = result.command;
    auto& trace = result.trace;
    command.source_frame_id = context.source_frame_id;
    command.generation = context.generation;
    const bool debug = config_.debug.mode != GuidanceDebugMode::disabled;
    const bool launch_delay = config_.debug.mode == GuidanceDebugMode::launch_delay;
    const bool request_launched = context.launched;
    const bool latest_imu_launched = context.latest_state &&
        context.latest_state->launch.phase == domain::LaunchPhase::launched;
    command.source_time = launch_delay && context.latest_state
                              ? context.latest_state->time
                              : context.source_time;
    // 最新 IMU 决定周期发射位；来源图像的发射状态单独约束识别和正常制导请求。
    command.launched = latest_imu_launched && context.generation == context.active_generation;
    command.target_found = !debug && request_launched && context.target_found && !context.expired;
    trace.emit_time_ns = context.emit_time_ns;

    // 零过载不代表未发射或未识别；协议标志由各调用路径独立决定。
    const auto zero_output_preserving_flags = [&](Reason reason) {
        command.valid = false;
        command.invalid_reason = reason;
        command.overload_g = {};
        trace.valid = false;
        trace.unlimited_body_overload.reset();
        trace.body_overload = {};
        trace.body_acceleration = {};
        trace.ballistic_acceleration = {};
        trace.command_overload = {};
        trace.command_phase = {};
        return result;
    };
    if (context.generation != context.active_generation) {
        command.launched = false;
        command.target_found = false;
        return zero_output_preserving_flags(Reason::stale_input);
    }
    if (context.latest_state &&
        (context.latest_state->launch.generation != context.active_generation || !latest_imu_launched)) {
        command.launched = false;
        command.target_found = false;
        return zero_output_preserving_flags(Reason::launch_not_confirmed);
    }
    if (!command.launched) {
        command.launched = false;
        command.target_found = false;
        return zero_output_preserving_flags(Reason::launch_not_confirmed);
    }
    if (!context.output_enabled) return zero_output_preserving_flags(Reason::stale_input);
    if (!valid()) return zero_output_preserving_flags(Reason::algorithm_rejected);
    const auto& latest_state = *context.latest_state;

    // 固定模式不依赖 PNG 请求有效性，触发条件分别取发射时间或视觉识别。
    if (debug) {
        if (context.emit_time_ns < 0) return zero_output_preserving_flags(Reason::stale_input);
        std::lock_guard<std::mutex> lock(debug_mutex_);
        if (last_generation_ != context.generation) {
            reset_debug_state_locked();
            last_generation_ = context.generation;
        }

        bool triggered = false;
        if (launch_delay) {
            const auto& launch_time = latest_state.launch.transition_time;
            if (!matching_time(launch_time, context.clock_domain) ||
                context.emit_time_ns < launch_time.monotonic_ns) {
                return zero_output_preserving_flags(Reason::stale_input);
            }
            const int64_t delay_ns =
                static_cast<int64_t>(config_.debug.trigger_delay_ms) * 1'000'000LL;
            const int64_t elapsed_ns = context.emit_time_ns - launch_time.monotonic_ns;
            if (elapsed_ns >= delay_ns) {
                if (config_.debug.duration_ms > 0) {
                    const int64_t duration_ns =
                        static_cast<int64_t>(config_.debug.duration_ms) * 1'000'000LL;
                    triggered = (elapsed_ns < delay_ns + duration_ns);
                } else {
                    triggered = true;
                }
            }
        } else if (config_.debug.mode == GuidanceDebugMode::target_detected) {
            // 识别须来自未过期的发射后图像；首帧速率无效不影响识别触发。
            if (request_launched && context.target_found && !context.expired &&
                !first_target_detected_time_ns_) {
                first_target_detected_time_ns_ = context.emit_time_ns;
            }
            if (first_target_detected_time_ns_) {
                if (context.emit_time_ns < *first_target_detected_time_ns_) {
                    return zero_output_preserving_flags(Reason::stale_input);
                }
                const int64_t elapsed_ns = context.emit_time_ns - *first_target_detected_time_ns_;
                if (config_.debug.duration_ms > 0) {
                    const int64_t duration_ns =
                        static_cast<int64_t>(config_.debug.duration_ms) * 1'000'000LL;
                    triggered = (elapsed_ns < duration_ns);
                } else {
                    triggered = true;
                }
            }
        }

        if (latest_state.attitude.valid) {
            trace.attitude = latest_state.attitude;
        }

        if (!triggered) {
            // 等待或窗口结束不是算法故障；识别模式在窗口结束后仍保留锁存位。
            command.target_found = first_target_detected_time_ns_.has_value();
            return zero_output_preserving_flags(Reason::none);
        }

        // 固定模式复用识别标志通知下位机执行，不要求当前图像仍检出目标。
        command.target_found = true;

        double y = config_.debug.pitch_overload_g;
        double z = config_.debug.yaw_overload_g;
        select_control_channel(config_.control_channel, y, z);
        trace.unlimited_body_overload = domain::DartOverloadG{0, y, z};
        trace.overload_limited =
            limit_overload_magnitude(y, z, config_.maximum_command_overload);
        trace.acceleration_limited = limit_overload_magnitude(
            y, z, config_.maximum_command_acceleration / config_.gravity);
        if (!quantize_overload(y, z, context.overload_step_g, config_, trace)) {
            return zero_output_preserving_flags(Reason::algorithm_rejected);
        }
        trace.body_overload = {0, y, z};
        trace.body_acceleration = {0, y * config_.gravity, z * config_.gravity};
        trace.command_overload.value = std::hypot(y, z);
        trace.command_phase.value = trace.command_overload.value == 0 ? 0 : std::atan2(z, y);
        trace.valid = true;
        command.valid = true;
        command.invalid_reason = Reason::none;
        command.overload_g = trace.body_overload;
        return result;
    }

    if (!request_launched) return zero_output_preserving_flags(Reason::launch_not_confirmed);
    if (context.expired) return zero_output_preserving_flags(Reason::stale_input);
    // 提交时间只检测生产断流，不能为曝光后排队或识别耗时重新授予有效期。
    if (!matching_time(context.source_time, context.clock_domain) ||
        context.emit_time_ns < context.source_time.monotonic_ns ||
        context.maximum_source_age_ns <= 0 ||
        context.emit_time_ns - context.source_time.monotonic_ns > context.maximum_source_age_ns) {
        command.target_found = false;
        return zero_output_preserving_flags(Reason::stale_input);
    }
    if (!context.request_valid) {
        return zero_output_preserving_flags(
            context.invalid_reason == Reason::none ? Reason::algorithm_rejected : context.invalid_reason);
    }
    if (request.x != 0 || !std::isfinite(request.y) || !std::isfinite(request.z)) {
        return zero_output_preserving_flags(Reason::algorithm_rejected);
    }
    trace.raw_navigation_overload = request;
    if (!latest_state.attitude.valid) return zero_output_preserving_flags(Reason::invalid_attitude);
    trace.attitude = latest_state.attitude;
    const auto& attitude_time = trace.attitude.time;
    if (!matching_time(attitude_time, context.clock_domain) ||
        !matching_time(latest_state.time, context.clock_domain) ||
        latest_state.time.monotonic_ns != attitude_time.monotonic_ns ||
        context.emit_time_ns < attitude_time.monotonic_ns ||
        context.maximum_attitude_age_ns <= 0 ||
        context.emit_time_ns - attitude_time.monotonic_ns > context.maximum_attitude_age_ns) {
        return zero_output_preserving_flags(Reason::stale_input);
    }
    const auto& m = trace.attitude.navigation_from_dart.matrix();
    if (!detail::valid_rotation(m)) return zero_output_preserving_flags(Reason::invalid_attitude);
    const double horizontal = std::hypot(m[4], m[5]);
    const double cosine = horizontal > 1e-6 ? m[4] / horizontal : 1.0;
    const double sine = horizontal > 1e-6 ? -m[5] / horizontal : 0.0;
    double y = cosine * request.y + sine * request.z;
    double z = -sine * request.y + cosine * request.z;
    select_control_channel(config_.control_channel, y, z);
    if (!std::isfinite(std::hypot(y, z))) return zero_output_preserving_flags(Reason::algorithm_rejected);
    trace.unlimited_body_overload = domain::DartOverloadG{0, y, z};
    trace.overload_limited = limit_overload_magnitude(y, z, config_.maximum_command_overload);
    trace.after_overload_limit = {0, cosine * y - sine * z, sine * y + cosine * z};
    trace.acceleration_limited = limit_overload_magnitude(
        y, z, config_.maximum_command_acceleration / config_.gravity);
    if (!quantize_overload(y, z, context.overload_step_g, config_, trace)) {
        return zero_output_preserving_flags(Reason::algorithm_rejected);
    }
    trace.body_overload = {0, y, z};
    trace.body_acceleration = {0, y * config_.gravity, z * config_.gravity};
    const double by = cosine * y - sine * z;
    const double bz = sine * y + cosine * z;
    trace.ballistic_acceleration = {0, by * config_.gravity, bz * config_.gravity};
    if (!std::isfinite(trace.body_acceleration.y) || !std::isfinite(trace.body_acceleration.z) ||
        !std::isfinite(trace.ballistic_acceleration.y) || !std::isfinite(trace.ballistic_acceleration.z)) {
        return zero_output_preserving_flags(Reason::algorithm_rejected);
    }
    trace.command_overload.value = std::hypot(y, z);
    trace.command_phase.value = trace.command_overload.value == 0 ? 0 : std::atan2(bz, by);
    trace.valid = true;
    command.valid = true;
    command.invalid_reason = Reason::none;
    command.overload_g = trace.body_overload;
    return result;
}
}
