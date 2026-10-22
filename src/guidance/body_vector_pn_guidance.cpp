#include <mosas/guidance/body_vector_pn_guidance.hpp>

#include "body_vector_math.hpp"

#include <algorithm>
#include <cmath>

namespace mosas::guidance {

BodyVectorPnGuidance::BodyVectorPnGuidance(const BodyVectorPnConfig &config) : config_(config) {}

bool BodyVectorPnGuidance::valid_config(const BodyVectorPnConfig &c) noexcept {
    return std::isfinite(c.navigation_constant) && c.navigation_constant > 0.0 &&
           std::isfinite(c.gravity) && c.gravity > 0.0 &&
           std::isfinite(c.maximum_command_overload) && c.maximum_command_overload >= 0.0 &&
           !std::isnan(c.maximum_command_acceleration) && c.maximum_command_acceleration >= 0.0;
}

domain::GuidanceOutput BodyVectorPnGuidance::calculate(const domain::GuidanceInput &input) {
    domain::GuidanceOutput output;
    output.command.source_frame_id = input.source_frame_id;
    output.command.source_time = input.time;
    const auto reject = [&output](domain::GuidanceInvalidReason reason) {
        output.invalid_reason = reason;
        output.command.invalid_reason = reason;
        return output;
    };
    if (!valid_config(config_) || !std::isfinite(input.speed_mps.value) ||
        input.speed_mps.value <= 0.0) {
        return reject(domain::GuidanceInvalidReason::algorithm_rejected);
    }
    const auto rotation = detail::attitude_matrix(input.attitude);
    if (!input.attitude.valid || !detail::valid_rotation(rotation)) {
        return reject(domain::GuidanceInvalidReason::invalid_attitude);
    }
    const auto l = detail::components(input.vector_los.direction);
    const auto u = detail::components(input.vector_los.inertial_direction_rate);
    const double rate_norm = detail::norm(u);
    const double radial_rate = detail::dot(l, u);
    if (!input.vector_los.valid || !detail::finite(l) || !detail::finite(u) ||
        !std::isfinite(rate_norm) || !std::isfinite(radial_rate) ||
        std::abs(detail::norm(l) - 1.0) > 1e-6 ||
        std::abs(radial_rate) > 1e-6 * std::max(1.0, rate_norm)) {
        return reject(domain::GuidanceInvalidReason::invalid_rate);
    }

    // PNG 的 q_dot_parabola=-g*cos(q_N)/(2V) 等价于下式单位方向导数。
    // 在导航系构造切向基准，再完整旋转到镖体系；极点不需要除以 cos(q_N)。
    const auto navigation_los = detail::apply(rotation, l);
    const double half_gravity_over_speed = 0.5 * config_.gravity / input.speed_mps.value;
    const detail::Vec3 parabolic_rate_navigation{
        half_gravity_over_speed * navigation_los[1] * navigation_los[0],
        -half_gravity_over_speed * (1.0 - navigation_los[1] * navigation_los[1]),
        half_gravity_over_speed * navigation_los[1] * navigation_los[2]};
    const auto parabolic_rate_dart =
        detail::apply(detail::transpose(rotation), parabolic_rate_navigation);
    const detail::Vec3 compensated_rate{u[0] - parabolic_rate_dart[0],
                                        u[1] - parabolic_rate_dart[1],
                                        u[2] - parabolic_rate_dart[2]};
    if (!std::isfinite(half_gravity_over_speed) || !detail::finite(compensated_rate)) {
        return reject(domain::GuidanceInvalidReason::algorithm_rejected);
    }

    const auto omega = detail::cross(l, compensated_rate);
    const auto pn_direction = detail::cross(omega, {1.0, 0.0, 0.0});
    const double gain = config_.navigation_constant * input.speed_mps.value;
    // 只对补偿后的惯性 LOS 速率施加 PN 增益，不在输出端再次叠加重力。
    detail::Vec3 specific_force{0.0, gain * pn_direction[1], gain * pn_direction[2]};
    if (!std::isfinite(gain) || !detail::finite(specific_force)) {
        return reject(domain::GuidanceInvalidReason::algorithm_rejected);
    }
    detail::Vec3 overload{0.0, specific_force[1] / config_.gravity,
                          specific_force[2] / config_.gravity};
    double magnitude = detail::norm(overload);
    if (!detail::finite(overload) || !std::isfinite(magnitude)) {
        return reject(domain::GuidanceInvalidReason::algorithm_rejected);
    }
    const double limit = std::min(config_.maximum_command_overload,
                                  config_.maximum_command_acceleration / config_.gravity);
    if (magnitude > limit) {
        const double scale = limit / magnitude;
        for (std::size_t i = 1; i < 3; ++i) {
            overload[i] *= scale;
            specific_force[i] *= scale;
        }
        magnitude = std::hypot(overload[1], overload[2]);
    }

    // 旧诊断字段属于弹道系：只为记录转换，不用于生成镖体系命令。
    const double roll = std::atan2(-rotation[1][2], rotation[1][1]);
    const double c = std::cos(roll), s = std::sin(roll);
    const double ballistic_y = c * specific_force[1] - s * specific_force[2];
    const double ballistic_z = s * specific_force[1] + c * specific_force[2];
    if (!std::isfinite(ballistic_y) || !std::isfinite(ballistic_z)) {
        return reject(domain::GuidanceInvalidReason::algorithm_rejected);
    }
    output.valid = output.command.valid = true;
    output.invalid_reason = output.command.invalid_reason = domain::GuidanceInvalidReason::none;
    output.command.overload_g = {0.0, overload[1], overload[2]};
    output.diagnostics.body_overload = output.command.overload_g;
    output.diagnostics.body_acceleration = {0.0, specific_force[1], specific_force[2]};
    output.diagnostics.navigation_acceleration = {0.0, ballistic_y, ballistic_z};
    output.diagnostics.command_overload.value = magnitude;
    output.diagnostics.command_phase.value =
        magnitude == 0.0 ? 0.0 : std::atan2(ballistic_z, ballistic_y);
    return output;
}

} // namespace mosas::guidance
