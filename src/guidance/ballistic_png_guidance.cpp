#include <mosas/guidance/ballistic_png_guidance.hpp>

#include <algorithm>
#include <cmath>

namespace mosas::guidance {
namespace {

constexpr double kEpsilon = 1e-12;
constexpr double kPoleThreshold = 1e-6;

bool finite(double v) noexcept { return std::isfinite(v); }

bool valid_rotation_matrix(const std::array<double, 9>& m) noexcept {
    for (double v : m) {
        if (!finite(v)) return false;
    }
    const double r0 = std::hypot(std::hypot(m[0], m[1]), m[2]);
    const double r1 = std::hypot(std::hypot(m[3], m[4]), m[5]);
    const double r2 = std::hypot(std::hypot(m[6], m[7]), m[8]);
    if (std::abs(r0 - 1.0) > 1e-4 || std::abs(r1 - 1.0) > 1e-4 || std::abs(r2 - 1.0) > 1e-4) {
        return false;
    }
    return true;
}

}  // namespace

BallisticPngGuidance::BallisticPngGuidance(const BallisticPngConfig& config)
    : config_(valid_config(config) ? config : BallisticPngConfig{}) {}

bool BallisticPngGuidance::valid_config(const BallisticPngConfig& c) noexcept {
    return valid_control_channel(c.control_channel) &&
           finite(c.navigation_constant_y) && c.navigation_constant_y > 0.0 &&
           finite(c.navigation_constant_z) && c.navigation_constant_z > 0.0 &&
           finite(c.gravity) && c.gravity > 0.0 &&
           finite(c.maximum_command_overload) && c.maximum_command_overload >= 0.0 &&
           !std::isnan(c.maximum_command_acceleration) && c.maximum_command_acceleration >= 0.0;
}

domain::GuidanceOutput BallisticPngGuidance::calculate(
    const domain::GuidanceInput& input) {
    domain::GuidanceOutput output{};
    output.command.source_frame_id = input.source_frame_id;
    output.command.source_time = input.time;

    const auto reject = [&output](domain::GuidanceInvalidReason reason) {
        output.valid = false;
        output.invalid_reason = reason;
        output.command.valid = false;
        output.command.invalid_reason = reason;
        return output;
    };

    if (!valid_config(config_) || !finite(input.speed_mps.value) ||
        input.speed_mps.value <= kEpsilon) {
        return reject(domain::GuidanceInvalidReason::algorithm_rejected);
    }

    if (!input.attitude.valid) {
        return reject(domain::GuidanceInvalidReason::invalid_attitude);
    }

    const auto& m = input.attitude.navigation_from_dart.matrix();
    if (!valid_rotation_matrix(m)) {
        return reject(domain::GuidanceInvalidReason::invalid_attitude);
    }

    if (!input.vector_los.valid) {
        return reject(domain::GuidanceInvalidReason::invalid_los);
    }

    const auto& l_D = input.vector_los.direction;
    const auto& u_D = input.vector_los.inertial_direction_rate;
    if (!finite(l_D.x) || !finite(l_D.y) || !finite(l_D.z) ||
        !finite(u_D.x) || !finite(u_D.y) || !finite(u_D.z)) {
        return reject(domain::GuidanceInvalidReason::invalid_los);
    }

    // 1. 将镖体系视线向量与导数投影到局部导航惯性系 (l_N = R_{N<-D} * l_D)
    const std::array<double, 3> l_N{
        m[0] * l_D.x + m[1] * l_D.y + m[2] * l_D.z,
        m[3] * l_D.x + m[4] * l_D.y + m[5] * l_D.z,
        m[6] * l_D.x + m[7] * l_D.y + m[8] * l_D.z};

    const std::array<double, 3> u_N{
        m[0] * u_D.x + m[1] * u_D.y + m[2] * u_D.z,
        m[3] * u_D.x + m[4] * u_D.y + m[5] * u_D.z,
        m[6] * u_D.x + m[7] * u_D.y + m[8] * u_D.z};

    // 2. 惯性角速率解析式与极点保护
    const double rho_xz = std::hypot(l_N[0], l_N[2]);
    if (rho_xz < kPoleThreshold) {
        return reject(domain::GuidanceInvalidReason::invalid_rate);
    }

    const double q_dot_y_N = u_N[1] / rho_xz;
    // 横向视线角速度（右手系修正：Omega_y = l_z * u_x - l_x * u_z = -(l_x * u_z - l_z * u_x)）
    const double q_dot_lateral_N = (l_N[0] * u_N[2] - l_N[2] * u_N[0]) / rho_xz;

    // 3. 自然重力抛物线补偿
    const double q_dot_parabola = -config_.gravity * rho_xz / (2.0 * input.speed_mps.value);
    const double q_dot_y_comp = q_dot_y_N - q_dot_parabola;

    // 4. 弹道系比例导引过载 (单位: g)
    const double speed_over_g = input.speed_mps.value / config_.gravity;
    const double n_y_B = speed_over_g * config_.navigation_constant_y * q_dot_y_comp;
    const double n_z_B = speed_over_g * config_.navigation_constant_z * q_dot_lateral_N;

    if (!finite(n_y_B) || !finite(n_z_B)) {
        return reject(domain::GuidanceInvalidReason::algorithm_rejected);
    }

    // 记录弹道系加速度诊断 (m/s²)
    output.diagnostics.navigation_acceleration.x = 0.0;
    output.diagnostics.navigation_acceleration.y = n_y_B * config_.gravity;
    output.diagnostics.navigation_acceleration.z = n_z_B * config_.gravity;

    // 5. 免三角函数 Roll 旋转映射到镖体系 (使用 m_11=m[4], m_12=m[5])
    const double cos_theta = std::hypot(m[4], m[5]);
    double n_y_D = n_y_B;
    double n_z_D = n_z_B;

    if (cos_theta > kPoleThreshold) {
        // [cos phi, sin phi] = [m_11 / cos_theta, -m_12 / cos_theta]
        // [n_y_D]   [ cos phi  sin phi] [n_y_B]   1 / cos_theta * [ m_11  -m_12] [n_y_B]
        // [n_z_D] = [-sin phi  cos phi] [n_z_B] =                 [ m_12   m_11] [n_z_B]
        const double inv_cos_theta = 1.0 / cos_theta;
        n_y_D = (m[4] * n_y_B - m[5] * n_z_B) * inv_cos_theta;
        n_z_D = (m[5] * n_y_B + m[4] * n_z_B) * inv_cos_theta;
    }
    // else: 万向节锁死极点保护回退，直接保持 n_y_D = n_y_B, n_z_D = n_z_B，避免除零 NaN

    // 6. 控制通道选择 (先 Roll 转换后再清零未启用通道)
    select_control_channel(config_.control_channel, n_y_D, n_z_D);

    // 7. 合成过载与物理加速度限幅截断
    const double limit_g = std::min(config_.maximum_command_overload,
                                    config_.maximum_command_acceleration / config_.gravity);
    const double mag = std::hypot(n_y_D, n_z_D);
    if (mag > limit_g && mag > kEpsilon) {
        const double scale = limit_g / mag;
        n_y_D *= scale;
        n_z_D *= scale;
    }

    // 8. 填充最终命令与诊断
    output.valid = true;
    output.invalid_reason = domain::GuidanceInvalidReason::none;
    output.command.valid = true;
    output.command.invalid_reason = domain::GuidanceInvalidReason::none;
    output.command.overload_g = {0.0, n_y_D, n_z_D};

    output.diagnostics.body_overload = {0.0, n_y_D, n_z_D};
    output.diagnostics.body_acceleration = {0.0, n_y_D * config_.gravity, n_z_D * config_.gravity};
    output.diagnostics.command_overload.value = std::hypot(n_y_D, n_z_D);
    output.diagnostics.command_phase.value = std::atan2(n_y_D, n_z_D);

    return output;
}

}  // namespace mosas::guidance
