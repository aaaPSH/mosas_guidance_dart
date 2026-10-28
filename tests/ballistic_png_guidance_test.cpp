#include <mosas/guidance/ballistic_png_guidance.hpp>

#include "body_vector_test_support.hpp"
#include "test_support.hpp"

#include <cmath>
#include <limits>

int main() {
    using namespace mosas;

    // 1. 偏航右手系符号验证：目标向右运动时产生向右的正过载
    {
        guidance::BallisticPngConfig config;
        config.navigation_constant_y = 3.0;
        config.navigation_constant_z = 3.0;
        config.gravity = 9.80665;
        config.maximum_command_overload = 10.0;
        config.maximum_command_acceleration = 100.0;
        config.control_channel = guidance::ControlChannel::both;
        guidance::BallisticPngGuidance guidance(config);

        domain::GuidanceInput input{};
        input.attitude = test::vector_test_attitude(0.0, 0.0, 0.0); // 无倾角无滚转
        input.speed_mps.value = 100.0;
        // 目标偏右，并向右移动
        input.vector_los.valid = true;
        input.vector_los.direction = {0.9, 0.0, 0.43588989435406735};
        input.vector_los.inertial_direction_rate = {-0.0484322, 0.0, 0.1}; // 满足切向正交

        auto output = guidance.calculate(input);
        CHECK(output.valid);
        CHECK(output.command.valid);
        // 向右移动必须产生向右 (Z 轴正方向) 的纠偏过载
        CHECK(output.command.overload_g.z > 0.5);
    }

    // 2. 抛物线补偿验证：仰角变化率等于自然下坠率且无横向偏转时，过载指令严格为零
    {
        guidance::BallisticPngConfig config;
        config.navigation_constant_y = 4.0;
        config.navigation_constant_z = 4.0;
        config.gravity = 9.80665;
        guidance::BallisticPngGuidance guidance(config);

        domain::GuidanceInput input{};
        input.attitude = test::vector_test_attitude();
        input.speed_mps.value = 50.0;

        const double rho_xz = 0.95;
        const double l_x = rho_xz;
        const double l_y = std::sqrt(1.0 - rho_xz * rho_xz);
        const double l_z = 0.0;

        // 理论自然下坠速率: -g * rho_xz / (2 * V)
        const double q_dot_para = -config.gravity * rho_xz / (2.0 * input.speed_mps.value);
        // dot_l_y = q_dot_para * rho_xz
        const double dot_l_y = q_dot_para * rho_xz;
        // dot_l_x 满足正交: l_x * dot_l_x + l_y * dot_l_y = 0
        const double dot_l_x = -l_y * dot_l_y / l_x;

        input.vector_los.valid = true;
        input.vector_los.direction = {l_x, l_y, l_z};
        input.vector_los.inertial_direction_rate = {dot_l_x, dot_l_y, 0.0};

        auto output = guidance.calculate(input);
        CHECK(output.valid);
        CHECK_NEAR(output.command.overload_g.y, 0.0, 1e-6);
        CHECK_NEAR(output.command.overload_g.z, 0.0, 1e-6);
    }

    // 3. 免三角函数 Roll 旋转映射等价性测试
    {
        guidance::BallisticPngConfig config;
        config.navigation_constant_y = 3.0;
        config.navigation_constant_z = 3.0;
        guidance::BallisticPngGuidance guidance(config);

        const double roll = 0.7853981633974483; // 45 度
        domain::GuidanceInput input{};
        input.attitude = test::vector_test_attitude(0.1, roll, 0.0);
        input.speed_mps.value = 80.0;
        input.vector_los.valid = true;
        input.vector_los.direction = {0.95, 0.0, 0.3122498999199199};
        input.vector_los.inertial_direction_rate = {0.0, 0.05, 0.0}; // 产生纵向过载

        auto output = guidance.calculate(input);
        CHECK(output.valid);

        // 与理论 atan2 + cos/sin 投影比较
        const auto& m = input.attitude.navigation_from_dart.matrix();
        const double phi = std::atan2(-m[5], m[4]); // m12, m11
        const double cos_phi = std::cos(phi);
        const double sin_phi = std::sin(phi);

        const double n_y_b = output.diagnostics.navigation_acceleration.y / config.gravity;
        const double n_z_b = output.diagnostics.navigation_acceleration.z / config.gravity;
        const double expected_y = cos_phi * n_y_b + sin_phi * n_z_b;
        const double expected_z = -sin_phi * n_y_b + cos_phi * n_z_b;

        CHECK_NEAR(output.command.overload_g.y, expected_y, 1e-10);
        CHECK_NEAR(output.command.overload_g.z, expected_z, 1e-10);
    }

    // 4. 万向节锁死极点保护测试 (cos_theta <= 1e-6)
    {
        guidance::BallisticPngConfig config;
        guidance::BallisticPngGuidance guidance(config);

        domain::GuidanceInput input{};
        // 俯仰 90 度垂直向上
        input.attitude = test::vector_test_attitude(1.5707963267948966, 0.0, 0.0);
        input.speed_mps.value = 80.0;
        input.vector_los.valid = true;
        input.vector_los.direction = {0.8, 0.0, 0.6};
        input.vector_los.inertial_direction_rate = {0.0, 0.05, 0.0};

        auto output = guidance.calculate(input);
        CHECK(output.valid);
        CHECK(std::isfinite(output.command.overload_g.y));
        CHECK(std::isfinite(output.command.overload_g.z));
    }

    // 5. 垂直视线极点保护测试 (rho_xz < 1e-6)
    {
        guidance::BallisticPngConfig config;
        guidance::BallisticPngGuidance guidance(config);

        domain::GuidanceInput input{};
        input.attitude = test::vector_test_attitude();
        input.speed_mps.value = 80.0;
        input.vector_los.valid = true;
        // 视线垂直直冲天顶
        input.vector_los.direction = {0.0, 1.0, 0.0};
        input.vector_los.inertial_direction_rate = {0.05, 0.0, 0.0};

        auto output = guidance.calculate(input);
        CHECK(!output.valid);
        CHECK(!output.command.valid);
    }

    // 6. 通道选择与限幅测试
    {
        guidance::BallisticPngConfig config;
        config.maximum_command_overload = 5.0; // 限幅 5g
        config.maximum_command_acceleration = 200.0;
        config.control_channel = guidance::ControlChannel::pitch; // 仅 pitch
        guidance::BallisticPngGuidance guidance(config);

        domain::GuidanceInput input{};
        input.attitude = test::vector_test_attitude();
        input.speed_mps.value = 100.0;
        input.vector_los.valid = true;
        input.vector_los.direction = {0.9, 0.0, 0.43588989};
        input.vector_los.inertial_direction_rate = {0.0, 0.5, 0.5}; // 产生大过载

        auto output = guidance.calculate(input);
        CHECK(output.valid);
        CHECK_NEAR(output.command.overload_g.z, 0.0, 1e-12); // yaw 关闭必须为 0
        CHECK(std::abs(output.command.overload_g.y) <= 5.0 + 1e-12); // pitch 受 5g 限幅
    }

    return 0;
}
