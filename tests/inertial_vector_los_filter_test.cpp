#include <mosas/guidance/inertial_vector_los_filter.hpp>

#include "body_vector_test_support.hpp"
#include "test_support.hpp"

#include <cmath>
#include <limits>

int main() {
    using namespace mosas;

    // 1. 首帧初始化：输入有效视线与姿态，验证建立基准、首帧不输出有效速率
    {
        guidance::InertialVectorLosFilter filter;
        domain::LosRateFilterInput input;
        input.line_of_sight = {true, {0.0}, {0.0}};
        input.attitude = test::vector_test_attitude();
        input.dt_seconds = 1.0; // 首帧不按占位时间传播

        auto first_estimate = filter.update(input);
        CHECK(!first_estimate.valid);

        // 第二帧输入相同静态视线，开始输出有效速率
        input.dt_seconds = 0.01;
        auto second_estimate = filter.update(input);
        CHECK(second_estimate.valid);
        CHECK(second_estimate.angular_velocity.valid);
        CHECK_NEAR(second_estimate.angular_velocity.q_y.value, 0.0, 1e-6);
        CHECK_NEAR(second_estimate.angular_velocity.q_z.value, 0.0, 1e-6);
    }

    // 2. 纯机体转动抗扰测试：惯性目标固定，镖体剧烈滚转/俯仰，滤波惯性角速度应收敛为 0
    {
        guidance::InertialVectorLosFilter filter;
        const std::array<double, 3> fixed_inertial_los{0.8, 0.36, 0.48};
        domain::LosRateFilterInput input;
        input.dt_seconds = 0.01;

        for (int i = 0; i < 200; ++i) {
            // 飞镖持续滚转（高角速度 5 rad/s）与俯仰摆动
            input.attitude = test::vector_test_attitude(
                0.2 * std::sin(0.05 * i),
                5.0 * i * input.dt_seconds,
                0.02 * i * input.dt_seconds);
            // 投影到镖体系测得的机体视线
            input.line_of_sight = test::test_los(test::to_test_dart(input.attitude, fixed_inertial_los));

            auto estimate = filter.update(input);
            if (i == 0) {
                CHECK(!estimate.valid);
                continue;
            }
            CHECK(estimate.valid);
            if (i > 50) {
                // 经过滤波器收敛后，惯性视线导数应严格趋近于 0
                CHECK_NEAR(estimate.vector.inertial_direction_rate.x, 0.0, 1e-4);
                CHECK_NEAR(estimate.vector.inertial_direction_rate.y, 0.0, 1e-4);
                CHECK_NEAR(estimate.vector.inertial_direction_rate.z, 0.0, 1e-4);
                CHECK_NEAR(estimate.angular_velocity.q_y.value, 0.0, 1e-4);
                CHECK_NEAR(estimate.angular_velocity.q_z.value, 0.0, 1e-4);
            }
        }
    }

    // 3. 动态角速度跟踪测试：惯性目标以恒定 0.2 rad/s 俯仰角速度运动
    {
        guidance::InertialVectorLosFilter filter;
        domain::LosRateFilterInput input;
        input.attitude = test::vector_test_attitude(); // 镖体静止
        input.dt_seconds = 0.01;

        domain::LosRateEstimate estimate;
        for (int i = 0; i < 200; ++i) {
            input.line_of_sight = {true, {0.2 * i * input.dt_seconds}, {0.0}};
            estimate = filter.update(input);
        }
        CHECK(estimate.valid);
        CHECK_NEAR(estimate.angular_velocity.q_y.value, 0.2, 0.005);
        CHECK_NEAR(estimate.angular_velocity.q_z.value, 0.0, 0.005);
    }

    // 4. 流形单位化与正交切向约束测试：任何时刻必须严格满足模长=1且正交
    {
        guidance::InertialVectorLosFilter filter;
        domain::LosRateFilterInput input;
        input.attitude = test::vector_test_attitude(0.1, 0.3, -0.2);
        input.dt_seconds = 0.02;

        for (int i = 0; i < 100; ++i) {
            input.line_of_sight = {true, {0.1 * std::sin(0.1 * i)}, {0.15 * std::cos(0.08 * i)}};
            auto est = filter.update(input);
            if (est.valid) {
                const auto l = est.vector.direction;
                const auto u = est.vector.inertial_direction_rate;
                const double l_norm = std::hypot(std::hypot(l.x, l.y), l.z);
                const double dot_prod = l.x * u.x + l.y * u.y + l.z * u.z;
                CHECK_NEAR(l_norm, 1.0, 1e-12);
                CHECK_NEAR(dot_prod, 0.0, 1e-12);
            }
        }
    }

    // 5. 超时断流保护测试：dt > maximum_gap_seconds 触发 reset
    {
        guidance::InertialVectorLosFilterConfig config;
        config.maximum_gap_seconds = 0.05; // 50ms 超时
        guidance::InertialVectorLosFilter filter(config);

        domain::LosRateFilterInput input;
        input.line_of_sight = {true, {0.1}, {0.1}};
        input.attitude = test::vector_test_attitude();
        input.dt_seconds = 0.01;

        filter.update(input); // frame 0 -> baseline
        auto est1 = filter.update(input); // frame 1 -> valid
        CHECK(est1.valid);

        // 模拟丢帧 100ms > 50ms
        input.dt_seconds = 0.10;
        auto est_gap = filter.update(input);
        CHECK(!est_gap.valid); // 应该重置，重新作为首帧基准

        // 下一帧恢复正常
        input.dt_seconds = 0.01;
        auto est_resumed = filter.update(input);
        CHECK(est_resumed.valid);
    }

    // 6. 非法输入保护
    {
        guidance::InertialVectorLosFilter filter;
        domain::LosRateFilterInput input;
        input.line_of_sight = {false, {0.0}, {0.0}};
        input.attitude = test::vector_test_attitude();
        input.dt_seconds = 0.01;
        CHECK(!filter.update(input).valid);

        input.line_of_sight.valid = true;
        input.attitude.valid = false;
        CHECK(!filter.update(input).valid);

        input.attitude.valid = true;
        input.dt_seconds = -0.01;
        CHECK(!filter.update(input).valid);
    }

    return 0;
}
