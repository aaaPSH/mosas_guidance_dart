#include <mosas/guidance/inertial_vector_los_filter.hpp>

#include "body_vector_test_support.hpp"
#include "test_support.hpp"

#include <cmath>
#include <limits>

namespace {
using Matrix2 = std::array<std::array<double, 2>, 2>;

/** @brief 用通用矩阵乘法计算 Joseph 参考值，独立于生产标量更新。 */
Matrix2 joseph_reference(const Matrix2& prior, double k0, double k1, double r) {
    const Matrix2 a{{{{1.0 - k0, 0.0}}, {{-k1, 1.0}}}};
    const std::array<double, 2> k{k0, k1};
    Matrix2 result{};
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j) {
            for (int p = 0; p < 2; ++p)
                for (int q = 0; q < 2; ++q)
                    result[i][j] += a[i][p] * prior[p][q] * a[j][q];
            result[i][j] += k[i] * r * k[j];
        }
    return result;
}

/** @brief 用单位化 Jacobian 计算参考切向导数。 */
void reference_project(std::array<double, 3>& l, std::array<double, 3>& u) {
    const double n = std::hypot(std::hypot(l[0], l[1]), l[2]);
    const double dot = l[0] * u[0] + l[1] * u[1] + l[2] * u[2];
    for (int i = 0; i < 3; ++i) {
        u[i] = u[i] / n - l[i] * dot / (n * n * n);
        l[i] /= n;
    }
}

/** @return 正确协方差、两次投影及拒绝路径均符合参考时返回 0。 */
int verify_numerical_regressions() {
    using namespace mosas;
    guidance::InertialVectorLosFilter filter;
    domain::LosRateFilterInput input;
    input.attitude = test::vector_test_attitude();
    input.line_of_sight = {true, {0.0}, {0.0}};
    input.dt_seconds = 0.01;
    CHECK(!filter.update(input).valid);
    CHECK(filter.update(input).valid);
    CHECK_NEAR(filter.covariance().p01, 9.84671349724e-4, 1e-14);
    CHECK_NEAR(filter.covariance().p11, 0.197681886603, 1e-12);

    filter.reset();
    CHECK(!filter.update(input).valid);
    const double r = 0.0035 * 0.0035;
    Matrix2 covariance{{{{r, 0.0}}, {{0.0, 1.0}}}};
    std::array<double, 3> l{1.0, 0.0, 0.0}, u{};
    constexpr std::array<double, 4> intervals{0.001, 0.008, 0.02, 0.05};
    for (int step = 0; step < 80; ++step) {
        const double dt = intervals[step % intervals.size()];
        input.dt_seconds = dt;
        input.line_of_sight = {true, {0.3 * std::sin(step * 0.2)}, {0.1 * std::cos(step * 0.1)}};
        const double elevation = input.line_of_sight.q_y.value;
        const double azimuth = input.line_of_sight.q_z.value;
        const std::array<double, 3> z{std::cos(elevation) * std::cos(azimuth),
                                     std::sin(elevation), std::cos(elevation) * std::sin(azimuth)};
        for (int i = 0; i < 3; ++i) l[i] += dt * u[i];
        reference_project(l, u);
        const Matrix2 f{{{{1.0, dt}}, {{0.0, 1.0}}}};
        const std::array<double, 2> b{dt * dt / 2.0, dt};
        Matrix2 prior{};
        for (int i = 0; i < 2; ++i)
            for (int j = 0; j < 2; ++j) {
                for (int p = 0; p < 2; ++p)
                    for (int q = 0; q < 2; ++q)
                        prior[i][j] += f[i][p] * covariance[p][q] * f[j][q];
                prior[i][j] += 25.0 * b[i] * b[j];
            }
        const double k0 = prior[0][0] / (prior[0][0] + r);
        const double k1 = prior[1][0] / (prior[0][0] + r);
        for (int i = 0; i < 3; ++i) {
            const double innovation = z[i] - l[i];
            l[i] += k0 * innovation;
            u[i] += k1 * innovation;
        }
        reference_project(l, u);
        covariance = joseph_reference(prior, k0, k1, r);
        const auto estimate = filter.update(input);
        CHECK(estimate.valid);
        CHECK_NEAR(filter.covariance().p00, covariance[0][0], 1e-13);
        CHECK_NEAR(filter.covariance().p01, covariance[0][1], 1e-13);
        CHECK_NEAR(filter.covariance().p11, covariance[1][1], 1e-12);
        CHECK(filter.covariance().p00 * filter.covariance().p11 >
              filter.covariance().p01 * filter.covariance().p01);
        CHECK_NEAR(estimate.vector.direction.x, l[0], 1e-11);
        CHECK_NEAR(estimate.vector.direction.y, l[1], 1e-11);
        CHECK_NEAR(estimate.vector.direction.z, l[2], 1e-11);
        CHECK_NEAR(estimate.vector.inertial_direction_rate.x, u[0], 1e-10);
        CHECK_NEAR(estimate.vector.inertial_direction_rate.y, u[1], 1e-10);
        CHECK_NEAR(estimate.vector.inertial_direction_rate.z, u[2], 1e-10);
    }
    input.dt_seconds = 0.0;
    CHECK(!filter.update(input).valid);
    input.dt_seconds = 0.01;
    input.attitude.navigation_from_dart = domain::Rotation3<domain::NavigationFrameTag, domain::DartFrameTag>(
        {1, 0, 0, 1, 0, 0, 0, 0, 1});
    CHECK(!filter.update(input).valid);
    input.attitude.navigation_from_dart = domain::Rotation3<domain::NavigationFrameTag, domain::DartFrameTag>(
        {-1, 0, 0, 0, 1, 0, 0, 0, 1});
    CHECK(!filter.update(input).valid);
    guidance::InertialVectorLosFilterConfig invalid;
    invalid.initial_rate_covariance = 0.0;
    CHECK(!guidance::InertialVectorLosFilter::valid_config(invalid));
    invalid.initial_rate_covariance = 1.0;
    invalid.sigma_l = std::numeric_limits<double>::min();
    CHECK(!guidance::InertialVectorLosFilter::valid_config(invalid));
    return 0;
}
}  // namespace

int main() {
    using namespace mosas;
    CHECK(verify_numerical_regressions() == 0);

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
