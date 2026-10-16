#include <mosas/guidance/body_vector_los_filter.hpp>

#include "body_vector_test_support.hpp"
#include "test_support.hpp"

#include <cmath>
#include <limits>

int main() {
    using namespace mosas;
    guidance::BodyVectorLosFilter filter;
    domain::LosRateFilterInput input;
    input.line_of_sight = {true, {0.0}, {0.0}};
    input.attitude = test::vector_test_attitude();
    input.dt_seconds = 1.0; // 首帧不按占位时间传播，也不因超过最大间隔拒绝建基准。
    CHECK(!filter.update(input).valid);
    input.dt_seconds = 0.01;
    auto estimate = filter.update(input);
    CHECK(estimate.valid && estimate.vector.valid && estimate.angular_velocity.valid);
    CHECK_NEAR(estimate.vector.direction.x, 1.0, 1e-12);
    CHECK_NEAR(estimate.vector.inertial_direction_rate.y, 0.0, 1e-12);
    CHECK_NEAR(estimate.angular_velocity.q_y.value, 0.0, 1e-12);

    filter.reset();
    for (int i = 0; i < 200; ++i) {
        input.line_of_sight.q_y.value = 0.2 * i * input.dt_seconds;
        estimate = filter.update(input);
    }
    CHECK(estimate.valid);
    CHECK_NEAR(estimate.angular_velocity.q_y.value, 0.2, 0.003);
    const auto l = estimate.vector.direction;
    const auto u = estimate.vector.inertial_direction_rate;
    CHECK_NEAR(std::hypot(std::hypot(l.x, l.y), l.z), 1.0, 1e-12);
    CHECK_NEAR(l.x * u.x + l.y * u.y + l.z * u.z, 0.0, 1e-12);

    // 纯转动：测得的镖体系 LOS 改变，但惯性 LOS 固定。
    filter.reset();
    const std::array<double, 3> fixed_los{0.8, 0.36, 0.48};
    for (int i = 0; i < 250; ++i) {
        input.attitude = test::vector_test_attitude(0.2 * std::sin(0.02 * i),
                                                    0.8 * std::sin(0.015 * i), 0.003 * i);
        input.line_of_sight = test::test_los(test::to_test_dart(input.attitude, fixed_los));
        estimate = filter.update(input);
        if (i == 0) {
            CHECK(!estimate.valid);
            continue;
        }
        CHECK(estimate.valid);
        CHECK_NEAR(estimate.vector.inertial_direction_rate.x, 0.0, 1e-10);
        CHECK_NEAR(estimate.vector.inertial_direction_rate.y, 0.0, 1e-10);
        CHECK_NEAR(estimate.vector.inertial_direction_rate.z, 0.0, 1e-10);
    }

    // 各向同性矢量滤波：相同物理测量序列在不同滚转坐标下给出一致结果。
    guidance::BodyVectorLosFilter reference_filter;
    guidance::BodyVectorLosFilter rolling_filter;
    for (int i = 0; i < 400; ++i) {
        domain::LosRateFilterInput reference_input;
        reference_input.attitude = test::vector_test_attitude();
        reference_input.dt_seconds = 0.01;
        reference_input.line_of_sight = {
            true, {0.001 * i + 0.002 * std::sin(1.7 * i)}, {0.1 + 0.003 * std::cos(1.1 * i)}};
        const double qy = reference_input.line_of_sight.q_y.value;
        const double qz = reference_input.line_of_sight.q_z.value;
        const std::array<double, 3> direction{std::cos(qy) * std::cos(qz), std::sin(qy),
                                              std::cos(qy) * std::sin(qz)};
        auto rolling_input = reference_input;
        rolling_input.attitude = test::vector_test_attitude(0.0, 1.2 * std::sin(0.02 * i));
        rolling_input.line_of_sight =
            test::test_los(test::to_test_dart(rolling_input.attitude, direction));
        const auto a = reference_filter.update(reference_input);
        const auto b = rolling_filter.update(rolling_input);
        CHECK(a.valid == b.valid);
        if (!a.valid)
            continue;
        const auto bl = b.vector.direction;
        const auto bu = b.vector.inertial_direction_rate;
        const auto physical_l =
            test::to_test_navigation(rolling_input.attitude, {bl.x, bl.y, bl.z});
        const auto physical_u =
            test::to_test_navigation(rolling_input.attitude, {bu.x, bu.y, bu.z});
        CHECK_NEAR(physical_l[0], a.vector.direction.x, 1e-10);
        CHECK_NEAR(physical_l[1], a.vector.direction.y, 1e-10);
        CHECK_NEAR(physical_l[2], a.vector.direction.z, 1e-10);
        CHECK_NEAR(physical_u[0], a.vector.inertial_direction_rate.x, 1e-10);
        CHECK_NEAR(physical_u[1], a.vector.inertial_direction_rate.y, 1e-10);
        CHECK_NEAR(physical_u[2], a.vector.inertial_direction_rate.z, 1e-10);
    }

    // 确定性高频视觉噪声：比较滤波速率与原始差分，而非比较过载平滑度。
    filter.reset();
    input.attitude = test::vector_test_attitude();
    double raw_error = 0.0, filtered_error = 0.0;
    for (int i = 0; i < 500; ++i) {
        input.line_of_sight = {true, {0.003 * std::sin(1.7 * i)}, {0.003 * std::cos(1.1 * i)}};
        estimate = filter.update(input);
        if (i < 50)
            continue;
        CHECK(estimate.valid);
        const auto raw = filter.sample().raw_angular_velocity;
        CHECK(raw.valid);
        raw_error += raw.q_y.value * raw.q_y.value + raw.q_z.value * raw.q_z.value;
        const auto filtered = estimate.angular_velocity;
        filtered_error +=
            filtered.q_y.value * filtered.q_y.value + filtered.q_z.value * filtered.q_z.value;
    }
    CHECK(filtered_error < 0.25 * raw_error);

    filter.reset();
    const double pi = std::acos(-1.0);
    input.line_of_sight = {true, {pi / 2.0}, {0.0}};
    CHECK(!filter.update(input).valid);
    estimate = filter.update(input);
    CHECK(estimate.valid && estimate.vector.valid);
    CHECK(!estimate.angular_velocity.valid);
    CHECK_NEAR(estimate.vector.direction.y, 1.0, 1e-12);

    input.dt_seconds = 0.3;
    CHECK(!filter.update(input).valid);
    input.dt_seconds = 0.01;
    CHECK(filter.update(input).valid);
    ++input.generation;
    CHECK(!filter.update(input).valid);
    CHECK(filter.update(input).valid);
    input.attitude.valid = false;
    CHECK(!filter.update(input).valid);
    input.attitude = test::vector_test_attitude();
    CHECK(!filter.update(input).valid);
    CHECK(filter.update(input).valid);
    input.line_of_sight.valid = false;
    CHECK(!filter.update(input).valid);
    CHECK(!filter.sample().line_of_sight.valid);
    input.line_of_sight.valid = true;
    CHECK(!filter.update(input).valid);
    CHECK(filter.update(input).valid);
    input.dt_seconds = -0.01;
    CHECK(!filter.update(input).valid);
    input.dt_seconds = 0.01;
    CHECK(!filter.update(input).valid);
    input.dt_seconds = 0.0;
    CHECK(!filter.update(input).valid);
    input.dt_seconds = 0.01;
    CHECK(!filter.update(input).valid);
    CHECK(filter.update(input).valid);
    input.attitude.navigation_from_dart =
        domain::Rotation3<domain::NavigationFrameTag, domain::DartFrameTag>(
            {-1, 0, 0, 0, 1, 0, 0, 0, 1});
    CHECK(!filter.update(input).valid);
    input.attitude = test::vector_test_attitude();
    input.line_of_sight.q_y.value = std::numeric_limits<double>::quiet_NaN();
    CHECK(!filter.update(input).valid);
    input.line_of_sight.q_y.value = 0.0;
    input.dt_seconds = std::numeric_limits<double>::infinity();
    CHECK(!filter.update(input).valid);

    guidance::BodyVectorLosFilterConfig config;
    CHECK(guidance::BodyVectorLosFilter::valid_config(config));
    config.sigma_l = 0.0;
    CHECK(!guidance::BodyVectorLosFilter::valid_config(config));
    input.dt_seconds = 0.01;
    CHECK(!guidance::BodyVectorLosFilter(config).update(input).valid);
    config.sigma_l = 1e-200;
    CHECK(!guidance::BodyVectorLosFilter::valid_config(config));
    config.sigma_l = 0.0035;
    config.sigma_a = std::numeric_limits<double>::infinity();
    CHECK(!guidance::BodyVectorLosFilter::valid_config(config));
    return 0;
}
