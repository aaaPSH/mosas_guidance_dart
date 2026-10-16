#include <mosas/guidance/body_vector_pn_guidance.hpp>

#include "body_vector_test_support.hpp"
#include "test_support.hpp"

#include <cmath>
#include <limits>

int main() {
    using namespace mosas;
    guidance::BodyVectorPnConfig config;
    config.maximum_command_overload = 100.0;
    config.maximum_command_acceleration = std::numeric_limits<double>::infinity();
    guidance::BodyVectorPnGuidance law(config);
    domain::GuidanceInput input;
    input.source_frame_id = 42;
    input.time.monotonic_ns = 123;
    input.attitude = test::vector_test_attitude();
    input.speed_mps.value = 13.0;
    input.vector_los = {true, {1.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};

    auto output = law.calculate(input);
    CHECK(output.valid);
    CHECK(output.invalid_reason == domain::GuidanceInvalidReason::none);
    CHECK(output.command.valid);
    CHECK(output.command.source_frame_id == 42);
    CHECK(output.command.source_time.monotonic_ns == 123);
    CHECK_NEAR(output.command.overload_g.x, 0.0, 1e-12);
    CHECK_NEAR(output.command.overload_g.y, 1.0, 1e-12);
    CHECK_NEAR(output.command.overload_g.z, 0.0, 1e-12);
    // 新 PN 不依赖标量 LOS/角速率诊断是否有效。
    CHECK(!input.line_of_sight.valid && !input.angular_velocity.valid);

    const double pi = std::acos(-1.0);
    input.attitude = test::vector_test_attitude(0.0, pi / 2.0);
    output = law.calculate(input);
    CHECK_NEAR(output.command.overload_g.y, 0.0, 1e-12);
    CHECK_NEAR(output.command.overload_g.z, -1.0, 1e-12);
    CHECK_NEAR(output.diagnostics.navigation_acceleration.y, config.gravity, 1e-12);
    CHECK_NEAR(output.diagnostics.command_phase.value, 0.0, 1e-12);

    input.attitude = test::vector_test_attitude(0.4, -0.7, 0.2);
    output = law.calculate(input);
    CHECK_NEAR(output.command.overload_g.y, std::cos(0.4) * std::cos(-0.7), 1e-12);
    CHECK_NEAR(output.command.overload_g.z, -std::cos(0.4) * std::sin(-0.7), 1e-12);

    input.attitude = test::vector_test_attitude();
    input.vector_los.inertial_direction_rate = {0.0, 0.1, -0.2};
    output = law.calculate(input);
    CHECK(output.valid);
    CHECK_NEAR(output.diagnostics.body_acceleration.y, 3.9 + config.gravity, 1e-12);
    CHECK_NEAR(output.diagnostics.body_acceleration.z, -7.8, 1e-12);
    input.speed_mps.value = 26.0;
    CHECK_NEAR(law.calculate(input).diagnostics.body_acceleration.z, -15.6, 1e-12);
    input.speed_mps.value = 13.0;
    auto doubled_config = config;
    doubled_config.navigation_constant = 6.0;
    CHECK_NEAR(guidance::BodyVectorPnGuidance(doubled_config)
                   .calculate(input)
                   .diagnostics.body_acceleration.z,
               -15.6, 1e-12);

    // 相同物理 LOS 运动：输入和姿态一起改变，比较导航系物理输出。
    const std::array<double, 3> l_n{0.6, 0.0, 0.8};
    const std::array<double, 3> u_n{0.08, 0.1, -0.06};
    std::array<double, 3> reference{};
    for (double roll : {0.0, 0.6, pi / 2.0, -2.0}) {
        input.attitude = test::vector_test_attitude(0.0, roll);
        const auto l = test::to_test_dart(input.attitude, l_n);
        const auto u = test::to_test_dart(input.attitude, u_n);
        input.vector_los = {true, {l[0], l[1], l[2]}, {u[0], u[1], u[2]}};
        output = law.calculate(input);
        CHECK(output.valid);
        const auto n = output.command.overload_g;
        const auto physical = test::to_test_navigation(input.attitude, {n.x, n.y, n.z});
        if (roll == 0.0)
            reference = physical;
        for (int axis = 0; axis < 3; ++axis) {
            CHECK_NEAR(physical[axis], reference[axis], 1e-11);
        }
    }
    input.attitude = test::vector_test_attitude();
    input.vector_los = {true, {0.0, 1.0, 0.0}, {0.1, 0.0, 0.2}};
    CHECK(law.calculate(input).valid);

    input.vector_los = {true, {1.0, 0.0, 0.0}, {0.0, 1.0, 1.0}};
    auto limited_config = config;
    limited_config.maximum_command_overload = 2.0;
    limited_config.maximum_command_acceleration = config.gravity;
    output = guidance::BodyVectorPnGuidance(limited_config).calculate(input);
    CHECK(output.valid);
    const auto n = output.command.overload_g;
    CHECK_NEAR(std::hypot(n.y, n.z), 1.0, 1e-12);
    CHECK_NEAR(n.z / n.y, 39.0 / (39.0 + config.gravity), 1e-12);
    CHECK_NEAR(output.diagnostics.command_overload.value, 1.0, 1e-12);
    limited_config.maximum_command_overload = 0.0;
    output = guidance::BodyVectorPnGuidance(limited_config).calculate(input);
    CHECK(output.valid);
    CHECK_NEAR(output.command.overload_g.y, 0.0, 1e-12);
    CHECK_NEAR(output.command.overload_g.z, 0.0, 1e-12);
    CHECK_NEAR(output.diagnostics.command_phase.value, 0.0, 1e-12);

    input.vector_los.valid = false;
    CHECK(law.calculate(input).invalid_reason == domain::GuidanceInvalidReason::invalid_rate);
    input.vector_los.valid = true;
    input.attitude.valid = false;
    CHECK(law.calculate(input).invalid_reason == domain::GuidanceInvalidReason::invalid_attitude);
    input.attitude = test::vector_test_attitude();
    input.attitude.navigation_from_dart =
        domain::Rotation3<domain::NavigationFrameTag, domain::DartFrameTag>(
            {2, 0, 0, 0, 1, 0, 0, 0, 1});
    CHECK(!law.calculate(input).valid);
    input.attitude = test::vector_test_attitude();
    input.speed_mps.value = 0.0;
    CHECK(!law.calculate(input).valid);
    input.speed_mps.value = 13.0;
    input.vector_los.direction.x = 2.0;
    CHECK(!law.calculate(input).valid);
    input.vector_los.direction = {1.0, 0.0, 0.0};
    input.vector_los.inertial_direction_rate.x = 0.1;
    CHECK(!law.calculate(input).valid);
    input.vector_los.inertial_direction_rate.x = 0.0;
    input.vector_los.inertial_direction_rate.y = std::numeric_limits<double>::quiet_NaN();
    CHECK(!law.calculate(input).valid);
    limited_config.navigation_constant = -1.0;
    CHECK(!guidance::BodyVectorPnGuidance(limited_config).calculate(input).valid);
    return 0;
}
