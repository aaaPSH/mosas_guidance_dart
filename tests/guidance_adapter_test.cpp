#include <mosas/guidance/default_algorithm_adapters.hpp>
#include <mosas/guidance/guidance_estimator.hpp>

#include "test_support.hpp"
#include "body_vector_test_support.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

int main() {
    VisionResult observation{};
    observation.found = true;
    observation.blob.center_x = 420.0;
    observation.blob.center_y = 190.0;
    const CameraIntrinsics intrinsics{320.0, 320.0, 320.0, 240.0};
    const RotationMatrix3 camera_to_body{
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0,
    };

    const LineOfSight los = GuidanceEstimator::calculate_body(
        observation, intrinsics, camera_to_body);
    CHECK(los.valid);
    CHECK_NEAR(los.q_y, 0.14804631692728484, 1e-12);
    CHECK_NEAR(los.q_z, 0.3028848683749714, 1e-12);

    observation.found = false;
    CHECK(!GuidanceEstimator::calculate_body(
               observation, intrinsics, camera_to_body)
               .valid);
    observation.found = true;
    CHECK(!GuidanceEstimator::calculate_body(
               observation, CameraIntrinsics{0.0, 320.0, 320.0, 240.0},
               camera_to_body)
               .valid);

    LineOfSightRateFilterConfig filter_config;
    filter_config.enable_filter = false;
    LineOfSightRateEstimator rate_estimator(filter_config);
    const auto first_rate = rate_estimator.update(
        {true, 0.0, 0.0}, {0.0, 0.0, 0.0}, 0.01);
    CHECK(!first_rate.valid);
    const auto second_rate = rate_estimator.update(
        {true, 0.01, 0.02}, {0.0, 0.0, 0.0}, 0.01);
    CHECK(second_rate.valid);
    CHECK_NEAR(second_rate.q_y, 1.0, 1e-12);
    CHECK_NEAR(second_rate.q_z, 2.0, 1e-12);
    const auto rate_sample = rate_estimator.sample();
    CHECK(rate_sample.line_of_sight.valid);
    CHECK(rate_sample.raw_angular_velocity.valid);
    CHECK(rate_sample.filtered_angular_velocity.valid);
    const auto lost_target = rate_estimator.update(
        {false, 0.0, 0.0}, {0.0, 0.0, 0.0}, 0.01);
    CHECK(!lost_target.valid);

    const PngGuidanceOutput command = PngGuidance::calculate(
        {true, 0.0, 0.0}, {true, 0.0, 0.0}, 100.0,
        mosas::test::runtime_test_rotation());
    CHECK(command.valid);
    CHECK_NEAR(command.navigation_acceleration.x, 0.0, 1e-12);
    CHECK_NEAR(command.navigation_acceleration.y, 9.80665, 1e-12);
    CHECK_NEAR(command.navigation_acceleration.z, 0.0, 1e-12);
    CHECK_NEAR(command.body_overload.y, 1.0, 1e-12);
    CHECK_NEAR(command.command_overload, 1.0, 1e-12);
    CHECK_NEAR(command.command_phase, 0.0, 1e-12);
    CHECK(!PngGuidance::calculate(
               {false, 0.0, 0.0}, {true, 0.0, 0.0}, 100.0,
               mosas::test::runtime_test_rotation())
               .valid);

    mosas::domain::VisionDetection domain_observation{};
    domain_observation.observation.valid = true;
    domain_observation.observation.center = {420.0, 190.0};
    mosas::guidance::ExistingLosEstimatorAdapter los_adapter;
    const auto domain_los = los_adapter.estimate(
        domain_observation, {320.0, 320.0, 320.0, 240.0},
        mosas::domain::Rotation3<mosas::domain::DartFrameTag,
                                 mosas::domain::CameraFrameTag>{});
    CHECK(domain_los.valid);
    CHECK_NEAR(domain_los.q_y.value, los.q_y, 1e-12);
    CHECK_NEAR(domain_los.q_z.value, los.q_z, 1e-12);

    LineOfSightRateFilterConfig adapter_filter_config;
    adapter_filter_config.enable_filter = false;
    mosas::guidance::ExistingLosRateFilterAdapter rate_adapter(
        adapter_filter_config);
    CHECK(!rate_adapter.update({{true, {0.0}, {0.0}}, {}, 0.01, 0}).valid);
    const auto adapted_rate = rate_adapter.update(
        {{true, {0.01}, {0.02}}, {}, 0.01, 0});
    CHECK(adapted_rate.valid);
    CHECK_NEAR(adapted_rate.angular_velocity.q_y.value, 1.0, 1e-12);
    CHECK_NEAR(adapted_rate.angular_velocity.q_z.value, 2.0, 1e-12);
    CHECK(!adapted_rate.vector.valid);
    CHECK(rate_adapter.sample().filtered_angular_velocity.valid);
    rate_adapter.reset();
    CHECK(!rate_adapter.sample().filtered_angular_velocity.valid);

    mosas::domain::GuidanceInput domain_guidance{};
    domain_guidance.source_frame_id = 99;
    domain_guidance.time.monotonic_ns = 123456;
    domain_guidance.line_of_sight = {true, {0.0}, {0.0}};
    domain_guidance.angular_velocity = {true, {0.0}, {0.0}};
    domain_guidance.speed_mps = {100.0};
    domain_guidance.attitude.valid = true;
    mosas::guidance::ExistingPngGuidanceLawAdapter guidance_adapter;
    const auto domain_output = guidance_adapter.calculate(domain_guidance);
    CHECK(domain_output.valid);
    CHECK(domain_output.command.valid);
    CHECK(domain_output.command.source_frame_id == 99);
    CHECK(domain_output.command.source_time.monotonic_ns == 123456);
    CHECK_NEAR(domain_output.command.overload_g.y, 1.0, 1e-12);
    CHECK_NEAR(domain_output.diagnostics.body_acceleration.y, 9.80665, 1e-12);
    CHECK(!guidance_adapter.calculate({}).valid);

    PngGuidanceConfig png;
    png.navigation_constant_y = 4.0;
    png.navigation_constant_z = 2.0;
    png.maximum_command_overload = 100.0;
    const double pitch = 0.35;
    const double roll = 0.7;
    const auto attitude = mosas::test::vector_test_attitude(pitch, roll, -0.4);
    const auto rotation = mosas::test::runtime_test_rotation(pitch, roll, -0.4);
    const LineOfSight measured_los{true, 0.2, -0.3};
    const auto inertial_direction = mosas::test::to_test_navigation(
        attitude, {std::cos(measured_los.q_y) * std::cos(measured_los.q_z),
                   std::sin(measured_los.q_y),
                   std::cos(measured_los.q_y) * std::sin(measured_los.q_z)});
    const double inertial_elevation = std::atan2(
        inertial_direction[1], std::hypot(inertial_direction[0], inertial_direction[2]));
    CHECK(std::abs(inertial_elevation - pitch) > 0.02);
    CHECK(std::abs(inertial_elevation - measured_los.q_y) > 0.02);
    mosas::guidance::ExistingPngGuidanceLawAdapter compensated_adapter(png);
    for (const double speed : {8.0, 16.0, 25.0}) {
        const double parabola_rate = -png.gravity * std::cos(inertial_elevation) / (2.0 * speed);
        PngCalculationTrace trace;
        const auto natural = PngGuidance::calculate(
            measured_los, {true, parabola_rate, 0.0}, speed, rotation, png, &trace);
        CHECK(natural.valid && trace.valid);
        CHECK_NEAR(trace.inertial_los_elevation_rad, inertial_elevation, 1e-12);
        CHECK_NEAR(trace.parabolic_elevation_rate_rad_s, parabola_rate, 1e-12);
        CHECK_NEAR(trace.compensated_elevation_rate_rad_s, 0.0, 1e-12);
        // 自然抛物线速率完全抵消，输出端不能再叠加重力过载。
        CHECK_NEAR(natural.body_overload.y, 0.0, 1e-12);
        CHECK_NEAR(natural.body_overload.z, 0.0, 1e-12);

        const double measured_y_rate = parabola_rate + 0.12;
        const auto perturbed = PngGuidance::calculate(
            measured_los, {true, measured_y_rate, -0.07}, speed, rotation, png, &trace);
        const double vertical = png.navigation_constant_y * speed * 0.12 / png.gravity;
        const double lateral = png.navigation_constant_z * speed * -0.07 /
                               png.gravity * std::cos(pitch);
        CHECK(perturbed.valid);
        CHECK_NEAR(trace.measured_elevation_rate_rad_s, measured_y_rate, 1e-12);
        CHECK_NEAR(trace.compensated_elevation_rate_rad_s, 0.12, 1e-12);
        CHECK_NEAR(perturbed.body_overload.y,
                   std::cos(roll) * vertical + std::sin(roll) * lateral, 1e-12);
        CHECK_NEAR(perturbed.body_overload.z,
                   -std::sin(roll) * vertical + std::cos(roll) * lateral, 1e-12);
        domain_guidance.attitude = attitude;
        domain_guidance.line_of_sight = {true, {measured_los.q_y}, {measured_los.q_z}};
        domain_guidance.angular_velocity = {true, {measured_y_rate}, {-0.07}};
        domain_guidance.speed_mps.value = speed;
        const auto adapted = compensated_adapter.calculate(domain_guidance);
        CHECK(adapted.valid);
        CHECK_NEAR(adapted.command.overload_g.y, perturbed.body_overload.y, 1e-12);
        CHECK_NEAR(adapted.command.overload_g.z, perturbed.body_overload.z, 1e-12);
    }
    // 通道选择发生在 roll 转换之后、两级限幅之前；未选轴不能挤占额度。
    for (const double test_roll : {0.0, 0.7, std::acos(-1.0) / 2.0}) {
        const auto test_rotation = mosas::test::runtime_test_rotation(0.35, test_roll);
        for (const LineOfSightAngularVelocity rates :
             {LineOfSightAngularVelocity{true, -0.25, 2.0},
              LineOfSightAngularVelocity{true, 2.0, -0.01},
              LineOfSightAngularVelocity{true, -2.0, -2.0}}) {
            const auto unrestricted = PngGuidance::calculate(
                measured_los, rates, 16.0, test_rotation, png);
            CHECK(unrestricted.valid);
            for (const auto channel : {mosas::guidance::ControlChannel::pitch,
                                       mosas::guidance::ControlChannel::yaw}) {
                for (const double acceleration_limit :
                     {std::numeric_limits<double>::infinity(), 0.1 * png.gravity}) {
                    auto selected = png;
                    selected.control_channel = channel;
                    selected.maximum_command_overload = 0.3;
                    selected.maximum_command_acceleration = acceleration_limit;
                    PngCalculationTrace trace;
                    const auto single = PngGuidance::calculate(
                        measured_los, rates, 16.0, test_rotation, selected, &trace);
                    CHECK(single.valid && trace.valid);
                    const bool pitch_channel = channel == mosas::guidance::ControlChannel::pitch;
                    const double raw = pitch_channel ? unrestricted.body_overload.y
                                                     : unrestricted.body_overload.z;
                    const double limit = std::min(0.3, acceleration_limit / png.gravity);
                    const double expected = std::copysign(std::min(std::abs(raw), limit), raw);
                    CHECK_NEAR(pitch_channel ? single.body_overload.y : single.body_overload.z,
                               expected, 1e-12);
                    CHECK((pitch_channel ? single.body_overload.z : single.body_overload.y) == 0.0);
                    CHECK_NEAR(single.command_overload, std::abs(expected), 1e-12);
                    CHECK_NEAR(single.body_acceleration.y, single.body_overload.y * png.gravity, 1e-12);
                    CHECK_NEAR(single.body_acceleration.z, single.body_overload.z * png.gravity, 1e-12);
                    CHECK_NEAR(single.navigation_acceleration.y,
                               std::cos(test_roll) * single.body_acceleration.y -
                                   std::sin(test_roll) * single.body_acceleration.z, 1e-12);
                    CHECK_NEAR(single.navigation_acceleration.z,
                               std::sin(test_roll) * single.body_acceleration.y +
                                   std::cos(test_roll) * single.body_acceleration.z, 1e-12);
                    CHECK_NEAR(trace.body_overload.y, single.body_overload.y, 0.0);
                    CHECK_NEAR(trace.body_overload.z, single.body_overload.z, 0.0);
                    CHECK_NEAR(trace.navigation_acceleration.y, single.navigation_acceleration.y, 0.0);
                    CHECK_NEAR(trace.navigation_acceleration.z, single.navigation_acceleration.z, 0.0);
                    CHECK_NEAR(std::hypot(trace.after_overload_limit.y, trace.after_overload_limit.z),
                               std::min(std::abs(raw), 0.3), 1e-12);
                    selected.maximum_command_overload = 0.0;
                    const auto zero = PngGuidance::calculate(
                        measured_los, rates, 16.0, test_rotation, selected);
                    CHECK(zero.valid && zero.body_overload.y == 0.0 && zero.body_overload.z == 0.0);
                }
            }
        }
    }
    auto invalid_channel_config = png;
    invalid_channel_config.control_channel = static_cast<mosas::guidance::ControlChannel>(-1);
    CHECK(!PngGuidance::calculate(measured_los, {true, 0.0, 0.0}, 16.0,
                                  rotation, invalid_channel_config).valid);

    auto invalid_rotation = rotation;
    invalid_rotation.m00 = 2.0;
    CHECK(!PngGuidance::calculate(measured_los, {true, 0.0, 0.0}, 16.0,
                                  invalid_rotation, png).valid);
    for (const double speed : {0.0, -1.0, std::numeric_limits<double>::infinity(),
                               std::numeric_limits<double>::quiet_NaN()}) {
        CHECK(!PngGuidance::calculate(measured_los, {true, 0.0, 0.0}, speed,
                                      rotation, png).valid);
    }
    return 0;
}
