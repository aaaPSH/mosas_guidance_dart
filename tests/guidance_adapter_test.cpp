#include <mosas/guidance/default_algorithm_adapters.hpp>
#include <mosas/guidance/guidance_estimator.hpp>

#include "test_support.hpp"

#include <cmath>

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
        {true, 0.0, 0.0}, {true, 0.0, 0.0}, 100.0, 0.0, 0.0);
    CHECK(command.valid);
    CHECK_NEAR(command.navigation_acceleration.x, 0.0, 1e-12);
    CHECK_NEAR(command.navigation_acceleration.y, 9.80665, 1e-12);
    CHECK_NEAR(command.navigation_acceleration.z, 0.0, 1e-12);
    CHECK_NEAR(command.body_overload.y, 1.0, 1e-12);
    CHECK_NEAR(command.command_overload, 1.0, 1e-12);
    CHECK_NEAR(command.command_phase, 0.0, 1e-12);
    CHECK(!PngGuidance::calculate(
               {false, 0.0, 0.0}, {true, 0.0, 0.0}, 100.0, 0.0, 0.0)
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
    CHECK(!rate_adapter.update({true, {0.0}, {0.0}}, {}, 0.01).valid);
    const auto adapted_rate = rate_adapter.update(
        {true, {0.01}, {0.02}}, {}, 0.01);
    CHECK(adapted_rate.valid);
    CHECK_NEAR(adapted_rate.q_y.value, 1.0, 1e-12);
    CHECK_NEAR(adapted_rate.q_z.value, 2.0, 1e-12);
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
    return 0;
}
