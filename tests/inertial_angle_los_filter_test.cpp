#include <mosas/guidance/inertial_angle_los_filter.hpp>
#include <mosas/guidance/ballistic_png_math.hpp>

#include "body_vector_test_support.hpp"
#include "test_support.hpp"

#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace {
using namespace mosas;
constexpr double kPi = 3.14159265358979323846;
using Matrix2 = std::array<std::array<double, 2>, 2>;

/** @brief 独立通用矩阵参考，使用原二状态协方差校正公式而非生产 Joseph 形式。 */
struct ReferenceKalman {
    double angle, rate = 0.0;
    Matrix2 p;

    /** @param measurement 当前角观测。 @param dt 帧间隔。 @param sigma 过程噪声标准差。 @param r 测量方差。 */
    void update(double measurement, double dt, double sigma, double r) {
        const Matrix2 f{{{{1.0, dt}}, {{0.0, 1.0}}}};
        const std::array<double, 2> g{dt * dt / 2.0, dt};
        Matrix2 prior{};
        for (int i = 0; i < 2; ++i)
            for (int j = 0; j < 2; ++j) {
                for (int a = 0; a < 2; ++a)
                    for (int b = 0; b < 2; ++b)
                        prior[i][j] += f[i][a] * p[a][b] * f[j][b];
                prior[i][j] += sigma * sigma * g[i] * g[j];
            }
        const double k0 = prior[0][0] / (prior[0][0] + r);
        const double k1 = prior[1][0] / (prior[0][0] + r);
        angle += rate * dt;
        const double innovation = measurement - angle;
        angle += k0 * innovation;
        rate += k1 * innovation;
        p = {{{{(1 - k0) * prior[0][0], (1 - k0) * prior[0][1]}},
              {{(1 - k0) * prior[1][0], prior[1][1] - k1 * prior[0][1]}}}};
    }
};

/** @return 双通道独立噪声、公共噪声回退及角状态到向量的解析映射正确时为 0。 */
int verify_kalman_reference() {
    for (bool fallback : {false, true}) {
        guidance::InertialAngleLosFilterConfig config;
        if (fallback) {
            config.sigma_a = 2.8;
            config.sigma_a_pitch = config.sigma_a_yaw = 0.0;
            config.initial_covariance = 0.0;
        }
        guidance::InertialAngleLosFilter filter(config);
        domain::LosRateFilterInput input;
        input.attitude = test::vector_test_attitude();
        input.line_of_sight = test::test_dart_los_from_angles(0.2, -0.3);
        input.dt_seconds = 1.0; // 首帧不传播这个超过 maximum_gap 的占位时间。
        CHECK(!filter.update(input).valid);
        CHECK(!filter.sample().raw_angular_velocity.valid);
        CHECK(!filter.sample().filtered_angular_velocity.valid);
        const double r = config.sigma_q * config.sigma_q;
        const Matrix2 initial{{{{r, 0.0}}, {{0.0, config.initial_covariance}}}};
        ReferenceKalman pitch{0.2, 0.0, initial}, yaw{-0.3, 0.0, initial};
        constexpr std::array<double, 4> intervals{0.001, 0.008, 0.02, 0.05};
        double previous_pitch = 0.2, previous_yaw = -0.3;
        for (int step = 1; step <= 80; ++step) {
            const double dt = intervals[step % intervals.size()];
            input.dt_seconds = dt;
            const double measured_pitch = 0.2 + 0.15 * std::sin(step * 0.1);
            const double measured_yaw = -0.3 + 0.12 * std::sin(step * 0.13);
            input.line_of_sight = test::test_dart_los_from_angles(measured_pitch, measured_yaw);
            pitch.update(measured_pitch, dt, fallback ? config.sigma_a : config.sigma_a_pitch, r);
            yaw.update(measured_yaw, dt, fallback ? config.sigma_a : config.sigma_a_yaw, r);
            const auto estimate = filter.update(input);
            CHECK(estimate.valid && estimate.vector.valid && estimate.angular_velocity.valid);
            const double cy = std::cos(pitch.angle), sy = std::sin(pitch.angle);
            const double cz = std::cos(yaw.angle), sz = std::sin(yaw.angle);
            CHECK_NEAR(estimate.vector.direction.x, cy * cz, 1e-12);
            CHECK_NEAR(estimate.vector.direction.y, sy, 1e-12);
            CHECK_NEAR(estimate.vector.direction.z, cy * sz, 1e-12);
            CHECK_NEAR(estimate.angular_velocity.q_y.value, pitch.rate, 1e-11);
            CHECK_NEAR(estimate.angular_velocity.q_z.value, cy * yaw.rate, 1e-11);
            const auto& l = estimate.vector.direction;
            const auto& u = estimate.vector.inertial_direction_rate;
            CHECK_NEAR(u.x, -sy * cz * pitch.rate - cy * sz * yaw.rate, 1e-11);
            CHECK_NEAR(u.y, cy * pitch.rate, 1e-11);
            CHECK_NEAR(u.z, -sy * sz * pitch.rate + cy * cz * yaw.rate, 1e-11);
            CHECK_NEAR(std::hypot(std::hypot(l.x, l.y), l.z), 1.0, 1e-12);
            CHECK_NEAR(l.x * u.x + l.y * u.y + l.z * u.z, 0.0, 1e-12);
            const auto raw = filter.sample().raw_angular_velocity;
            CHECK(raw.valid);
            CHECK_NEAR(raw.q_y.value, (measured_pitch - previous_pitch) / dt, 1e-10);
            CHECK_NEAR(raw.q_z.value, std::cos(measured_pitch) * (measured_yaw - previous_yaw) / dt, 1e-10);
            previous_pitch = measured_pitch;
            previous_yaw = measured_yaw;
            const auto png = guidance::evaluate_ballistic_png(l, u, 16.0, 3.0, 3.0, 9.80665);
            CHECK(png.valid);
            CHECK_NEAR(png.measured_elevation_rate_rad_s, pitch.rate, 1e-11);
            CHECK_NEAR(png.measured_lateral_rate_rad_s, cy * yaw.rate, 1e-11);
        }
    }
    return 0;
}

/** @return 滤波之前使用曝光姿态旋转而非机体角度，纯机体转动不产生视线速率时为 0。 */
int verify_inertial_rotation() {
    guidance::InertialAngleLosFilter filter;
    const std::array<double, 3> fixed{0.8, 0.36, 0.48};
    domain::LosRateFilterInput input;
    input.dt_seconds = 0.01;
    for (int step = 0; step < 100; ++step) {
        input.attitude = test::vector_test_attitude(0.2 * std::sin(step * 0.05), step * 0.1, -step * 0.03);
        const auto body = test::to_test_dart(input.attitude, fixed);
        input.line_of_sight = {true, {2.0 * body[0], 2.0 * body[1], 2.0 * body[2]}};
        const auto estimate = filter.update(input);
        if (step == 0) { CHECK(!estimate.valid); continue; }
        CHECK(estimate.valid);
        CHECK_NEAR(estimate.vector.direction.x, fixed[0], 1e-12);
        CHECK_NEAR(estimate.vector.direction.y, fixed[1], 1e-12);
        CHECK_NEAR(estimate.vector.direction.z, fixed[2], 1e-12);
        CHECK_NEAR(estimate.angular_velocity.q_y.value, 0.0, 1e-11);
        CHECK_NEAR(estimate.angular_velocity.q_z.value, 0.0, 1e-11);
        CHECK_NEAR(filter.sample().line_of_sight.direction.x, body[0], 1e-12);
    }
    return 0;
}

/** @return 绝对杆臂补偿在 atan2 和角度差分之前完成，消除姿态摆动带来的视差误差，且对小距离限幅。 */
int verify_compensated_angles() {
    guidance::InertialAngleLosFilterConfig config;
    config.enable_filter = false;
    config.camera_lever_arm_x = 0.18;
    config.estimated_target_distance = 15.0;
    guidance::InertialAngleLosFilter filter(config);
    const std::array<double, 3> target{12.0, 6.0, 7.0};
    domain::LosRateFilterInput input;
    input.dt_seconds = 0.01;
    double previous_pitch = 0.0, previous_yaw = 0.0;
    for (int step = 0; step < 30; ++step) {
        const bool baseline = step == 0 || step == 15;
        input.dt_seconds = step == 15 ? 0.1 : 0.01;
        input.attitude = test::vector_test_attitude(0.3 * std::sin(step * 0.2), step * 0.05, step * 0.07);
        const auto& m = input.attitude.navigation_from_dart.matrix();
        const std::array<double, 3> position{m[0] * 0.18, m[3] * 0.18, m[6] * 0.18};
        std::array<double, 3> camera_to_target{};
        for (int i = 0; i < 3; ++i) camera_to_target[i] = target[i] - position[i];
        const double distance = std::hypot(std::hypot(camera_to_target[0], camera_to_target[1]), camera_to_target[2]);
        std::array<double, 3> corrected{};
        for (int i = 0; i < 3; ++i)
            corrected[i] = camera_to_target[i] / distance + position[i] / 15.0;
        const double pitch = std::atan2(corrected[1], std::hypot(corrected[0], corrected[2]));
        const double yaw = std::atan2(corrected[2], corrected[0]);
        input.line_of_sight = test::test_dart_los(test::to_test_dart(input.attitude, camera_to_target));
        const auto estimate = filter.update(input);
        if (baseline) { CHECK(!estimate.valid); }
        else {
            CHECK(estimate.valid);
            const auto& l = estimate.vector.direction;
            CHECK_NEAR(std::atan2(l.y, std::hypot(l.x, l.z)), pitch, 1e-12);
            CHECK_NEAR(std::atan2(l.z, l.x), yaw, 1e-12);
            CHECK_NEAR(estimate.angular_velocity.q_y.value, (pitch - previous_pitch) / input.dt_seconds, 1e-11);
            CHECK_NEAR(estimate.angular_velocity.q_z.value,
                       std::cos(pitch) * (yaw - previous_yaw) / input.dt_seconds, 1e-11);
            CHECK_NEAR(filter.sample().raw_angular_velocity.q_y.value,
                       filter.sample().filtered_angular_velocity.q_y.value, 1e-12);
        }
        previous_pitch = pitch;
        previous_yaw = yaw;
    }

    // 验证最小距离限幅保护：当 estimated_target_distance 小于 1.0m 时，分母限幅为 1.0m
    {
        guidance::InertialAngleLosFilterConfig small_dist_cfg;
        small_dist_cfg.enable_filter = false;
        small_dist_cfg.camera_lever_arm_x = 0.18;
        small_dist_cfg.estimated_target_distance = 0.2; // 0.2m < 1.0m，应限幅为 1.0m
        guidance::InertialAngleLosFilter small_dist_filter(small_dist_cfg);
        domain::LosRateFilterInput test_input;
        test_input.dt_seconds = 0.01;
        test_input.attitude = test::vector_test_attitude(0.1, 0.0, 0.0);
        const auto& m = test_input.attitude.navigation_from_dart.matrix();
        const std::array<double, 3> position{m[0] * 0.18, m[3] * 0.18, m[6] * 0.18};
        const std::array<double, 3> cam_to_tgt{10.0 - position[0], -position[1], -position[2]};
        const double d = std::hypot(std::hypot(cam_to_tgt[0], cam_to_tgt[1]), cam_to_tgt[2]);
        test_input.line_of_sight = test::test_dart_los(test::to_test_dart(test_input.attitude, cam_to_tgt));
        CHECK(!small_dist_filter.update(test_input).valid); // 首帧仅建基准
        const auto est = small_dist_filter.update(test_input);
        CHECK(est.valid);
        const std::array<double, 3> expected_corrected{
            cam_to_tgt[0] / d + position[0] / 1.0,
            cam_to_tgt[1] / d + position[1] / 1.0,
            cam_to_tgt[2] / d + position[2] / 1.0};
        const double expected_pitch = std::atan2(expected_corrected[1],
            std::hypot(expected_corrected[0], expected_corrected[2]));
        CHECK_NEAR(std::atan2(est.vector.direction.y,
            std::hypot(est.vector.direction.x, est.vector.direction.z)), expected_pitch, 1e-12);
    }

    return 0;
}

/** @return 10 Hz 镖体摆动下，绝对杆臂补偿有效消除视线角速度虚假振荡。 */
int verify_oscillation_suppression() {
    constexpr double r_true = 15.0;
    constexpr double lever_arm = 0.18;
    const std::array<double, 3> target_pos_N{r_true, 0.0, 0.0};

    guidance::InertialAngleLosFilterConfig cfg_uncomp;
    cfg_uncomp.camera_lever_arm_x = 0.0;
    guidance::InertialAngleLosFilter filter_uncomp(cfg_uncomp);

    guidance::InertialAngleLosFilterConfig cfg_comp;
    cfg_comp.camera_lever_arm_x = lever_arm;
    cfg_comp.estimated_target_distance = r_true;
    guidance::InertialAngleLosFilter filter_comp(cfg_comp);

    domain::LosRateFilterInput input;
    input.dt_seconds = 0.01;

    double max_rate_uncomp = 0.0;
    double max_rate_comp = 0.0;

    for (int i = 0; i < 200; ++i) {
        const double t = i * input.dt_seconds;
        const double pitch = 0.05 * std::sin(2.0 * M_PI * 10.0 * t);
        input.attitude = test::vector_test_attitude(pitch, 0.0, 0.0);

        const auto& m = input.attitude.navigation_from_dart.matrix();
        const std::array<double, 3> p_cam_N{m[0] * lever_arm, m[3] * lever_arm, m[6] * lever_arm};

        const std::array<double, 3> r_cam_to_tgt_N{
            target_pos_N[0] - p_cam_N[0],
            target_pos_N[1] - p_cam_N[1],
            target_pos_N[2] - p_cam_N[2]};

        input.line_of_sight = test::test_dart_los(test::to_test_dart(input.attitude, r_cam_to_tgt_N));

        auto est_uncomp = filter_uncomp.update(input);
        auto est_comp = filter_comp.update(input);

        if (i > 50) {
            CHECK(est_uncomp.valid && est_comp.valid);
            max_rate_uncomp = std::max(max_rate_uncomp, std::abs(est_uncomp.angular_velocity.q_y.value));
            max_rate_comp = std::max(max_rate_comp, std::abs(est_comp.angular_velocity.q_y.value));
        }
    }

    CHECK(max_rate_uncomp > 0.01);
    CHECK(max_rate_comp < 0.002);
    return 0;
}

/** @return 首帧不积分，后续变速、变帧间隔使用梯形积分，并在 1 m 饱和时为 0。 */
int verify_distance_integration() {
    for (bool enabled : {false, true}) {
        guidance::InertialAngleLosFilterConfig config;
        config.enable_filter = enabled;
        config.camera_lever_arm_x = 0.18;
        config.estimated_target_distance = 2.0;
        guidance::InertialAngleLosFilter filter(config);
        domain::LosRateFilterInput input;
        input.attitude = test::vector_test_attitude(0.3, -0.1, 0.15);
        input.line_of_sight = test::test_dart_los_from_angles(0.15, -0.2);
        const auto navigation = input.attitude.navigation_from_dart.apply(input.line_of_sight.direction);
        const auto& m = input.attitude.navigation_from_dart.matrix();
        constexpr std::array<double, 7> intervals{1.0, 0.01, 0.02, 0.03, 0.04, 0.05, 0.01};
        constexpr std::array<double, 7> speeds{2.0, 6.0, 10.0, 14.0, 18.0, 22.0, 0.0};
        constexpr std::array<double, 7> distances{2.0, 1.96, 1.8, 1.44, 1.0, 1.0, 1.0};
        CHECK(!filter.sample().lever_arm_distance_valid);
        CHECK_NEAR(filter.sample().lever_arm_distance_m, 0.0, 1e-12);
        double previous_pitch = 0.0, previous_yaw = 0.0;
        for (std::size_t i = 0; i < intervals.size(); ++i) {
            input.dt_seconds = intervals[i];
            input.speed_mps.value = speeds[i];
            const double x = navigation.x + m[0] * config.camera_lever_arm_x / distances[i];
            const double y = navigation.y + m[3] * config.camera_lever_arm_x / distances[i];
            const double z = navigation.z + m[6] * config.camera_lever_arm_x / distances[i];
            const double pitch = std::atan2(y, std::hypot(x, z));
            const double yaw = std::atan2(z, x);
            const auto estimate = filter.update(input);
            const auto snapshot = filter.sample();
            CHECK(snapshot.lever_arm_distance_valid);
            CHECK_NEAR(snapshot.lever_arm_distance_m, distances[i], 1e-12);
            CHECK_NEAR(filter.sample().lever_arm_distance_m, snapshot.lever_arm_distance_m, 1e-12);
            if (i == 0) {
                CHECK(!estimate.valid);
            } else {
                CHECK(estimate.valid);
                const auto raw = filter.sample().raw_angular_velocity;
                CHECK(raw.valid);
                CHECK_NEAR(raw.q_y.value, (pitch - previous_pitch) / intervals[i], 1e-11);
                CHECK_NEAR(raw.q_z.value, std::cos(pitch) * (yaw - previous_yaw) / intervals[i], 1e-11);
                if (!enabled) {
                    const auto& l = estimate.vector.direction;
                    CHECK_NEAR(std::atan2(l.y, std::hypot(l.x, l.z)), pitch, 1e-12);
                    CHECK_NEAR(std::atan2(l.z, l.x), yaw, 1e-12);
                }
            }
            previous_pitch = pitch;
            previous_yaw = yaw;
        }
    }
    return 0;
}

/** @return 连续段重建时恢复初始距离，非法速度重置状态而非增大距离时为 0。 */
int verify_distance_reset() {
    guidance::InertialAngleLosFilterConfig config;
    config.enable_filter = false;
    config.camera_lever_arm_x = 0.18;
    config.estimated_target_distance = 2.0;
    guidance::InertialAngleLosFilter filter(config);
    domain::LosRateFilterInput input;
    input.attitude = test::vector_test_attitude();
    input.line_of_sight = test::test_dart_los_from_angles(0.3, 0.2);
    input.dt_seconds = 0.01;
    const auto& d = input.line_of_sight.direction;
    const double initial_pitch = std::atan2(d.y, std::hypot(d.x + 0.09, d.z));
    const double minimum_pitch = std::atan2(d.y, std::hypot(d.x + 0.18, d.z));
    for (int reason = 0; reason < 6; ++reason) {
        input.speed_mps.value = 100.0;
        CHECK(!filter.update(input).valid);
        const auto approaching = filter.update(input);
        CHECK(approaching.valid);
        CHECK_NEAR(std::atan2(approaching.vector.direction.y,
                             std::hypot(approaching.vector.direction.x, approaching.vector.direction.z)),
                   minimum_pitch, 1e-12);
        input.speed_mps.value = 0.0;
        if (reason == 0) {
            filter.reset();
        } else {
            if (reason == 1) ++input.generation;
            if (reason == 2) input.dt_seconds = 0.1;
            if (reason == 3) input.speed_mps.value = -1.0;
            if (reason == 4) input.speed_mps.value = std::numeric_limits<double>::quiet_NaN();
            if (reason == 5) input.speed_mps.value = std::numeric_limits<double>::infinity();
            CHECK(!filter.update(input).valid);
        }
        CHECK(filter.sample().lever_arm_distance_valid == (reason == 1 || reason == 2));
        input.dt_seconds = 0.01;
        // 代际和超时帧已经建好基准；其他重置原因需再用首帧建基准。
        input.speed_mps.value = 0.0;
        if (reason != 1 && reason != 2) CHECK(!filter.update(input).valid);
        const auto restored = filter.update(input);
        CHECK(restored.valid);
        CHECK_NEAR(std::atan2(restored.vector.direction.y,
                             std::hypot(restored.vector.direction.x, restored.vector.direction.z)),
                   initial_pitch, 1e-12);
        CHECK(filter.sample().lever_arm_distance_valid);
        CHECK_NEAR(filter.sample().lever_arm_distance_m, config.estimated_target_distance, 1e-12);
        filter.reset();
        CHECK(!filter.sample().lever_arm_distance_valid);
        CHECK_NEAR(filter.sample().lever_arm_distance_m, 0.0, 1e-12);
    }
    return 0;
}

/** @return 杆臂开关独立于 Kalman 开关，关闭后仍做曝光姿态转换和角速率估计时为 0。 */
int verify_compensation_switch() {
    for (bool enabled : {false, true}) {
        guidance::InertialAngleLosFilterConfig config;
        config.enable_filter = enabled;
        guidance::InertialAngleLosFilter reference(config);
        config.enable_lever_arm_compensation = false;
        config.camera_lever_arm_x = 0.18;
        config.estimated_target_distance = 0.2;
        guidance::InertialAngleLosFilter disabled(config);
        domain::LosRateFilterInput input;
        input.dt_seconds = 0.01;
        for (int step = 0; step < 20; ++step) {
            input.attitude = test::vector_test_attitude(step * 0.01, step * 0.03, -step * 0.02);
            input.line_of_sight = test::test_dart_los_from_angles(0.2 + step * 0.002, -0.3 + step * 0.003);
            input.speed_mps.value = 0.0;
            const auto expected = reference.update(input);
            // 关闭补偿时，速度不参与计算，也不应影响姿态转换和滤波。
            input.speed_mps.value = step % 2 ? -1.0 : std::numeric_limits<double>::quiet_NaN();
            const auto actual = disabled.update(input);
            CHECK(!disabled.sample().lever_arm_distance_valid);
            CHECK_NEAR(disabled.sample().lever_arm_distance_m, 0.0, 1e-12);
            CHECK(actual.valid == expected.valid);
            if (step == 0) continue;
            CHECK(actual.valid);
            CHECK_NEAR(actual.vector.direction.x, expected.vector.direction.x, 1e-12);
            CHECK_NEAR(actual.vector.direction.y, expected.vector.direction.y, 1e-12);
            CHECK_NEAR(actual.vector.direction.z, expected.vector.direction.z, 1e-12);
            CHECK_NEAR(actual.angular_velocity.q_y.value, expected.angular_velocity.q_y.value, 1e-12);
            CHECK_NEAR(actual.angular_velocity.q_z.value, expected.angular_velocity.q_z.value, 1e-12);
        }
    }
    return 0;
}

/** @return 方位跨越 +/-pi 时差分和 Kalman 创新均走最短圆周差、没有速率尖峰时为 0。 */
int verify_azimuth_wrap() {
    for (bool enabled : {false, true}) {
        guidance::InertialAngleLosFilterConfig config;
        config.enable_filter = enabled;
        guidance::InertialAngleLosFilter filter(config);
        domain::LosRateFilterInput input;
        input.attitude = test::vector_test_attitude();
        input.dt_seconds = 0.01;
        domain::LosRateEstimate estimate;
        for (int step = 0; step <= 100; ++step) {
            input.line_of_sight = test::test_dart_los_from_angles(0.4, kPi - 0.004 + step * 0.008);
            estimate = filter.update(input);
            if (step == 0) { CHECK(!estimate.valid); continue; }
            CHECK(estimate.valid);
            CHECK_NEAR(filter.sample().raw_angular_velocity.q_z.value, std::cos(0.4) * 0.8, 1e-11);
            CHECK(std::abs(estimate.angular_velocity.q_z.value) < 1.5);
        }
        CHECK_NEAR(estimate.angular_velocity.q_z.value, std::cos(0.4) * 0.8, 1e-5);
    }
    return 0;
}

/** @return 非法输入、方位极点、时间异常、代际切换均清除连续段而非外推时为 0。 */
int verify_reset_paths() {
    guidance::InertialAngleLosFilter filter;
    domain::LosRateFilterInput input;
    input.attitude = test::vector_test_attitude();
    input.line_of_sight = {true, {1, 0, 0}};
    input.dt_seconds = 0.0;
    CHECK(!filter.update(input).valid);
    input.dt_seconds = 0.01;
    CHECK(filter.update(input).valid);
    ++input.generation;
    CHECK(!filter.update(input).valid);
    CHECK(filter.update(input).valid);
    CHECK_NEAR(filter.sample().filtered_angular_velocity.q_y.value, 0.0, 1e-12);
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    const double max = std::numeric_limits<double>::max();
    for (const auto& bad : std::array<domain::DirectionDartUnit, 8>{{
             {0, 0, 0}, {1e-13, 0, 0}, {nan, 0, 0}, {1, inf, 0}, {max, max, max},
             {0, 1, 0}, {0, -1, 0}, {1e-9, 1, 0}}}) {
        input.line_of_sight.direction = bad;
        CHECK(!filter.update(input).valid);
        CHECK(!filter.sample().line_of_sight.valid);
        CHECK(!filter.sample().lever_arm_distance_valid);
        input.line_of_sight.direction = {1, 0, 0};
        CHECK(!filter.update(input).valid);
        CHECK(filter.update(input).valid);
    }
    for (double bad : {0.0, -0.01, nan, inf}) {
        input.dt_seconds = bad;
        CHECK(!filter.update(input).valid);
        input.dt_seconds = 0.01;
        CHECK(!filter.update(input).valid);
        CHECK(filter.update(input).valid);
    }
    for (const auto& matrix : std::array<std::array<double, 9>, 2>{{
             {1, 0, 0, 1, 0, 0, 0, 0, 1}, {-1, 0, 0, 0, 1, 0, 0, 0, 1}}}) {
        input.attitude.navigation_from_dart = domain::Rotation3<domain::NavigationFrameTag, domain::DartFrameTag>(matrix);
        CHECK(!filter.update(input).valid);
        input.attitude = test::vector_test_attitude();
        CHECK(!filter.update(input).valid);
        CHECK(filter.update(input).valid);
    }
    input.attitude.valid = false;
    CHECK(!filter.update(input).valid);
    CHECK(!filter.sample().lever_arm_distance_valid);
    input.attitude.valid = true;
    input.line_of_sight.valid = false;
    CHECK(!filter.update(input).valid);
    CHECK(!filter.sample().lever_arm_distance_valid);
    input.line_of_sight.valid = true;
    CHECK(!filter.update(input).valid);
    CHECK(filter.update(input).valid);
    filter.reset();
    CHECK(!filter.sample().line_of_sight.valid);
    CHECK(!filter.update(input).valid);
    return 0;
}

/** @return 旧参数语义恢复、不可表示的方差和非法几何配置被拒绝时为 0。 */
int verify_config() {
    using Config = guidance::InertialAngleLosFilterConfig;
    CHECK_NEAR(Config{}.sigma_a, 8.0, 1e-12);
    CHECK_NEAR(Config{}.sigma_a_pitch, 10.0, 1e-12);
    CHECK_NEAR(Config{}.sigma_a_yaw, 3.5, 1e-12);
    CHECK_NEAR(Config{}.sigma_q, 0.0035, 1e-12);
    CHECK(Config{}.enable_lever_arm_compensation);
    for (auto member : {&Config::sigma_a, &Config::sigma_a_pitch, &Config::sigma_a_yaw,
                       &Config::sigma_q, &Config::initial_covariance, &Config::maximum_gap_seconds,
                       &Config::camera_lever_arm_x, &Config::estimated_target_distance}) {
        Config config;
        config.*member = std::numeric_limits<double>::quiet_NaN();
        CHECK(!guidance::InertialAngleLosFilter::valid_config(config));
        bool threw = false;
        try { guidance::InertialAngleLosFilter invalid(config); }
        catch (const std::invalid_argument&) { threw = true; }
        CHECK(threw);
    }
    for (auto member : {&Config::sigma_a, &Config::sigma_q}) {
        for (double invalid : {0.0, -1.0, std::numeric_limits<double>::min(), std::numeric_limits<double>::max()}) {
            Config config;
            config.*member = invalid;
            CHECK(!guidance::InertialAngleLosFilter::valid_config(config));
        }
    }
    Config config;
    config.sigma_a_pitch = config.sigma_a_yaw = config.initial_covariance = 0.0;
    config.camera_lever_arm_x = -0.18;
    CHECK(guidance::InertialAngleLosFilter::valid_config(config));
    config.sigma_a_pitch = -1.0;
    CHECK(!guidance::InertialAngleLosFilter::valid_config(config));
    config.sigma_a_pitch = 10.0;
    config.estimated_target_distance = 0.0;
    CHECK(!guidance::InertialAngleLosFilter::valid_config(config));
    return 0;
}
}  // namespace

int main() {
    CHECK(verify_kalman_reference() == 0);
    CHECK(verify_inertial_rotation() == 0);
    CHECK(verify_compensated_angles() == 0);
    CHECK(verify_oscillation_suppression() == 0);
    CHECK(verify_distance_integration() == 0);
    CHECK(verify_distance_reset() == 0);
    CHECK(verify_compensation_switch() == 0);
    CHECK(verify_azimuth_wrap() == 0);
    CHECK(verify_reset_paths() == 0);
    CHECK(verify_config() == 0);
    return 0;
}
