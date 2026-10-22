#include <mosas/app/app_config.hpp>
#include <mosas/app/default_guidance_algorithms.hpp>

#include "test_support.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <system_error>
#include <utility>

namespace {

struct TemporaryFile {
    explicit TemporaryFile(std::filesystem::path file_path)
        : path(std::move(file_path)) {}
    ~TemporaryFile() {
        std::error_code error;
        std::filesystem::remove(path, error);
    }
    std::filesystem::path path;
};

}  // namespace

int main() {
    mosas::app::AppConfig config;
    std::string error;
    const auto config_path =
        std::filesystem::path(MOSAS_SOURCE_DIR) / "config" /
        "mosas_guidance.conf";
    CHECK(mosas::app::AppConfigLoader::load(config_path, &config, &error));
    CHECK(config.camera.fps == 120);
    CHECK(config.command.send_rate_hz == 100);
    CHECK(config.command.command_timeout_ms == 100);
    CHECK(std::isfinite(config.algorithms.attitude.launch_speed_mps));
    CHECK(config.algorithms.attitude.launch_speed_mps > 0.0);
    CHECK(config.algorithms.attitude.launch_confirm_ns == 20'000'000);
    CHECK(config.algorithms.attitude.launch_detection_mode ==
          mosas::runtime::LaunchDetectionMode::two_stage);
    CHECK(config.algorithms.algorithm == mosas::app::GuidanceAlgorithm::body_vector_pn);
    CHECK(config.algorithms.png_guidance.control_channel == mosas::guidance::ControlChannel::both);
    CHECK(config.algorithms.body_vector_pn.control_channel == mosas::guidance::ControlChannel::both);
    CHECK_NEAR(config.algorithms.body_vector_filter.sigma_a, 4.0, 1e-12);
    CHECK_NEAR(config.algorithms.body_vector_filter.sigma_l, 0.0005, 1e-12);
    CHECK_NEAR(config.algorithms.body_vector_filter.initial_rate_covariance, 1.0, 1e-12);
    CHECK_NEAR(config.algorithms.body_vector_filter.maximum_gap_seconds, 0.2, 1e-12);
    CHECK_NEAR(config.algorithms.body_vector_pn.maximum_command_overload, 0.3, 1e-12);
    CHECK_NEAR(config.algorithms.body_vector_pn.maximum_command_acceleration, 9.80665, 1e-12);
    CHECK(config.algorithms.vision_config.initial_roi.width == 640);
    CHECK_NEAR(config.guidance.camera_intrinsics.fx,
               486.244430986463, 1e-9);
    CHECK_NEAR(config.algorithms.png_guidance.gravity,
               config.algorithms.attitude.gravity_mps2, 1e-12);

    std::ifstream source_config(config_path);
    CHECK(source_config.good());
    std::ostringstream source_text;
    source_text << source_config.rdbuf();
    std::string invalid_rotation_config = source_text.str();
    const std::string valid_matrix_entry = "camera_to_body_m00=1\n";
    const std::size_t matrix_entry_position =
        invalid_rotation_config.find(valid_matrix_entry);
    CHECK(matrix_entry_position != std::string::npos);
    invalid_rotation_config.replace(matrix_entry_position,
                                    valid_matrix_entry.size(),
                                    "camera_to_body_m00=2\n");
    const auto temp_path = std::filesystem::temp_directory_path() /
        ("mosas_invalid_rotation_" +
         std::to_string(std::chrono::steady_clock::now()
                            .time_since_epoch()
                            .count()) +
         ".conf");
    TemporaryFile temporary_config(temp_path);
    std::ofstream temp_output(temporary_config.path);
    CHECK(temp_output.good());
    temp_output << invalid_rotation_config;
    temp_output.close();
    CHECK(temp_output.good());
    mosas::app::AppConfig invalid_config;
    CHECK(!mosas::app::AppConfigLoader::load(temporary_config.path,
                                              &invalid_config, &error));
    CHECK(error.find("制导参数无效") != std::string::npos);

    auto dependencies =
        mosas::app::make_default_guidance_algorithms(config.algorithms);
    CHECK(dependencies.vision_detector != nullptr);
    CHECK(dependencies.flight_state_estimator != nullptr);
    CHECK(dependencies.los_estimator != nullptr);
    CHECK(dependencies.los_rate_filter != nullptr);
    CHECK(dependencies.guidance_law != nullptr);
    CHECK(dependencies.monotonic_clock != nullptr);
    CHECK(dynamic_cast<mosas::guidance::BodyVectorLosFilter*>(
              dependencies.los_rate_filter.get()) != nullptr);
    CHECK(dynamic_cast<mosas::guidance::BodyVectorPnGuidance*>(
              dependencies.guidance_law.get()) != nullptr);
    auto png_config = config.algorithms;
    png_config.algorithm = mosas::app::GuidanceAlgorithm::png;
    bool observed_png=false;
    auto observed_dependencies=mosas::app::make_default_guidance_algorithms(png_config,
        [&](const mosas::domain::GuidanceInput& input,const PngCalculationTrace& trace){
            observed_png=input.source_frame_id==91&&trace.valid;
        });
    mosas::domain::GuidanceInput observed_input;
    observed_input.source_frame_id=91;observed_input.line_of_sight={true,{0},{0}};
    observed_input.angular_velocity={true,{0},{0}};observed_input.attitude.valid=true;
    observed_input.speed_mps.value=config.algorithms.attitude.launch_speed_mps;
    CHECK(observed_dependencies.guidance_law->calculate(observed_input).valid);
    CHECK(observed_png);

    const auto load_variant = [&](const std::string& keys, mosas::app::AppConfig* output) {
        auto text = source_text.str();
        std::istringstream overrides(keys);
        std::string line;
        std::string additions;
        while (std::getline(overrides, line)) {
            const auto position = text.find("\n" + line.substr(0, line.find('=')) + "=");
            if (position == std::string::npos) {
                additions += line + '\n';
            } else {
                const auto end = text.find('\n', position + 1);
                text.replace(position + 1, end - position - 1, line);
            }
        }
        std::ofstream file(temporary_config.path);
        file << text << "\n[guidance]\n" << additions;
        file.close();
        return file.good() && mosas::app::AppConfigLoader::load(temporary_config.path, output, &error);
    };
    mosas::app::AppConfig command_config;
    CHECK(load_variant("send_rate_hz=50\ncommand_timeout_ms=40\n", &command_config));
    CHECK(command_config.command.send_rate_hz == 50);
    CHECK(command_config.command.command_timeout_ms == 40);
    for (const auto* invalid : {"send_rate_hz=0\n", "send_rate_hz=-1\n",
                                "send_rate_hz=1001\n", "send_rate_hz=1.5\n",
                                "command_timeout_ms=0\n", "command_timeout_ms=-1\n"}) {
        CHECK(!load_variant(invalid, &command_config));
    }
    {
        auto text = source_text.str();
        for (const auto* key : {"send_rate_hz", "command_timeout_ms"}) {
            const auto begin = text.find("\n" + std::string(key) + "=");
            CHECK(begin != std::string::npos);
            text.erase(begin, text.find('\n', begin + 1) - begin);
        }
        std::ofstream file(temporary_config.path);
        file << text;
        file.close();
        CHECK(mosas::app::AppConfigLoader::load(temporary_config.path, &command_config, &error));
        CHECK(command_config.command.send_rate_hz == 100);
        CHECK(command_config.command.command_timeout_ms == 100);
    }
    // Omitting the algorithm selector still uses PNG, independently of the shipped configuration.
    {
        auto text = source_text.str();
        const auto begin = text.find("\nguidance_algorithm=");
        CHECK(begin != std::string::npos);
        const auto end = text.find('\n', begin + 1);
        text.erase(begin, end - begin);
        { std::ofstream file(temporary_config.path); file << text; }
        mosas::app::AppConfig without_algorithm;
        CHECK(mosas::app::AppConfigLoader::load(
            temporary_config.path, &without_algorithm, &error));
        CHECK(without_algorithm.algorithms.algorithm == mosas::app::GuidanceAlgorithm::png);
    }
    // 缺省通道保持双通道；同一配置项同时应用于两种制导律。
    {
        auto text = source_text.str();
        const auto begin = text.find("\ncontrol_channel=");
        CHECK(begin != std::string::npos);
        text.erase(begin, text.find('\n', begin + 1) - begin);
        { std::ofstream file(temporary_config.path); file << text; }
        mosas::app::AppConfig without_channel;
        CHECK(mosas::app::AppConfigLoader::load(temporary_config.path, &without_channel, &error));
        CHECK(without_channel.algorithms.png_guidance.control_channel == mosas::guidance::ControlChannel::both);
        CHECK(without_channel.algorithms.body_vector_pn.control_channel == mosas::guidance::ControlChannel::both);
    }
    for (const std::string channel : {"both", "pitch", "yaw"}) {
        for (const std::string algorithm : {"png", "body_vector_pn"}) {
            mosas::app::AppConfig selected;
            CHECK(load_variant("control_channel=" + channel + "\nguidance_algorithm=" + algorithm + "\n", &selected));
            const auto expected_channel = channel == "pitch" ? mosas::guidance::ControlChannel::pitch
                                        : channel == "yaw" ? mosas::guidance::ControlChannel::yaw
                                                           : mosas::guidance::ControlChannel::both;
            CHECK(selected.algorithms.png_guidance.control_channel == expected_channel);
            CHECK(selected.algorithms.body_vector_pn.control_channel == expected_channel);
            auto selected_dependencies = mosas::app::make_default_guidance_algorithms(selected.algorithms);
            auto input = observed_input;
            input.angular_velocity = {true, {0.2}, {-0.4}};
            input.vector_los = {true, {1.0, 0.0, 0.0}, {0.0, 0.2, -0.4}};
            const auto output = selected_dependencies.guidance_law->calculate(input);
            CHECK(output.valid && output.command.valid);
            const auto command = output.command.overload_g;
            if (channel == "pitch") {
                CHECK_NEAR(command.y, 0.3, 1e-12);
                CHECK(command.z == 0.0);
            } else if (channel == "yaw") {
                CHECK(command.y == 0.0);
                CHECK_NEAR(command.z, -0.3, 1e-12);
            } else {
                CHECK(command.y > 0.0 && command.z < 0.0);
                CHECK_NEAR(std::hypot(command.y, command.z), 0.3, 1e-12);
            }
            CHECK_NEAR(output.diagnostics.body_overload.y, command.y, 0.0);
            CHECK_NEAR(output.diagnostics.body_overload.z, command.z, 0.0);
        }
    }
    for (const auto* invalid : {"unknown", "pitch_only", "yaw_only", "Pitch", "1"}) {
        mosas::app::AppConfig rejected;
        CHECK(!load_variant("control_channel=" + std::string(invalid) + "\n", &rejected));
        CHECK(error.find("guidance.control_channel") != std::string::npos);
    }

    // Existing configurations without the selector retain the acceleration trigger.
    {
        auto text = source_text.str();
        const auto begin = text.find("\nlaunch_detection_mode=");
        CHECK(begin != std::string::npos);
        const auto end = text.find('\n', begin + 1);
        text.erase(begin, end - begin);
        { std::ofstream file(temporary_config.path); file << text; }
        mosas::app::AppConfig without_selector;
        CHECK(mosas::app::AppConfigLoader::load(
            temporary_config.path, &without_selector, &error));
        CHECK(without_selector.algorithms.attitude.launch_detection_mode ==
              mosas::runtime::LaunchDetectionMode::acceleration);
    }
    for (const std::string mode : {"acceleration", "two_stage"}) {
        mosas::app::AppConfig selected;
        CHECK(load_variant("launch_detection_mode=" + mode + "\n", &selected));
        const bool two_stage = mode == "two_stage";
        CHECK(selected.algorithms.attitude.launch_detection_mode ==
              (two_stage ? mosas::runtime::LaunchDetectionMode::two_stage
                         : mosas::runtime::LaunchDetectionMode::acceleration));
        auto selected_dependencies =
            mosas::app::make_default_guidance_algorithms(selected.algorithms);
        mosas::domain::ImuKinematicSample imu;
        imu.time.monotonic_ns = 0;
        imu.acceleration = {0.0, selected.algorithms.attitude.gravity_mps2, 0.0};
        selected_dependencies.flight_state_estimator->update(imu);
        imu.acceleration = {30.0, 0.0, 0.0};
        imu.time.monotonic_ns = 5'000'000;
        selected_dependencies.flight_state_estimator->update(imu);
        imu.time.monotonic_ns = 25'000'000;
        const auto acceleration_state =
            selected_dependencies.flight_state_estimator->update(imu);
        CHECK((acceleration_state.launch.phase ==
               mosas::domain::LaunchPhase::launched) == !two_stage);
        imu.acceleration = {1.0, 0.0, 0.0};
        imu.time.monotonic_ns = 30'000'000;
        selected_dependencies.flight_state_estimator->update(imu);
        imu.time.monotonic_ns = 40'000'000;
        const auto triggered = selected_dependencies.flight_state_estimator->update(imu);
        CHECK(triggered.launch.phase == mosas::domain::LaunchPhase::launched);
        CHECK(triggered.launch.transition_time.monotonic_ns ==
              (two_stage ? 40'000'000 : 25'000'000));
    }
    for (const double speed : {13.0, 16.0}) {
        mosas::app::AppConfig speed_config;
        auto speed_text = source_text.str();
        const std::string key = "\nlaunch_speed_mps=";
        const auto key_position = speed_text.find(key);
        CHECK(key_position != std::string::npos);
        const auto value_begin = key_position + key.size();
        const auto value_end = speed_text.find('\n', value_begin);
        CHECK(value_end != std::string::npos);
        speed_text.replace(value_begin, value_end - value_begin,
                           std::to_string(speed));
        {
            std::ofstream file(temporary_config.path);
            CHECK(file.good());
            file << speed_text;
        }
        CHECK(mosas::app::AppConfigLoader::load(temporary_config.path,
                                               &speed_config, &error));
        CHECK_NEAR(speed_config.algorithms.attitude.launch_speed_mps, speed, 1e-12);
        speed_config.algorithms.attitude.launch_detection_mode =
            mosas::runtime::LaunchDetectionMode::two_stage;
        for (const auto algorithm : {mosas::app::GuidanceAlgorithm::png,
                                     mosas::app::GuidanceAlgorithm::body_vector_pn}) {
            speed_config.algorithms.algorithm = algorithm;
            auto speed_dependencies =
                mosas::app::make_default_guidance_algorithms(speed_config.algorithms);
            CHECK(speed_dependencies.guidance_law != nullptr);
            mosas::domain::ImuKinematicSample imu;
            imu.time.monotonic_ns = 0;
            imu.acceleration = {0.0, speed_config.algorithms.attitude.gravity_mps2, 0.0};
            CHECK_NEAR(speed_dependencies.flight_state_estimator->update(imu)
                           .speed_mps.value, speed, 1e-12);
            imu.acceleration.x = 30.0;
            imu.time.monotonic_ns = 10'000'000;
            speed_dependencies.flight_state_estimator->update(imu);
            imu.time.monotonic_ns = 30'000'000;
            const auto rail_state = speed_dependencies.flight_state_estimator->update(imu);
            CHECK(rail_state.launch.phase == mosas::domain::LaunchPhase::pre_launch);
            imu.acceleration = {1.0, 0.0, 0.0};
            imu.time.monotonic_ns = 40'000'000;
            speed_dependencies.flight_state_estimator->update(imu);
            imu.time.monotonic_ns = 50'000'000;
            const auto launched_state = speed_dependencies.flight_state_estimator->update(imu);
            CHECK(launched_state.launch.phase == mosas::domain::LaunchPhase::launched);
            CHECK_NEAR(launched_state.speed_mps.value, speed, 1e-12);
            imu.time.monotonic_ns = 60'000'000;
            imu.acceleration.x = 2.0;
            CHECK_NEAR(speed_dependencies.flight_state_estimator->update(imu)
                           .speed_mps.value, speed + 0.02, 1e-12);
        }
    }
    mosas::app::AppConfig vector_config;
    CHECK(load_variant("guidance_algorithm=body_vector_pn\n", &vector_config));
    CHECK(vector_config.algorithms.algorithm == mosas::app::GuidanceAlgorithm::body_vector_pn);
    CHECK_NEAR(vector_config.algorithms.body_vector_pn.navigation_constant, 3.0, 1e-12);
    CHECK_NEAR(vector_config.algorithms.body_vector_filter.maximum_gap_seconds, 0.2, 1e-12);
    CHECK_NEAR(vector_config.algorithms.attitude.launch_release_acceleration_max_mps2, 3.0, 1e-12);
    CHECK(vector_config.algorithms.attitude.launch_release_confirm_ns == 10'000'000);
    CHECK(vector_config.algorithms.attitude.launch_release_timeout_ns == 500'000'000);
    CHECK(vector_config.algorithms.attitude.maximum_sample_gap_ns == 20'000'000);
    auto vector_dependencies = mosas::app::make_default_guidance_algorithms(vector_config.algorithms);
    CHECK(dynamic_cast<mosas::guidance::BodyVectorLosFilter*>(
              vector_dependencies.los_rate_filter.get()) != nullptr);
    CHECK(dynamic_cast<mosas::guidance::BodyVectorPnGuidance*>(
              vector_dependencies.guidance_law.get()) != nullptr);
    mosas::domain::LosRateFilterInput filter_input;
    filter_input.attitude.valid = true;
    filter_input.line_of_sight = {true, {0.0}, {0.0}};
    filter_input.dt_seconds = 0.01;
    CHECK(!vector_dependencies.los_rate_filter->update(filter_input).valid);
    const auto vector_rate = vector_dependencies.los_rate_filter->update(filter_input);
    CHECK(vector_rate.valid && vector_rate.vector.valid);
    mosas::domain::GuidanceInput guidance_input;
    guidance_input.attitude = filter_input.attitude;
    guidance_input.vector_los = vector_rate.vector;
    guidance_input.speed_mps.value = 13.0;
    CHECK_NEAR(vector_dependencies.guidance_law->calculate(guidance_input).command.overload_g.y,
               0.3, 1e-12);

    CHECK(load_variant("guidance_algorithm=body_vector_pn\n"
                       "body_vector_navigation_constant=4.5\n"
                       "body_vector_los_sigma_a=2\n"
                       "body_vector_los_sigma_l=0.001\n"
                       "body_vector_los_initial_rate_covariance=0.5\n"
                       "body_vector_los_maximum_gap_ms=80\n"
                       "body_vector_maximum_command_overload=3\n"
                       "body_vector_maximum_command_acceleration=inf\n", &vector_config));
    CHECK_NEAR(vector_config.algorithms.body_vector_pn.navigation_constant, 4.5, 1e-12);
    CHECK_NEAR(vector_config.algorithms.body_vector_filter.sigma_a, 2.0, 1e-12);
    CHECK_NEAR(vector_config.algorithms.body_vector_filter.sigma_l, 0.001, 1e-12);
    CHECK_NEAR(vector_config.algorithms.body_vector_filter.initial_rate_covariance, 0.5, 1e-12);
    CHECK_NEAR(vector_config.algorithms.body_vector_filter.maximum_gap_seconds, 0.08, 1e-12);
    CHECK(std::isinf(vector_config.algorithms.body_vector_pn.maximum_command_acceleration));
    for (const char* bad : {
             "launch_detection_mode=unknown\n", "launch_detection_mode=1\n",
             "launch_detection_mode=Two_Stage\n",
             "launch_speed_mps=0\n", "launch_speed_mps=-1\n",
             "launch_speed_mps=inf\n", "launch_speed_mps=nan\n",
             "launch_release_acceleration_max_mps2=0\n",
             "launch_release_acceleration_max_mps2=-1\n",
             "launch_release_acceleration_max_mps2=9.6\n",
             "launch_release_acceleration_max_mps2=nan\n",
             "launch_release_acceleration_max_mps2=inf\n",
             "launch_release_confirm_ms=0\n", "launch_release_confirm_ms=-1\n",
             "launch_release_confirm_ms=inf\n", "launch_release_confirm_ms=500\n",
             "launch_release_timeout_ms=0\n", "launch_release_timeout_ms=-1\n",
             "launch_release_timeout_ms=5\n", "launch_release_timeout_ms=10\n",
             "launch_release_timeout_ms=inf\n",
             "attitude_maximum_gap_ms=0\n", "attitude_maximum_gap_ms=-1\n",
             "attitude_maximum_gap_ms=nan\n",
             "guidance_algorithm=unknown\n", "body_vector_navigation_constant=0\n",
             "body_vector_los_sigma_a=0\n", "body_vector_los_sigma_l=0\n",
             "body_vector_los_initial_rate_covariance=-1\n",
             "body_vector_los_maximum_gap_ms=0\n",
             "body_vector_maximum_command_overload=-1\n",
             "body_vector_maximum_command_acceleration=-1\n",
             "body_vector_los_sigma_a=nan\n"}) {
        CHECK(!load_variant(bad, &vector_config));
    }
    CHECK(load_variant("launch_release_acceleration_max_mps2=2.5\n"
                       "launch_release_confirm_ms=15\n"
                       "launch_release_timeout_ms=250\n"
                       "attitude_maximum_gap_ms=12\n", &vector_config));
    CHECK_NEAR(vector_config.algorithms.attitude.launch_release_acceleration_max_mps2, 2.5, 1e-12);
    CHECK(vector_config.algorithms.attitude.launch_release_confirm_ns == 15'000'000);
    CHECK(vector_config.algorithms.attitude.launch_release_timeout_ns == 250'000'000);
    CHECK(vector_config.algorithms.attitude.maximum_sample_gap_ns == 12'000'000);
    CHECK(load_variant("body_vector_maximum_command_overload=0\n", &vector_config));

    // 工厂使用与姿态估计相同的重力，不采用独立且可能冲突的 PN 重力副本。
    vector_config.algorithms.algorithm = mosas::app::GuidanceAlgorithm::body_vector_pn;
    vector_config.algorithms.attitude.gravity_mps2 = 10.0;
    vector_config.algorithms.body_vector_pn.maximum_command_overload = 3.0;
    vector_config.algorithms.body_vector_pn.maximum_command_acceleration = 100.0;
    vector_dependencies = mosas::app::make_default_guidance_algorithms(vector_config.algorithms);
    CHECK_NEAR(vector_dependencies.guidance_law->calculate(guidance_input)
                   .diagnostics.body_acceleration.y, 15.0, 1e-12);
    guidance_input.vector_los.inertial_direction_rate.y =
        -10.0 / (2.0 * guidance_input.speed_mps.value);
    const auto parabolic_output = vector_dependencies.guidance_law->calculate(guidance_input);
    CHECK(parabolic_output.valid);
    CHECK_NEAR(parabolic_output.command.overload_g.y, 0.0, 1e-12);
    return 0;
}
