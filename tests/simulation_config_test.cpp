#include <mosas/app/default_guidance_algorithms.hpp>
#include <mosas/app/simulation_config.hpp>
#include "test_support.hpp"
#include "body_vector_test_support.hpp"
#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <unistd.h>

struct Fixture {
    std::filesystem::path directory;
    std::string original;
    Fixture() {
        char name[] = "/tmp/mosas-config-XXXXXX";
        directory = mkdtemp(name);
        std::ifstream input(std::filesystem::path(MOSAS_SOURCE_DIR) / "config/mosas_guidance.conf");
        std::ostringstream text; text << input.rdbuf(); original = text.str();
        std::ofstream(directory / "app.conf") << original;
    }
    ~Fixture() { std::filesystem::remove_all(directory); }
    std::filesystem::path write(const std::string& text) {
        const auto path = directory / "case.conf";
        std::ofstream(path) << text;
        return path;
    }
    void change(const std::string& from, const std::string& to) {
        auto text = original; const auto at = text.find(from);
        if (at == std::string::npos) throw std::runtime_error("fixture key absent");
        text.replace(at, from.size(), to);
        std::ofstream(directory / "app.conf") << text;
    }
};
int main() {
    using namespace mosas;
    Fixture fixture; app::SimulationConfig cfg; std::string error;
    const std::string base = "[simulation]\napp_config=app.conf\nrecording_directory=records\n";
    auto load = [&](const std::string& text) {
        return app::SimulationConfigLoader::load(fixture.write(text), &cfg, &error);
    };
    CHECK(load(base));
    CHECK(cfg.app.camera.width == 640 && cfg.app.camera.height == 480);
    CHECK(cfg.app.camera.fps == 120 && cfg.app.imu.sample_rate_hz == 1000);
    CHECK(cfg.app.algorithms.attitude.launch_speed_mps == 16);
    CHECK(cfg.app.algorithms.attitude.launch_detection_mode == runtime::LaunchDetectionMode::two_stage);
    CHECK(cfg.app.algorithms.algorithm == app::GuidanceAlgorithm::body_vector_pn);
    CHECK_NEAR(cfg.app.algorithms.body_vector_filter.sigma_a, 4.0, 1e-12);
    CHECK_NEAR(cfg.app.algorithms.body_vector_filter.sigma_l, 0.0005, 1e-12);
    CHECK_NEAR(cfg.app.algorithms.body_vector_filter.initial_rate_covariance, 1.0, 1e-12);
    CHECK_NEAR(cfg.app.algorithms.body_vector_filter.maximum_gap_seconds, 0.2, 1e-12);
    CHECK_NEAR(cfg.app.algorithms.body_vector_pn.maximum_command_overload, 0.3, 1e-12);
    CHECK_NEAR(cfg.app.algorithms.body_vector_pn.maximum_command_acceleration, 9.80665, 1e-12);
    auto algorithms = app::make_default_guidance_algorithms(cfg.app.algorithms);
    domain::GuidanceInput input;
    input.attitude.valid = true;
    input.line_of_sight = {true, {0}, {0}};
    input.angular_velocity = {true, {0}, {0}};
    input.vector_los = {true, {1, 0, 0}, {0, 0, 0}};
    input.speed_mps = {cfg.app.algorithms.attitude.launch_speed_mps};
    auto command = algorithms.guidance_law->calculate(input);
    CHECK(command.valid);
    // Vector PN includes transverse gravity in the same 0.3g combined limit.
    CHECK_NEAR(command.command.overload_g.y, 0.3, 1e-12);
    input.attitude = test::vector_test_attitude(0.6108652381980153, 0.7);
    input.angular_velocity = {true, {0.5}, {-0.4}};
    input.vector_los.inertial_direction_rate = {0, 0.5, -0.4};
    command = algorithms.guidance_law->calculate(input);
    CHECK(command.valid);
    CHECK_NEAR(std::hypot(command.command.overload_g.y,
                          command.command.overload_g.z), 0.3, 1e-12);
    CHECK_NEAR(command.diagnostics.command_overload.value, 0.3, 1e-12);
    CHECK(cfg.app.algorithms.vision_config.initial_roi.width == 640);
    CHECK(cfg.app.algorithms.vision_config.initial_roi.height == 480);
    CHECK(cfg.app.algorithms.vision_config.min_blob_area == 1);
    CHECK(cfg.app.algorithms.vision_config.threshold.h_max == 100);
    CHECK(cfg.app.output.mode == output::OutputMode::match);
    CHECK(cfg.app.output.match_window_duration_ms == 4000);
    CHECK(cfg.app.output.recording_path == (fixture.directory / "records/mosas_guidance.avi").string());
    app::AppConfig original;
    CHECK(app::AppConfigLoader::load(fixture.directory / "app.conf", &original, &error));
    CHECK(cfg.app.guidance.camera_intrinsics.fx == original.guidance.camera_intrinsics.fx);
    CHECK(cfg.app.algorithms.body_vector_pn.maximum_command_overload == original.algorithms.body_vector_pn.maximum_command_overload);
    CHECK(cfg.app.algorithms.los_rate_filter.sigma_a_pitch == original.algorithms.los_rate_filter.sigma_a_pitch);
    CHECK(cfg.app.serial.device == original.serial.device); // Parsed, never opened.
    for (const auto& bad : {"[simulation]\napp_config=app.conf\n", "[simulation]\nrecording_directory=x\n",
                           "[simulation]\napp_config=absent.conf\nrecording_directory=x\n"}) CHECK(!load(bad));
    for (const auto& suffix : {"app_config=app.conf\n", "sync_budget_ms=0\n", "sync_budget_ms=nan\n",
                              "socket_directory=relative\n", "socket_directory=/tmp/../unsafe\n",
                              "[guidance]\nlaunch_speed_mps=25\n", "[output]\nmode=debug\n"}) CHECK(!load(base + suffix));
    CHECK(cfg.app.algorithms.attitude.launch_speed_mps == 16); // Atomic on failure.
    fixture.change("mode=match", "mode=rtsp"); CHECK(!load(base));
    fixture.change("mode=free_run", "mode=hardware_trigger"); CHECK(!load(base));
    fixture.change("matrix_m01=0", "matrix_m01=1"); CHECK(!load(base));
    fixture.change("match_window_duration_ms=4000", "match_window_duration_ms=20000"); CHECK(!load(base));
    fixture.change("launch_speed_mps=16", "launch_speed_mps=17"); CHECK(load(base));
    CHECK(cfg.app.algorithms.attitude.launch_speed_mps == 17);
    const auto cwd = std::filesystem::current_path();
    std::filesystem::current_path("/");
    const bool from_other_cwd = app::SimulationConfigLoader::load(fixture.directory / "case.conf", &cfg, &error);
    std::filesystem::current_path(cwd);
    CHECK(from_other_cwd);
    CHECK(cfg.app_config_path == fixture.directory / "app.conf");
    CHECK(!app::SimulationConfigLoader::load(fixture.directory / "case.conf", nullptr, &error));
    return 0;
}
