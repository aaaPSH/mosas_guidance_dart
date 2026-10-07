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
    CHECK(config.guidance.launch_speed_mps == 13.0);
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
    return 0;
}
