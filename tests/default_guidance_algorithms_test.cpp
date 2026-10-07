#include <mosas/app/app_config.hpp>
#include <mosas/app/default_guidance_algorithms.hpp>

#include "test_support.hpp"

#include <filesystem>
#include <string>

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
