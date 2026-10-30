#include <mosas/app/device_application.hpp>
#include "test_support.hpp"
#include <iostream>
#include <sstream>
#include <stdexcept>

int main() {
    std::ostringstream captured;
    auto* original = std::cerr.rdbuf(captured.rdbuf());
    {
        mosas::app::DeviceApplicationComponents components;
        components.report_acquisition_time();
        components.report_acquisition_time();
    }
    const auto first = captured.str();
    const auto marker = first.find("source=camera");
    const bool once = marker != std::string::npos && first.find("source=camera", marker + 1) == std::string::npos;
    captured.str({});
    try {
        mosas::app::DeviceApplicationComponents partial;
        auto camera = std::make_unique<mosas::app::NoriSdkCameraSource>(mosas::app::CameraAppConfig{});
        partial.camera_source_status = camera.get();
        partial.camera_source = std::move(camera);
        throw std::runtime_error("initialization failed before camera start");
    } catch (const std::runtime_error&) {}
    std::cerr.rdbuf(original);
    CHECK(once);
    CHECK(captured.str().find("source=camera configured_hz=30") != std::string::npos);
    CHECK(captured.str().find("quality=not_synchronized") != std::string::npos);
    return 0;
}
