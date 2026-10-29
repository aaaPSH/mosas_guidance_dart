#include <mosas/app/simulation_contract.hpp>
#include <mosas/adapters/simulation_session.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace mosas::app {
namespace {
void check(double actual, double expected, const std::string& field) {
    if (!std::isfinite(actual) || !std::isfinite(expected) ||
        std::abs(actual-expected) > 1e-6 * std::max({1.0,std::abs(actual),std::abs(expected)}))
        throw std::runtime_error("simulation " + field + " does not match original app_config");
}
}
void validate_simulation_metadata(const AppConfig& app,
                                  const dart_sim_transport::ServerSnapshot& metadata) {
    const auto& c=metadata.camera;
    if(c.width != static_cast<std::uint32_t>(app.camera.width))
        throw std::runtime_error("simulation camera width does not match original app_config");
    if(c.height != static_cast<std::uint32_t>(app.camera.height))
        throw std::runtime_error("simulation camera height does not match original app_config");
    if(c.stride != c.width*3 || c.pixel_format != dart_sim_transport::PixelFormat::rgb8)
        throw std::runtime_error("simulation camera stride/format is not packed RGB8");
    check(c.camera_hz,app.camera.fps,"camera_hz");
    check(c.imu_hz,app.imu.sample_rate_hz,"imu_hz");
    const auto& k=app.guidance.camera_intrinsics;
    check(c.fx,k.fx,"fx");check(c.fy,k.fy,"fy");
    check(c.cx,k.cx,"cx");check(c.cy,k.cy,"cy");
    const auto actual=adapters::simulation_camera_rotation(c.R_S_from_C).matrix();
    const auto expected=app.guidance.dart_from_camera.matrix();
    for(std::size_t i=0;i<actual.size();++i)check(actual[i],expected[i],"camera rotation");
    constexpr double g=dart_sim_transport::standard_gravity;
    check(metadata.session.g0_mps2,g,"gravity g0");
    check(metadata.session.world_gravity_mps2[0],0,"world gravity x");
    check(metadata.session.world_gravity_mps2[1],0,"world gravity y");
    check(metadata.session.world_gravity_mps2[2],-g,"world gravity z");
    check(app.algorithms.attitude.gravity_mps2,g,"attitude gravity");
    // 导引与输出映射由工厂统一注入上述重力，不再核验废弃算法副本。
}
}  // namespace mosas::app
