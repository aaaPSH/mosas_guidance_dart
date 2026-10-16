#include <mosas/app/simulation_config.hpp>

#include "config_parsing.hpp"

#include <cmath>
#include <limits>
#include <sys/un.h>
#include <unistd.h>

namespace mosas::app {
namespace {
bool close(double value, double expected) {
    return std::isfinite(value) && std::abs(value - expected) <= 1e-6;
}
}

SimulationConfig::SimulationConfig() {
    socket_directory = "/tmp/dart-sim-" + std::to_string(::geteuid()) +
                       "/dart_projectile";
}

bool SimulationConfigLoader::load(const std::filesystem::path& path,
                                  SimulationConfig* config, std::string* error) {
    using namespace config_parsing;
    if (!config) return fail_global(path, "配置输出指针为空", error);
    SimulationConfig loaded;
    std::string app_path, recording_directory;
    std::set<std::string> seen;
    if (!read_ini(path, {"simulation"}, [&](std::size_t line,
                     const std::string&, const std::string& key,
                     const std::string& value) {
        if (!seen.insert(key).second)
            return fail(path, line, "配置键重复: " + key, error);
        if (key == "app_config") app_path = value;
        else if (key == "recording_directory") recording_directory = value;
        else if (key == "socket_directory") loaded.socket_directory = value;
        else if (key == "sync_budget_ms") {
            double ms = 0;
            if (!parse_double(value, &ms) || ms * 1e6 < 1 ||
                ms >= static_cast<double>(std::numeric_limits<runtime::TimestampNs>::max()) / 1e6)
                return fail(path, line, "sync_budget_ms 无效", error);
            loaded.sync_budget_ns = static_cast<runtime::TimestampNs>(ms * 1e6);
        } else return fail(path, line, "未知仿真配置键: " + key, error);
        return true;
    }, error)) return false;
    if (app_path.empty() || recording_directory.empty())
        return fail_global(path, "必须配置非空 app_config 和 recording_directory", error);
    const auto& endpoint = loaded.socket_directory;
    if (endpoint.empty() || endpoint.front() != '/' || endpoint.back() == '/' ||
        endpoint.find("//") != std::string::npos ||
        endpoint.size() + sizeof("/control.sock") > sizeof(sockaddr_un::sun_path))
        return fail_global(path, "Socket 目录必须为可表示的绝对路径", error);
    for (const auto& part : std::filesystem::path(endpoint))
        if (part == "." || part == "..")
            return fail_global(path, "Socket 目录含不安全路径分量", error);
    const auto parent = std::filesystem::absolute(path).parent_path();
    loaded.app_config_path = (parent / app_path).lexically_normal();
    if (!AppConfigLoader::load(loaded.app_config_path, &loaded.app, error)) return false;
    if (loaded.app.camera.capture_mode != runtime::CameraCaptureMode::free_run)
        return fail_global(path, "仿真相机仅支持 free_run", error);
    if (loaded.app.output.mode != output::OutputMode::debug &&
        loaded.app.output.mode != output::OutputMode::match)
        return fail_global(path, "仿真内录要求原配置 output.mode=debug|match", error);
    const auto& k = loaded.app.camera.camera_matrix;
    if (!close(k.m01, 0) || !close(k.m10, 0) || !close(k.m20, 0) ||
        !close(k.m21, 0) || !close(k.m22, 1))
        return fail_global(path, "DSIM 相机仅支持标准零 skew 的针孔内参矩阵", error);
    auto filename = std::filesystem::path(loaded.app.output.recording_path).filename();
    if (filename.empty()) filename = "mosas_guidance.avi";
    loaded.app.output.recording_path =
        (parent / recording_directory / filename).lexically_normal().string();
    *config = std::move(loaded);
    if (error) error->clear();
    return true;
}

}  // namespace mosas::app
