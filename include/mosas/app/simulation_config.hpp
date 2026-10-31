#pragma once

#include <mosas/app/app_config.hpp>

namespace mosas::app {

/** @brief Original application configuration plus simulation-only connection settings. */
struct SimulationConfig {
    std::string socket_directory;
    runtime::TimestampNs sync_budget_ns = 10'000'000;
    std::filesystem::path app_config_path;
    AppConfig app{};
    SimulationConfig();
};

class SimulationConfigLoader {
public:
    /**
     * @brief Load a simulation wrapper and its referenced original application INI.
     * @param path Wrapper INI; relative paths resolve against its parent directory.
     * @param config Nonnull output, replaced only after complete validation.
     * @param error Optional file/line diagnostic.
     * @return Whether loading and validation succeeded; never starts hardware or workers.
     */
    static bool load(const std::filesystem::path& path, SimulationConfig* config,
                     std::string* error);
};

}  // namespace mosas::app
