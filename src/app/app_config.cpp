#include <mosas/app/app_config.hpp>
#include "config_parsing.hpp"

namespace mosas::app {
bool AppConfigLoader::load(const std::filesystem::path& path,AppConfig* config,
                           std::string* error) {
    return config_parsing::load_hardware_config(path,config,error);
}
} // namespace mosas::app
