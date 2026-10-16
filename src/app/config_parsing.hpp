#pragma once
#include <mosas/app/app_config.hpp>
#include <functional>
#include <set>

namespace mosas::app::config_parsing {
using IniVisitor=std::function<bool(std::size_t,const std::string&,const std::string&,const std::string&)>;
/** @brief Strict shared INI reader; visits nonempty keys in allowed sections with line numbers. */
bool read_ini(const std::filesystem::path& path,const std::set<std::string>& sections,const IniVisitor& visitor,std::string* error);
/** @brief Shared key mapping, including paired guidance selection; records duplicate keys. */
bool parse_key(const std::filesystem::path& path,std::size_t line,const std::string& section,
  const std::string& key,const std::string& value,AppConfig* config,std::set<std::string>* seen,std::string* error);
/** @brief Shared algorithm and runtime validation; excludes all hardware/output requirements. */
bool validate_guidance(const std::filesystem::path& path,const AppConfig& config,std::string* error);
/** @brief Hardware loader preserving Nori/serial/output validation. */
bool load_hardware_config(const std::filesystem::path& path,AppConfig* config,std::string* error);
bool fail(const std::filesystem::path& path,std::size_t line,const std::string& message,std::string* error);
bool fail_global(const std::filesystem::path& path,const std::string& message,std::string* error);
bool parse_double(const std::string& text,double* value);
} // namespace mosas::app::config_parsing
