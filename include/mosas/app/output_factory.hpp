#ifndef MOSAS_APP_OUTPUT_FACTORY_HPP
#define MOSAS_APP_OUTPUT_FACTORY_HPP

#include <mosas/app/app_config.hpp>
#include <mosas/runtime/output_ports.hpp>

#include <iosfwd>
#include <memory>

namespace mosas::app {

runtime::RuntimeOutputPorts make_runtime_output_ports(
    const AppConfig& config, std::ostream& log_output,
    std::unique_ptr<runtime::IVisualizationSink> visualization_override =
        nullptr);

}  // namespace mosas::app

#endif  // MOSAS_APP_OUTPUT_FACTORY_HPP
