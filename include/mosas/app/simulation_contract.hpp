#pragma once
#include <mosas/app/app_config.hpp>
#include <dart_sim_transport/endpoint.hpp>

namespace mosas::app {
/**
 * @brief Reject a simulation handshake inconsistent with the original application configuration.
 * @param app Validated original configuration, without hardware device activation.
 * @param metadata Actual session/sensor metadata from the Gazebo transport.
 * @throws std::runtime_error On mismatched geometry, rates, calibration or gravity units.
 */
void validate_simulation_metadata(const AppConfig& app,
                                  const dart_sim_transport::ServerSnapshot& metadata);
}  // namespace mosas::app
