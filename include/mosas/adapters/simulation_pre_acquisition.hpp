#pragma once

#include <mosas/ports/vision_detector.hpp>
#include <mosas/runtime/runtime_interfaces.hpp>

#include <functional>
#include <memory>
#include <optional>

namespace mosas::adapters {
class SimulationAudit;
/** @brief External simulation output policy; original algorithms and their results are unchanged.
 * One instance belongs to one runtime/session. Only a fresh, healthy, naturally launched
 * frame with an actual not-found observation may replace its neutral command by +Y.
 * The first real detection latches acquisition, including detections before launch;
 * later target loss never rearms the preset. No Gazebo target/attitude truth is used.
 */
class SimulationPreAcquisitionPolicy {
public:
    using FlightState = std::function<std::optional<runtime::ImuStateSnapshot>()>;
    struct Ports {
        std::unique_ptr<ports::IVisionDetector> vision_detector;
        std::unique_ptr<runtime::CommandSink> command_sink;
    };
    struct Diagnostics {
        std::uint64_t preset_submitted = 0;
        std::int64_t first_preset_ns = -1;
        std::int64_t last_preset_ns = -1;
        std::int64_t first_detection_ns = -1;
    };
    /** @brief Configure an external preset from the original PNG overload limit.
     * @param maximum_overload_g Finite positive original combined overload limit, in g.
     * @param flight_state Read-only query of the original processed IMU/launch state.
     * @param audit Optional read-only diagnostic sink; never supplies controller inputs.
     * @throws std::invalid_argument For an invalid limit or missing state query.
     */
    SimulationPreAcquisitionPolicy(double maximum_overload_g, FlightState flight_state,
                                  std::shared_ptr<SimulationAudit> audit = {});
    /** @brief Wrap one original vision port and output sink without changing their contracts.
     * @param vision_detector Original algorithm instance; all detections pass through unchanged.
     * @param command_sink Original nonblocking output adapter; retains generation/cancel/stop guards.
     * @return Ports sharing only observed acquisition state and external preset selection.
     * @throws std::invalid_argument For null ports or wrapping a policy more than once.
     */
    Ports wrap(std::unique_ptr<ports::IVisionDetector> vision_detector,
               std::unique_ptr<runtime::CommandSink> command_sink);
    /** @brief Query policy submissions, not physical acknowledgements or algorithm diagnostics.
     * @return Thread-safe per-runtime submission counts and source simulation timestamps.
     */
    Diagnostics diagnostics() const noexcept;
private:
    struct State;
    class ObservingVision;
    class SelectingSink;
    std::shared_ptr<State> state_;
};
} // namespace mosas::adapters
