#include <mosas/adapters/simulation_pre_acquisition.hpp>
#include <mosas/adapters/simulation_audit.hpp>

#include <cmath>
#include <mutex>
#include <stdexcept>
#include <utility>

namespace mosas::adapters {
namespace {
bool simulation_time(const domain::SensorTime& time) {
    return time.is_valid() && time.source_time_ns == time.monotonic_ns &&
           time.uncertainty_ns == 0 && time.source_domain == domain::ClockDomain::simulation &&
           time.source == domain::TimestampSource::simulation &&
           time.quality == domain::TimestampQuality::simulation;
}
}
struct SimulationPreAcquisitionPolicy::State {
    struct Observation {
        std::uint64_t frame_id;
        domain::SensorTime time;
        std::uint64_t generation;
        bool healthy_launched_not_found;
    };
    State(double value, FlightState query, std::shared_ptr<SimulationAudit> sink)
        : limit(value), flight_state(std::move(query)), audit(std::move(sink)) {}
    std::mutex mutex;
    const double limit;
    const FlightState flight_state;
    const std::shared_ptr<SimulationAudit> audit;
    bool wrapped = false, active = false, acquired = false;
    std::uint64_t generation = 0;
    std::optional<Observation> observation;
    Diagnostics diagnostics;
};

/** @brief Observe original detections without changing pixels, detections, or algorithm state. */
class SimulationPreAcquisitionPolicy::ObservingVision final : public ports::IVisionDetector {
public:
    ObservingVision(std::unique_ptr<ports::IVisionDetector> original, std::shared_ptr<State> state)
        : original_(std::move(original)), state_(std::move(state)) {}
    domain::VisionDetection detect(const domain::ImageView& image, std::uint64_t frame_id,
                                    const domain::SensorTime& time) override {
        auto result = original_->detect(image, frame_id, time);
        // Query outside the policy lock: this callback reads the original runtime's IMU state.
        const auto flight = state_->flight_state();
        std::lock_guard<std::mutex> lock(state_->mutex);
        if (result.observation.valid && !state_->acquired) {
            state_->acquired = true;
            state_->diagnostics.first_detection_ns = time.monotonic_ns;
        }
        if(state_->audit&&!state_->audit->record_vision(frame_id,time,
                flight?std::optional<std::uint64_t>{flight->launch.generation}:std::nullopt,
                result.observation.valid,static_cast<std::uint32_t>(result.observation.reason)))
            throw std::runtime_error("simulation vision audit failed");
        const bool eligible = !result.observation.valid &&
            result.observation.reason == domain::ObservationReason::not_found &&
            simulation_time(time) && flight && simulation_time(flight->time) && flight->attitude.valid &&
            flight->time.monotonic_ns >= time.monotonic_ns &&
            flight->launch.phase == domain::LaunchPhase::launched &&
            flight->launch.reason == domain::LaunchReason::threshold_confirmed;
        state_->observation = State::Observation{frame_id, time,
            flight ? flight->launch.generation : 0, eligible};
        return result;
    }
    void reset() noexcept override {
        original_->reset();
        std::lock_guard<std::mutex> lock(state_->mutex);
        state_->observation.reset();
        state_->acquired = false;
        state_->diagnostics = {};
    }
private:
    std::unique_ptr<ports::IVisionDetector> original_;
    std::shared_ptr<State> state_;
};

/** @brief Select only the approved pre-acquisition preset; original valid PNG commands pass through. */
class SimulationPreAcquisitionPolicy::SelectingSink final : public runtime::CommandSink {
public:
    SelectingSink(std::unique_ptr<runtime::CommandSink> original, std::shared_ptr<State> state)
        : original_(std::move(original)), state_(std::move(state)) {}
    bool start() override {
        std::lock_guard<std::mutex> lock(state_->mutex);
        state_->active = original_->start();
        return state_->active;
    }
    bool try_submit(const runtime::GuidanceCommand& command) noexcept override {
        try {
            std::lock_guard<std::mutex> lock(state_->mutex);
            if (!state_->active) return false;
            const auto observation = std::exchange(state_->observation, std::nullopt);
            const bool preset = !state_->acquired && observation &&
                observation->healthy_launched_not_found &&
                observation->generation == state_->generation && command.generation == state_->generation &&
                simulation_time(command.source_time) &&
                observation->time.monotonic_ns == command.source_time.monotonic_ns &&
                !command.valid && command.invalid_reason == domain::GuidanceInvalidReason::stale_input &&
                command.overload_g.x == 0 && command.overload_g.y == 0 && command.overload_g.z == 0;
            auto selected = command;
            if (preset) {
                selected.valid = true;
                selected.invalid_reason = domain::GuidanceInvalidReason::none;
                selected.source_frame_id = observation->frame_id;
                selected.overload_g = {0, state_->limit, 0};
            }
            if(state_->audit&&!state_->audit->record_selected(selected,
                    preset?AuditStrategy::preset:(selected.valid?AuditStrategy::png:AuditStrategy::neutral)))
                return false;
            if (!original_->try_submit(selected)) return false;
            if (preset) {
                auto& diagnostics = state_->diagnostics;
                if (diagnostics.preset_submitted++ == 0)
                    diagnostics.first_preset_ns = selected.source_time.monotonic_ns;
                diagnostics.last_preset_ns = selected.source_time.monotonic_ns;
            }
            return true;
        } catch (...) { return false; }
    }
    void set_active_generation(std::uint64_t generation) noexcept override {
        std::lock_guard<std::mutex> lock(state_->mutex);
        state_->generation = generation;
        state_->observation.reset();
        // Generation changes invalidate pending frames, never a real first-detection latch.
        original_->set_active_generation(generation);
    }
    void cancel() noexcept override {
        std::lock_guard<std::mutex> lock(state_->mutex);
        state_->active = false;
        state_->observation.reset();
        original_->cancel();
    }
    void stop() noexcept override {
        std::lock_guard<std::mutex> lock(state_->mutex);
        state_->active = false;
        state_->observation.reset();
        original_->stop();
    }
    Diagnostics diagnostics() const noexcept override { return original_->diagnostics(); }
private:
    std::unique_ptr<runtime::CommandSink> original_;
    std::shared_ptr<State> state_;
};

SimulationPreAcquisitionPolicy::SimulationPreAcquisitionPolicy(double limit, FlightState state,
    std::shared_ptr<SimulationAudit> audit)
    : state_(std::make_shared<State>(limit, std::move(state), std::move(audit))) {
    if (!std::isfinite(limit) || limit <= 0 || !state_->flight_state)
        throw std::invalid_argument("invalid pre-acquisition policy configuration");
}
SimulationPreAcquisitionPolicy::Ports SimulationPreAcquisitionPolicy::wrap(
    std::unique_ptr<ports::IVisionDetector> vision, std::unique_ptr<runtime::CommandSink> sink) {
    std::lock_guard<std::mutex> lock(state_->mutex);
    if (!vision || !sink || state_->wrapped)
        throw std::invalid_argument("pre-acquisition policy requires one original pair of ports");
    state_->wrapped = true;
    return {std::make_unique<ObservingVision>(std::move(vision), state_),
            std::make_unique<SelectingSink>(std::move(sink), state_)};
}
SimulationPreAcquisitionPolicy::Diagnostics SimulationPreAcquisitionPolicy::diagnostics() const noexcept {
    std::lock_guard<std::mutex> lock(state_->mutex);
    return state_->diagnostics;
}
} // namespace mosas::adapters
