#ifndef MOSAS_RUNTIME_VALUE_HELPERS_HPP
#define MOSAS_RUNTIME_VALUE_HELPERS_HPP

#include <mosas/runtime/runtime_interfaces.hpp>

#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>

namespace mosas::runtime::detail {

inline std::string format_source_error(const char* source_name,
                                       const char* operation,
                                       const SourceResult& result) {
    std::string message = std::string(source_name) + " " + operation + " " +
                          source_status_name(result.status);
    if (!result.message.empty()) {
        message += ": ";
        message += result.message;
    } else if (result.status == SourceStatus::fatal) {
        message += ": source returned fatal without details";
    }
    return message;
}

inline domain::SensorTime make_monotonic_time(TimestampNs timestamp_ns) {
    domain::SensorTime time{};
    time.monotonic_ns = timestamp_ns;
    time.source_domain = domain::ClockDomain::monotonic;
    return time;
}

/** @brief Preserve supplied acquisition metadata, rejecting inconsistent or disguised time.
 * @throws std::invalid_argument On timestamp mismatch or mixed simulation/hardware labels.
 */
inline domain::SensorTime resolve_sensor_time(
    TimestampNs timestamp_ns, const std::optional<domain::SensorTime>& supplied) {
    if (!supplied) return make_monotonic_time(timestamp_ns);
    const auto& time = *supplied;
    if (!time.is_valid() || time.monotonic_ns != timestamp_ns)
        throw std::invalid_argument("sensor timestamp and metadata disagree");
    const bool simulation = time.source == domain::TimestampSource::simulation ||
        time.source_domain == domain::ClockDomain::simulation ||
        time.quality == domain::TimestampQuality::simulation;
    if (simulation && (time.source != domain::TimestampSource::simulation ||
        time.source_domain != domain::ClockDomain::simulation ||
        time.quality != domain::TimestampQuality::simulation ||
        time.source_time_ns != timestamp_ns || time.uncertainty_ns != 0))
        throw std::invalid_argument("simulation time must not claim hardware or host timing");
    return time;
}

/**
 * @brief Build an invalid zero-overload command without clearing status flags.
 * @param source_time Acquisition metadata to retain.
 * @param generation Active launch cycle.
 * @param launched Current launch state, independent of guidance validity.
 * @param target_found Current frame detection result; false on faults or skipped detection.
 * @return Neutral command retaining the supplied metadata and status flags.
 */
inline GuidanceCommand make_neutral_command(const domain::SensorTime& source_time,
                                            std::uint64_t generation,
                                            bool launched, bool target_found) {
    GuidanceCommand command{};
    command.generation = generation;
    command.source_time = source_time;
    command.invalid_reason = domain::GuidanceInvalidReason::stale_input;
    command.launched = launched;
    command.target_found = target_found;
    return command;
}

/** @overload
 * @param timestamp_ns Host monotonic timestamp for a command without acquisition metadata.
 */
inline GuidanceCommand make_neutral_command(TimestampNs timestamp_ns,
                                            std::uint64_t generation,
                                            bool launched, bool target_found) {
    return make_neutral_command(make_monotonic_time(timestamp_ns), generation,
                                launched, target_found);
}

} // namespace mosas::runtime::detail

#endif // MOSAS_RUNTIME_VALUE_HELPERS_HPP
