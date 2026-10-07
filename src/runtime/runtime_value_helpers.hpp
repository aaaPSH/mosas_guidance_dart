#ifndef MOSAS_RUNTIME_VALUE_HELPERS_HPP
#define MOSAS_RUNTIME_VALUE_HELPERS_HPP

#include <mosas/runtime/runtime_interfaces.hpp>

#include <cstdint>
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

inline GuidanceCommand make_neutral_command(TimestampNs timestamp_ns,
                                            std::uint64_t generation) {
    GuidanceCommand command{};
    command.generation = generation;
    command.source_time = make_monotonic_time(timestamp_ns);
    command.invalid_reason = domain::GuidanceInvalidReason::stale_input;
    return command;
}

} // namespace mosas::runtime::detail

#endif // MOSAS_RUNTIME_VALUE_HELPERS_HPP
