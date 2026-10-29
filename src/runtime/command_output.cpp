#include <mosas/runtime/command_output.hpp>
#include <utility>
namespace mosas::runtime {
bool CommandOutputProcessor::bind(CommandOutputBinding binding) {
    if (!binding.mapper || !binding.mapper->valid() || !binding.latest_state || !binding.now_ns ||
        !binding.output_enabled || binding.maximum_attitude_age_ns <= 0 ||
        (binding.clock_domain != domain::ClockDomain::monotonic && binding.clock_domain != domain::ClockDomain::simulation)) return false;
    binding_ = std::move(binding);
    reset();
    return true;
}
bool CommandOutputProcessor::configured() const noexcept { return binding_.mapper != nullptr; }
void CommandOutputProcessor::reset() noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_.reset(); next_attempt_.store(0, std::memory_order_relaxed);
}
void CommandOutputProcessor::invalidate() noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_.reset();
}
domain::CommandEmissionSnapshot CommandOutputProcessor::prepare(const domain::GuidanceCommand& request,
    std::uint64_t active_generation, bool expired) noexcept {
    domain::CommandEmissionSnapshot result;
    result.attempt_id = next_attempt_.fetch_add(1, std::memory_order_relaxed) + 1;
    domain::CommandOutputContext c;
    c.source_frame_id = request.source_frame_id; c.source_time = request.source_time;
    c.generation = request.generation; c.active_generation = active_generation;
    c.request_valid = request.valid; c.invalid_reason = request.invalid_reason;
    c.launched = request.launched; c.target_found = request.target_found; c.expired = expired;
    c.clock_domain = binding_.clock_domain; c.maximum_attitude_age_ns = binding_.maximum_attitude_age_ns;
    try {
        if (configured()) {
            c.latest_state = binding_.latest_state();
            c.emit_time_ns = binding_.now_ns();
            c.output_enabled = binding_.output_enabled();
            result.mapping = binding_.mapper->map(request.overload_g, c);
            return result;
        }
    } catch (...) {
        // 状态源异常只允许本次零输出，不借旧姿态或旧 body 值继续执行。
    }
    auto& command = result.mapping.command;
    command.source_frame_id = request.source_frame_id; command.source_time = request.source_time;
    command.generation = request.generation;
    command.launched = request.launched && request.generation == active_generation;
    command.target_found = request.target_found && !expired;
    command.invalid_reason = domain::GuidanceInvalidReason::invalid_attitude;
    return result;
}
void CommandOutputProcessor::complete(domain::CommandEmissionSnapshot snapshot,
    domain::CommandEmissionDisposition disposition) noexcept {
    snapshot.disposition = disposition;
    std::lock_guard<std::mutex> lock(mutex_);
    latest_ = std::move(snapshot);
}
std::optional<domain::CommandEmissionSnapshot> CommandOutputProcessor::latest_emission() const noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_;
}
}
