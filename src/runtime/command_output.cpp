#include <mosas/runtime/command_output.hpp>
#include <algorithm>
#include <utility>
namespace mosas::runtime {
bool CommandOutputProcessor::bind(CommandOutputBinding binding) {
    if (!binding.mapper || !binding.mapper->valid() || !binding.latest_state || !binding.now_ns ||
        !binding.output_enabled || binding.maximum_attitude_age_ns <= 0 || binding.maximum_source_age_ns <= 0 ||
        (binding.clock_domain != domain::ClockDomain::monotonic && binding.clock_domain != domain::ClockDomain::simulation)) return false;
    binding_ = std::move(binding);
    reset();
    return true;
}
bool CommandOutputProcessor::configured() const noexcept { return binding_.mapper != nullptr; }
void CommandOutputProcessor::reset() noexcept {
    invalidate();
    next_attempt_.store(0, std::memory_order_relaxed);
}
void CommandOutputProcessor::invalidate() noexcept {
    if (binding_.mapper) binding_.mapper->reset();
    std::lock_guard<std::mutex> lock(mutex_);
    latest_.reset();
    for (const auto& pending : pending_emissions_) {
        if (const auto receipt = pending.lock()) {
            receipt->missing_reason_ = domain::GuidanceInvalidReason::output_invalidated;
            receipt->completion_.set_value(std::nullopt);
        }
    }
    pending_emissions_.clear();
}
domain::CommandEmissionSnapshot CommandOutputProcessor::prepare(const domain::GuidanceCommand& request,
    std::uint64_t active_generation, bool expired, double overload_step_g) noexcept {
    domain::CommandEmissionSnapshot result;
    result.attempt_id = next_attempt_.fetch_add(1, std::memory_order_relaxed) + 1;
    domain::CommandOutputContext c;
    c.source_frame_id = request.source_frame_id; c.source_time = request.source_time;
    c.generation = request.generation; c.active_generation = active_generation;
    c.request_valid = request.valid; c.invalid_reason = request.invalid_reason;
    c.launched = request.launched; c.target_found = request.target_found; c.expired = expired;
    c.clock_domain = binding_.clock_domain; c.maximum_attitude_age_ns = binding_.maximum_attitude_age_ns;
    c.maximum_source_age_ns = binding_.maximum_source_age_ns;
    c.overload_step_g = overload_step_g;
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
    command.invalid_reason = configured() ? domain::GuidanceInvalidReason::state_source_failed
                                          : domain::GuidanceInvalidReason::invalid_configuration;
    return result;
}
std::shared_ptr<CommandEmissionReceipt> CommandOutputProcessor::track_emission(
    const domain::GuidanceCommand& request,
    std::chrono::steady_clock::duration maximum_wait) {
    auto receipt = std::shared_ptr<CommandEmissionReceipt>(new CommandEmissionReceipt(request, maximum_wait));
    std::lock_guard<std::mutex> lock(mutex_);
    pending_emissions_.erase(std::remove_if(pending_emissions_.begin(), pending_emissions_.end(),
        [](const auto& pending) { return pending.expired(); }), pending_emissions_.end());
    pending_emissions_.push_back(receipt);
    return receipt;
}
void CommandOutputProcessor::complete(domain::CommandEmissionSnapshot snapshot,
    domain::CommandEmissionDisposition disposition) noexcept {
    snapshot.disposition = disposition;
    std::lock_guard<std::mutex> lock(mutex_);
    const auto& command = snapshot.mapping.command;
    pending_emissions_.erase(std::remove_if(pending_emissions_.begin(), pending_emissions_.end(),
        [&](const auto& pending) {
            const auto receipt = pending.lock();
            if (!receipt) return true;
            const auto& request = receipt->request_;
            if (request.generation != command.generation ||
                request.source_time.source_domain != command.source_time.source_domain) return false;
            if (request.source_frame_id == command.source_frame_id &&
                request.source_time.monotonic_ns == command.source_time.monotonic_ns) {
                receipt->completion_.set_value(snapshot);
                return true;
            }
            // 来源帧按提交顺序递增；发送更新请求意味着旧请求已被邮箱覆盖。
            if (request.source_frame_id < command.source_frame_id &&
                request.source_time.monotonic_ns <= command.source_time.monotonic_ns) {
                receipt->missing_reason_ = domain::GuidanceInvalidReason::output_replaced;
                receipt->completion_.set_value(std::nullopt);
                return true;
            }
            return false;
        }), pending_emissions_.end());
    latest_ = std::move(snapshot);
}
std::optional<domain::CommandEmissionSnapshot> CommandOutputProcessor::latest_emission() const noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_;
}
}
