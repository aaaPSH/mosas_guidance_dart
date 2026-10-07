#ifndef MOSAS_RUNTIME_LAUNCH_GENERATION_COORDINATOR_HPP
#define MOSAS_RUNTIME_LAUNCH_GENERATION_COORDINATOR_HPP

#include <mosas/runtime/runtime_interfaces.hpp>

#include <atomic>
#include <cstdint>
#include <mutex>
#include <utility>

namespace mosas::runtime {

class LaunchGenerationCoordinator final {
public:
    std::uint64_t current() const noexcept {
        return generation_.load(std::memory_order_acquire);
    }

    void reset(CommandSink& command_sink,
               std::uint64_t generation = 0) {
        std::lock_guard<std::mutex> lock(mutex_);
        command_sink.set_active_generation(generation);
        generation_.store(generation, std::memory_order_release);
    }

    std::uint64_t advance(CommandSink& command_sink) {
        std::lock_guard<std::mutex> lock(mutex_);
        const std::uint64_t next =
            generation_.load(std::memory_order_relaxed) + 1;
        command_sink.set_active_generation(next);
        generation_.store(next, std::memory_order_release);
        return next;
    }

    template <typename Function>
    decltype(auto) with_generation(Function&& function) {
        std::lock_guard<std::mutex> lock(mutex_);
        return std::forward<Function>(function)(
            generation_.load(std::memory_order_relaxed));
    }

private:
    mutable std::mutex mutex_;
    std::atomic<std::uint64_t> generation_{0};
};

}  // namespace mosas::runtime

#endif  // MOSAS_RUNTIME_LAUNCH_GENERATION_COORDINATOR_HPP
