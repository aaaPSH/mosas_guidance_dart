#ifndef MOSAS_TEST_FAKES_FAKE_COMMAND_SINK_HPP
#define MOSAS_TEST_FAKES_FAKE_COMMAND_SINK_HPP

#include <mosas/runtime/runtime_interfaces.hpp>

#include <atomic>
#include <cstdint>
#include <mutex>
#include <vector>

namespace mosas::test {

class FakeCommandSink final : public runtime::CommandSink {
public:
    bool start() override {
        std::lock_guard<std::mutex> lock(mutex_);
        commands_.clear();
        active_generation_ = 0;
        return true;
    }

    void cancel() noexcept override {
        cancel_count_.fetch_add(1, std::memory_order_relaxed);
    }

    void stop() noexcept override {
        stop_count_.fetch_add(1, std::memory_order_relaxed);
    }

    std::size_t cancel_count() const noexcept {
        return cancel_count_.load(std::memory_order_relaxed);
    }

    std::size_t stop_count() const noexcept {
        return stop_count_.load(std::memory_order_relaxed);
    }

    bool try_submit(const runtime::GuidanceCommand& command) noexcept override {
        std::lock_guard<std::mutex> lock(mutex_);
        if (command.generation != active_generation_) {
            return false;
        }
        commands_.push_back(command);
        return true;
    }

    void set_active_generation(std::uint64_t generation) noexcept override {
        std::lock_guard<std::mutex> lock(mutex_);
        active_generation_ = generation;
    }

    std::vector<runtime::GuidanceCommand> commands() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return commands_;
    }

private:
    mutable std::mutex mutex_;
    std::uint64_t active_generation_ = 0;
    std::vector<runtime::GuidanceCommand> commands_;
    std::atomic<std::size_t> cancel_count_{0};
    std::atomic<std::size_t> stop_count_{0};
};

}  // namespace mosas::test

#endif  // MOSAS_TEST_FAKES_FAKE_COMMAND_SINK_HPP
