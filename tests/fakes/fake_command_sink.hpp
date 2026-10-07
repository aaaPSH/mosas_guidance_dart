#ifndef MOSAS_TEST_FAKES_FAKE_COMMAND_SINK_HPP
#define MOSAS_TEST_FAKES_FAKE_COMMAND_SINK_HPP

#include <mosas/runtime/runtime_interfaces.hpp>

#include <cstdint>
#include <mutex>
#include <vector>

namespace mosas::test {

class FakeCommandSink final : public runtime::CommandSink {
public:
    bool start() override { return true; }

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
};

}  // namespace mosas::test

#endif  // MOSAS_TEST_FAKES_FAKE_COMMAND_SINK_HPP
