#include "fakes/fake_algorithm_ports.hpp"
#include "fakes/fake_imu_source.hpp"
#include "test_support.hpp"

#include "imu_pipeline.hpp"
#include "launch_generation_coordinator.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace {

class GenerationCommandSink final : public mosas::runtime::CommandSink {
public:
    bool start() override { return true; }

    bool try_submit(
        const mosas::runtime::GuidanceCommand& command) noexcept override {
        if (command.generation != active_generation_) {
            return false;
        }
        pending_commands_.push_back(command);
        return true;
    }

    void set_active_generation(std::uint64_t generation) noexcept override {
        active_generation_ = generation;
    }

    std::uint64_t active_generation() const noexcept {
        return active_generation_;
    }

    std::vector<mosas::runtime::GuidanceCommand> drain_pending() {
        std::vector<mosas::runtime::GuidanceCommand> sent;
        for (const auto& command : pending_commands_) {
            if (command.generation == active_generation_) {
                sent.push_back(command);
            }
        }
        pending_commands_.clear();
        return sent;
    }

private:
    std::uint64_t active_generation_ = 0;
    std::vector<mosas::runtime::GuidanceCommand> pending_commands_;
};

}  // namespace

int main() {
    using namespace mosas;
    using namespace std::chrono_literals;

    runtime::LaunchGenerationCoordinator generation_coordinator;
    GenerationCommandSink command_sink;
    generation_coordinator.reset(command_sink);
    CHECK(generation_coordinator.current() == 0);
    const bool initial_command_accepted =
        generation_coordinator.with_generation([&](std::uint64_t generation) {
            runtime::GuidanceCommand command{};
            command.generation = generation;
            return command_sink.try_submit(command);
        });
    CHECK(initial_command_accepted);
    CHECK(generation_coordinator.advance(command_sink) == 1);
    CHECK(command_sink.active_generation() == 1);
    CHECK(command_sink.drain_pending().empty());
    const bool stale_command_accepted =
        generation_coordinator.with_generation([&](std::uint64_t) {
            runtime::GuidanceCommand command{};
            command.generation = 0;
            return command_sink.try_submit(command);
        });
    CHECK(!stale_command_accepted);
    const bool current_command_accepted =
        generation_coordinator.with_generation([&](std::uint64_t generation) {
            runtime::GuidanceCommand command{};
            command.generation = generation;
            return command_sink.try_submit(command);
        });
    CHECK(current_command_accepted);
    const auto transmitted_commands = command_sink.drain_pending();
    CHECK(transmitted_commands.size() == 1);
    CHECK(transmitted_commands[0].generation == 1);

    std::atomic<bool> stop_requested{false};
    std::mutex fault_mutex;
    std::condition_variable fault_condition;
    std::string fault_message;
    auto bad_source = std::make_unique<test::FakeImuSource>(
        std::vector<runtime::ImuAngularVelocitySample>{
            {-1, {0.0, 0.0, 9.80665}, {0.0, 0.0, 0.0}, 0.0, 0.0}});
    auto* bad_source_control = bad_source.get();
    runtime::ImuPipelineCallbacks callbacks;
    callbacks.stop_requested = [&] { return stop_requested.load(); };
    callbacks.try_record_sample = [](const domain::ImuKinematicSample&) {
        return true;
    };
    callbacks.report_fault = [&](std::string message) {
        {
            std::lock_guard<std::mutex> lock(fault_mutex);
            fault_message = std::move(message);
        }
        fault_condition.notify_all();
    };
    runtime::ImuPipeline pipeline(
        std::move(bad_source),
        std::make_unique<test::FakeFlightStateEstimator>(), 8,
        generation_coordinator, callbacks);
    CHECK(pipeline.start_source().status == runtime::SourceStatus::ok);
    pipeline.reset();
    pipeline.start_worker();
    bad_source_control->release();
    {
        std::unique_lock<std::mutex> lock(fault_mutex);
        CHECK(fault_condition.wait_for(lock, 1s,
                                       [&] { return !fault_message.empty(); }));
        CHECK(fault_message ==
              "imu source returned an invalid kinematic sample");
    }
    CHECK(!pipeline.latest_state().has_value());
    CHECK(pipeline.angular_velocity_history_size() == 0);
    pipeline.cancel_source();
    pipeline.join_worker();
    pipeline.stop_source();

    {
        std::lock_guard<std::mutex> lock(fault_mutex);
        fault_message.clear();
    }
    stop_requested.store(false);
    auto recording_failure_source = std::make_unique<test::FakeImuSource>(
        std::vector<runtime::ImuAngularVelocitySample>{
            {1, {0.0, 0.0, 9.80665}, {0.0, 0.0, 0.0}, 0.0, 0.0}});
    auto* recording_failure_source_control = recording_failure_source.get();
    runtime::ImuPipelineCallbacks recording_failure_callbacks;
    recording_failure_callbacks.stop_requested =
        [&] { return stop_requested.load(); };
    recording_failure_callbacks.try_record_sample =
        [](const domain::ImuKinematicSample&) { return false; };
    recording_failure_callbacks.report_fault = [&](std::string message) {
        {
            std::lock_guard<std::mutex> lock(fault_mutex);
            fault_message = std::move(message);
        }
        fault_condition.notify_all();
    };
    runtime::ImuPipeline recording_failure_pipeline(
        std::move(recording_failure_source),
        std::make_unique<test::FakeFlightStateEstimator>(), 8,
        generation_coordinator, std::move(recording_failure_callbacks));
    CHECK(recording_failure_pipeline.start_source().status ==
          runtime::SourceStatus::ok);
    recording_failure_pipeline.reset();
    recording_failure_pipeline.start_worker();
    recording_failure_source_control->release();
    {
        std::unique_lock<std::mutex> lock(fault_mutex);
        CHECK(fault_condition.wait_for(
            lock, 1s, [&] { return !fault_message.empty(); }));
        CHECK(fault_message == "failed to record IMU sample");
    }
    recording_failure_pipeline.cancel_source();
    recording_failure_pipeline.join_worker();
    recording_failure_pipeline.stop_source();

    stop_requested.store(false);
    auto blocked_source = std::make_unique<test::FakeImuSource>(
        std::vector<runtime::ImuAngularVelocitySample>{
            {1, {0.0, 0.0, 9.80665}, {0.0, 0.0, 0.0}, 0.0, 0.0}});
    auto* blocked_source_control = blocked_source.get();
    std::string cancel_fault;
    runtime::ImuPipelineCallbacks blocked_callbacks;
    blocked_callbacks.stop_requested = [&] { return stop_requested.load(); };
    blocked_callbacks.try_record_sample =
        [](const domain::ImuKinematicSample&) { return true; };
    blocked_callbacks.report_fault = [&](std::string message) {
        std::lock_guard<std::mutex> lock(fault_mutex);
        cancel_fault = std::move(message);
        fault_condition.notify_all();
    };
    runtime::ImuPipeline blocked_pipeline(
        std::move(blocked_source),
        std::make_unique<test::FakeFlightStateEstimator>(), 8,
        generation_coordinator, std::move(blocked_callbacks));
    CHECK(blocked_pipeline.start_source().status == runtime::SourceStatus::ok);
    blocked_pipeline.reset();
    blocked_pipeline.start_worker();
    const auto wait_deadline = std::chrono::steady_clock::now() + 1s;
    while (blocked_source_control->wait_count() == 0 &&
           std::chrono::steady_clock::now() < wait_deadline) {
        std::this_thread::yield();
    }
    CHECK(blocked_source_control->wait_count() > 0);
    stop_requested.store(true);
    blocked_pipeline.cancel_source();
    blocked_pipeline.join_worker();
    blocked_pipeline.stop_source();
    CHECK(cancel_fault.empty());
    return 0;
}
