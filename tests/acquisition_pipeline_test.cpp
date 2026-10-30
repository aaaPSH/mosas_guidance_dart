#include "imu_pipeline.hpp"
#include "launch_generation_coordinator.hpp"
#include "test_support.hpp"
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <thread>

namespace {
using namespace mosas;
/** @brief Controlled hardware-like source for synchronization/launch boundary regression tests. */
class Source final : public runtime::ImuSource {
public:
    bool uses_sequence_time() const noexcept override { return true; }
    runtime::SourceResult start() override { return {runtime::SourceStatus::ok, {}}; }
    bool try_pop(runtime::ImuAngularVelocitySample* value) noexcept override {
        std::lock_guard<std::mutex> lock(mutex);
        if (queue.empty()) return false;
        *value = queue.front(); queue.pop_front(); ++popped; return true;
    }
    bool wait_for_data() noexcept override {
        std::unique_lock<std::mutex> lock(mutex);
        ++waits;
        condition.wait(lock, [&] { return stopped || !queue.empty(); });
        return !stopped;
    }
    void cancel() noexcept override { std::lock_guard<std::mutex> lock(mutex); stopped = true; condition.notify_all(); }
    void stop() noexcept override { cancel(); }
    void push(std::int64_t time, std::uint64_t epoch) {
        std::lock_guard<std::mutex> lock(mutex);
        runtime::ImuAngularVelocitySample sample;
        sample.timestamp_ns = time; sample.acquisition_epoch = epoch;
        queue.push_back(sample); condition.notify_all();
    }
    std::atomic<unsigned> popped{0}, waits{0};
private:
    std::mutex mutex;
    std::condition_variable condition;
    std::deque<runtime::ImuAngularVelocitySample> queue;
    bool stopped = false;
};
class Estimator final : public ports::IFlightStateEstimator {
public:
    std::atomic<unsigned> resets{0}, updates{0};
    std::atomic<bool> launch{false};
    domain::FlightStateSnapshot update(const domain::ImuKinematicSample&) override {
        ++updates;
        domain::FlightStateSnapshot result;
        result.launch.phase = launch ? domain::LaunchPhase::launched : domain::LaunchPhase::unknown;
        return result;
    }
    void reset() noexcept override { ++resets; }
};
class Sink final : public runtime::CommandSink {
public:
    bool start() override { return true; }
    bool try_submit(const runtime::GuidanceCommand&) noexcept override { return true; }
};
/** @return Whether the expected worker event occurs within a bounded time. */
template<class Predicate> bool wait(Predicate predicate) {
    const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    while (!predicate() && std::chrono::steady_clock::now() < until) std::this_thread::yield();
    return predicate();
}
}
int main() {
    using namespace mosas;
    auto sync = std::make_shared<domain::AcquisitionSync>(3);
    auto source = std::make_unique<Source>(); auto* input = source.get();
    auto estimator = std::make_unique<Estimator>(); auto* state = estimator.get();
    runtime::LaunchGenerationCoordinator generation;
    Sink sink; generation.reset(sink);
    std::atomic<bool> stop{false}, fault{false};
    std::atomic<unsigned> records{0};
    runtime::ImuPipelineCallbacks callbacks;
    callbacks.stop_requested = [&] { return stop.load(); };
    callbacks.report_fault = [&](std::string) { fault = true; };
    callbacks.try_record_sample = [&](const domain::ImuKinematicSample&) { ++records; return true; };
    callbacks.acquisition_sync = sync;
    callbacks.acquisition_reset = [&] { generation.advance(sink); };
    runtime::ImuPipeline pipeline(std::move(source), std::move(estimator), 8, generation, callbacks);
    CHECK(pipeline.start_source().status == runtime::SourceStatus::ok);
    pipeline.reset(); pipeline.start_worker();
    CHECK(wait([&] { return input->waits >= 1; }));
    input->push(100, 0);
    CHECK(wait([&] { return input->popped == 1 && input->waits >= 2; }));
    CHECK(state->updates == 0 && !pipeline.latest_state());
    sync->ready(1, 0); sync->ready(2, 0);
    input->push(200, 0);
    CHECK(wait([&] { return records == 1; }));
    CHECK(pipeline.latest_state().has_value());
    CHECK(sync->invalidate());
    CHECK(!pipeline.latest_state());
    sync->ready(1, 1); sync->ready(2, 1);
    input->push(300, 0); // Already queued sample from the invalid epoch must never reach the estimator.
    CHECK(wait([&] { return input->popped == 3 && state->resets >= 2; }));
    input->push(400, 1);
    CHECK(wait([&] { return records == 2; }));
    CHECK(pipeline.angular_velocity_history_size() == 1);
    CHECK(generation.current() == 1);
    state->launch = true;
    input->push(500, 1);
    CHECK(wait([&] { return records == 3; }));
    CHECK(sync->state().launched);
    const auto resets = state->resets.load();
    CHECK(!sync->invalidate());
    input->push(600, 1);
    CHECK(wait([&] { return records == 4; }));
    CHECK(state->resets == resets && generation.current() == 1 && !fault);
    stop = true; pipeline.cancel_source(); pipeline.join_worker(); pipeline.stop_source();
    return 0;
}
