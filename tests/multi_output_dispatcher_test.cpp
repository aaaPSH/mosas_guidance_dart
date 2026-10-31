#include "output_dispatcher.hpp"

#include "test_support.hpp"

#include <mosas/runtime/output_ports.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace {

using namespace std::chrono_literals;

struct SinkProbe {
    bool block_first = false;
    bool fail = false;
    bool entered = false;
    bool released = false;
    bool cancelled = false;
    mutable std::mutex mutex;
    std::condition_variable condition;
    std::vector<std::uint64_t> frame_ids;
    std::string error;

    bool consume(const mosas::runtime::ProcessedFrame& frame) {
        std::unique_lock<std::mutex> lock(mutex);
        if (block_first && frame_ids.empty()) {
            entered = true;
            condition.notify_all();
            condition.wait(lock, [this] { return released || cancelled; });
        }
        frame_ids.push_back(frame.frame.source_frame_id);
        if (fail) {
            error = "injected sink failure";
        }
        condition.notify_all();
        return !fail;
    }

    bool wait_for_entered() {
        std::unique_lock<std::mutex> lock(mutex);
        return condition.wait_for(lock, 1s, [this] { return entered; });
    }

    bool wait_for_count(std::size_t count) {
        std::unique_lock<std::mutex> lock(mutex);
        return condition.wait_for(
            lock, 1s, [this, count] { return frame_ids.size() >= count; });
    }

    std::vector<std::uint64_t> snapshot() const {
        std::lock_guard<std::mutex> lock(mutex);
        return frame_ids;
    }

    void release() {
        std::lock_guard<std::mutex> lock(mutex);
        released = true;
        condition.notify_all();
    }

    void cancel() {
        std::lock_guard<std::mutex> lock(mutex);
        cancelled = true;
        condition.notify_all();
    }

    std::string error_snapshot() const {
        std::lock_guard<std::mutex> lock(mutex);
        return error;
    }
};

class FakeVideoRecorder final : public mosas::runtime::IVideoRecorder {
public:
    explicit FakeVideoRecorder(std::shared_ptr<SinkProbe> probe)
        : probe_(std::move(probe)) {}

    bool start() override { return true; }
    void reset() noexcept override {}
    bool record(const mosas::runtime::ProcessedFrame& frame) override {
        return probe_->consume(frame);
    }
    void cancel() noexcept override { probe_->cancel(); }
    void stop() noexcept override {}
    std::string last_error() const override { return probe_->error_snapshot(); }

private:
    std::shared_ptr<SinkProbe> probe_;
};

class FakeTelemetryRecorder final : public mosas::runtime::ITelemetryRecorder {
public:
    explicit FakeTelemetryRecorder(std::shared_ptr<SinkProbe> probe)
        : probe_(std::move(probe)) {}

    bool start() override { return true; }
    void reset() noexcept override {}
    bool record(const mosas::runtime::ProcessedFrame& frame) override {
        return probe_->consume(frame);
    }
    void cancel() noexcept override { probe_->cancel(); }
    void stop() noexcept override {}
    std::string last_error() const override { return probe_->error_snapshot(); }

private:
    std::shared_ptr<SinkProbe> probe_;
};

class FakeVisualizationSink final
    : public mosas::runtime::IVisualizationSink {
public:
    explicit FakeVisualizationSink(std::shared_ptr<SinkProbe> probe)
        : probe_(std::move(probe)) {}

    bool start() override { return true; }
    void reset() noexcept override {}
    bool publish(const mosas::runtime::ProcessedFrame& frame) override {
        return probe_->consume(frame);
    }
    void cancel() noexcept override { probe_->cancel(); }
    void stop() noexcept override {}
    std::string last_error() const override { return probe_->error_snapshot(); }

private:
    std::shared_ptr<SinkProbe> probe_;
};

mosas::runtime::ProcessedFrame make_frame(std::uint64_t id) {
    mosas::runtime::ProcessedFrame frame{};
    frame.frame.source_frame_id = id;
    frame.flight_state.launch.generation = 0;
    return frame;
}

mosas::runtime::OutputDispatcherCallbacks make_callbacks() {
    mosas::runtime::OutputDispatcherCallbacks callbacks;
    callbacks.stop_requested = [] { return false; };
    callbacks.active_generation = [] { return 0; };
    callbacks.report_fault = [](std::string) {};
    return callbacks;
}

}  // namespace

int main() {
    using namespace mosas;

    auto blocked_video = std::make_shared<SinkProbe>();
    blocked_video->block_first = true;
    auto telemetry = std::make_shared<SinkProbe>();
    auto visualization = std::make_shared<SinkProbe>();
    runtime::OutputDispatcher dispatcher(
        std::make_unique<FakeVideoRecorder>(blocked_video),
        std::make_unique<FakeTelemetryRecorder>(telemetry),
        std::make_unique<FakeVisualizationSink>(visualization), nullptr, 1,
        make_callbacks());
    dispatcher.reset();
    dispatcher.start();

    CHECK(dispatcher.enqueue(make_frame(1)));
    CHECK(blocked_video->wait_for_entered());
    CHECK(telemetry->wait_for_count(1));
    CHECK(visualization->wait_for_count(1));
    CHECK(dispatcher.enqueue(make_frame(2)));
    CHECK(telemetry->wait_for_count(2));
    CHECK(visualization->wait_for_count(2));
    CHECK(dispatcher.enqueue(make_frame(3)));
    CHECK(telemetry->wait_for_count(3));
    CHECK(visualization->wait_for_count(3));

    auto diagnostics = dispatcher.diagnostics();
    CHECK(diagnostics.video.enabled);
    CHECK(diagnostics.video.dropped_count == 1);
    CHECK(diagnostics.video.queue_depth == 1);
    CHECK(diagnostics.video.queue_capacity == 1);
    CHECK(diagnostics.telemetry.dropped_count == 0);
    CHECK(diagnostics.visualization.dropped_count == 0);
    CHECK(telemetry->snapshot() == std::vector<std::uint64_t>({1, 2, 3}));
    CHECK(visualization->snapshot() == std::vector<std::uint64_t>({1, 2, 3}));

    blocked_video->release();
    CHECK(blocked_video->wait_for_count(2));
    CHECK(blocked_video->snapshot() == std::vector<std::uint64_t>({1, 3}));
    dispatcher.close_queue();
    dispatcher.join();
    CHECK(dispatcher.stop_sink().empty());

    auto failing_video = std::make_shared<SinkProbe>();
    failing_video->fail = true;
    auto healthy_telemetry = std::make_shared<SinkProbe>();
    std::atomic<bool> fault_reported{false};
    std::string reported_error;
    auto callbacks = make_callbacks();
    callbacks.report_fault = [&](std::string error) {
        reported_error = std::move(error);
        fault_reported.store(true);
    };
    runtime::OutputDispatcher failing_dispatcher(
        std::make_unique<FakeVideoRecorder>(failing_video),
        std::make_unique<FakeTelemetryRecorder>(healthy_telemetry), nullptr,
        nullptr, 2, std::move(callbacks));
    failing_dispatcher.reset();
    failing_dispatcher.start();
    CHECK(failing_dispatcher.enqueue(make_frame(9)));
    const auto fault_deadline = std::chrono::steady_clock::now() + 1s;
    while (!fault_reported.load() &&
           std::chrono::steady_clock::now() < fault_deadline) {
        std::this_thread::yield();
    }
    CHECK(fault_reported.load());
    CHECK(reported_error == "frame output failed");
    CHECK(healthy_telemetry->wait_for_count(1));
    diagnostics = failing_dispatcher.diagnostics();
    CHECK(!diagnostics.video.healthy);
    CHECK(diagnostics.video.failure_count == 1);
    CHECK(diagnostics.video.last_error == "injected sink failure");
    CHECK(diagnostics.telemetry.healthy);
    CHECK(diagnostics.telemetry.failure_count == 0);
    failing_dispatcher.cancel();
    failing_dispatcher.close_queue();
    failing_dispatcher.join();
    CHECK(failing_dispatcher.stop_sink() == "injected sink failure");
    return 0;
}
