#include "output_dispatcher.hpp"

#include "test_support.hpp"

#include <mosas/runtime/output_ports.hpp>
#include <mosas/runtime/command_output.hpp>

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
    std::vector<mosas::runtime::ProcessedFrame> frames;
    std::string error;

    bool consume(const mosas::runtime::ProcessedFrame& frame) {
        std::unique_lock<std::mutex> lock(mutex);
        if (block_first && frame_ids.empty()) {
            entered = true;
            condition.notify_all();
            condition.wait(lock, [this] { return released || cancelled; });
        }
        frame_ids.push_back(frame.frame.source_frame_id);
        frames.push_back(frame);
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

    mosas::runtime::ProcessedFrame frame_at(std::size_t index) const {
        std::lock_guard<std::mutex> lock(mutex);
        return frames.at(index);
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

/** @brief 延后输出只阻塞观测 worker；各 sink 保留同帧首次真实输出，覆盖、超时和换代不借值。 */
int test_delayed_emission() {
    using namespace mosas;
    runtime::CommandOutputProcessor processor;
    auto video = std::make_shared<SinkProbe>();
    auto telemetry = std::make_shared<SinkProbe>();
    auto visualization = std::make_shared<SinkProbe>();
    std::atomic<std::uint64_t> generation{0};
    auto callbacks = make_callbacks();
    callbacks.active_generation = [&] { return generation.load(); };
    runtime::OutputDispatcher dispatcher(
        std::make_unique<FakeVideoRecorder>(video),
        std::make_unique<FakeTelemetryRecorder>(telemetry),
        std::make_unique<FakeVisualizationSink>(visualization), nullptr, 8,
        std::move(callbacks));
    dispatcher.reset();
    dispatcher.start();

    auto frame = make_frame(10);
    frame.frame.timestamp_ns = 100;
    domain::GuidanceCommand request;
    request.source_frame_id = 10;
    request.source_time.monotonic_ns = 100;
    frame.pending_command_emission = processor.track_emission(request, 1s);
    CHECK(dispatcher.enqueue(frame));
    // 在允许等待的区间内，帧不能以尚未发送的快照提前落盘。
    std::this_thread::sleep_for(10ms);
    CHECK(video->snapshot().empty());
    CHECK(telemetry->snapshot().empty());
    CHECK(visualization->snapshot().empty());

    domain::CommandEmissionSnapshot emission;
    emission.attempt_id = 17;
    emission.mapping.command.source_frame_id = 10;
    emission.mapping.command.source_time = request.source_time;
    emission.mapping.command.valid = true;
    emission.mapping.command.overload_g = {0, 0.25, -0.5};
    emission.mapping.trace.attitude.valid = true;
    emission.mapping.trace.attitude.roll_rad.value = 0.3;
    emission.mapping.trace.attitude.pitch_rad.value = -0.2;
    processor.complete(emission, domain::CommandEmissionDisposition::serial_written);
    // 即使 latest 已被下一次重发覆盖，各 sink 仍应取得同一个首次输出。
    emission.attempt_id = 18;
    emission.mapping.command.overload_g.y = 9;
    processor.complete(emission, domain::CommandEmissionDisposition::serial_written);
    for (const auto& probe : {video, telemetry, visualization}) {
        CHECK(probe->wait_for_count(1));
        const auto recorded = probe->frame_at(0);
        CHECK(recorded.command_emission);
        CHECK(recorded.command_emission->attempt_id == 17);
        CHECK(recorded.overlay.body_overload_valid);
        CHECK_NEAR(recorded.overlay.body_overload.y, 0.25, 1e-12);
        CHECK_NEAR(recorded.overlay.output_attitude_roll.value, 0.3, 1e-12);
        CHECK_NEAR(recorded.overlay.output_attitude_pitch.value, -0.2, 1e-12);
        CHECK(!recorded.pending_command_emission);
    }

    // 来源帧 11 被 mailbox 覆盖，12 实际发送；两帧都应完成观测，不错配。
    auto skipped = make_frame(11);
    skipped.frame.timestamp_ns = 100;
    request.source_frame_id = 11;
    skipped.pending_command_emission = processor.track_emission(request, 1s);
    auto sent = make_frame(12);
    sent.frame.timestamp_ns = 100;
    request.source_frame_id = 12;
    sent.pending_command_emission = processor.track_emission(request, 1s);
    CHECK(dispatcher.enqueue(skipped));
    CHECK(dispatcher.enqueue(sent));
    emission.attempt_id = 19;
    emission.mapping.command.source_frame_id = 12;
    processor.complete(emission, domain::CommandEmissionDisposition::serial_written);
    CHECK(telemetry->wait_for_count(3));
    CHECK(!telemetry->frame_at(1).command_emission);
    CHECK(!telemetry->frame_at(1).overlay.body_overload_valid);
    CHECK(telemetry->frame_at(2).command_emission->attempt_id == 19);

    auto expired = make_frame(13);
    request.source_frame_id = 13;
    expired.pending_command_emission = processor.track_emission(request, 5ms);
    CHECK(dispatcher.enqueue(expired));
    CHECK(telemetry->wait_for_count(4));
    CHECK(!telemetry->frame_at(3).command_emission);

    auto stale = make_frame(14);
    request.source_frame_id = 14;
    stale.pending_command_emission = processor.track_emission(request, 10s);
    CHECK(dispatcher.enqueue(stale));
    generation = 1;
    processor.invalidate();
    auto current = make_frame(15);
    current.flight_state.launch.generation = 1;
    CHECK(dispatcher.enqueue(current));
    CHECK(telemetry->wait_for_count(5));
    CHECK(telemetry->frame_at(4).frame.source_frame_id == 15);
    CHECK(!telemetry->frame_at(4).command_emission);

    // 取消必须打断长等待，不能等到凭据期限到达才退出。
    auto waiting = make_frame(16);
    waiting.flight_state.launch.generation = 1;
    request.source_frame_id = 16;
    request.generation = 1;
    waiting.pending_command_emission = processor.track_emission(request, 10s);
    CHECK(dispatcher.enqueue(waiting));
    const auto cancelled_at = std::chrono::steady_clock::now();
    dispatcher.cancel();
    dispatcher.close_queue();
    dispatcher.join();
    CHECK(std::chrono::steady_clock::now() - cancelled_at < 500ms);
    CHECK(dispatcher.stop_sink().empty());
    return 0;
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
    CHECK(test_delayed_emission() == 0);
    return 0;
}
