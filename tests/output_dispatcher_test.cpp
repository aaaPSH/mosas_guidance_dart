#include "output_dispatcher.hpp"

#include "test_support.hpp"

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

class FakeVideoRecorder final : public mosas::runtime::IVideoRecorder {
public:
    struct Published {
        std::uint64_t frame_id = 0;
        std::uint64_t generation = 0;
        double processing_fps = 0.0;
    };

    explicit FakeVideoRecorder(bool fail_record = false,
                               bool block_first_record = false)
        : fail_record_(fail_record), block_first_record_(block_first_record) {}

    bool start() override {
        std::lock_guard<std::mutex> lock(mutex_);
        ++start_count_;
        return true;
    }

    void reset() noexcept override {
        std::lock_guard<std::mutex> lock(mutex_);
        ++reset_count_;
        rearm_pending_ = false;
        cancelled_ = false;
    }

    bool record(const mosas::runtime::ProcessedFrame& frame) override {
        std::unique_lock<std::mutex> lock(mutex_);
        if (block_first_record_ && published_.empty()) {
            record_entered_ = true;
            condition_.notify_all();
            condition_.wait(
                lock, [this] { return cancelled_ || record_released_; });
        }
        published_.push_back({frame.frame.source_frame_id,
                               frame.flight_state.launch.generation,
                               frame.overlay.processing_fps});
        if (request_rearm_once_) {
            request_rearm_once_ = false;
            rearm_pending_ = true;
        }
        if (fail_record_) {
            last_error_ = "injected record failure";
        }
        condition_.notify_all();
        return !fail_record_;
    }

    bool take_rearm_request() noexcept override {
        std::lock_guard<std::mutex> lock(mutex_);
        const bool pending = rearm_pending_;
        rearm_pending_ = false;
        return pending;
    }

    void cancel() noexcept override {
        std::lock_guard<std::mutex> lock(mutex_);
        ++cancel_count_;
        cancelled_ = true;
        condition_.notify_all();
    }

    void stop() noexcept override {
        std::lock_guard<std::mutex> lock(mutex_);
        ++stop_count_;
    }

    std::string last_error() const override {
        std::lock_guard<std::mutex> lock(mutex_);
        return last_error_;
    }

    void request_rearm_once() noexcept {
        std::lock_guard<std::mutex> lock(mutex_);
        request_rearm_once_ = true;
    }

    bool wait_for_published(std::size_t count) {
        std::unique_lock<std::mutex> lock(mutex_);
        return condition_.wait_for(lock, std::chrono::seconds(1),
                                   [&] { return published_.size() >= count; });
    }

    bool wait_for_record_entered() {
        std::unique_lock<std::mutex> lock(mutex_);
        return condition_.wait_for(lock, std::chrono::seconds(1),
                                   [this] { return record_entered_; });
    }

    void release_record() noexcept {
        std::lock_guard<std::mutex> lock(mutex_);
        record_released_ = true;
        condition_.notify_all();
    }

    std::vector<Published> published() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return published_;
    }
    std::size_t start_count() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return start_count_;
    }
    std::size_t reset_count() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return reset_count_;
    }
    std::size_t cancel_count() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return cancel_count_;
    }
    std::size_t stop_count() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return stop_count_;
    }

private:
    const bool fail_record_;
    const bool block_first_record_;
    mutable std::mutex mutex_;
    std::condition_variable condition_;
    std::vector<Published> published_;
    std::string last_error_;
    std::size_t start_count_ = 0;
    std::size_t reset_count_ = 0;
    std::size_t cancel_count_ = 0;
    std::size_t stop_count_ = 0;
    bool request_rearm_once_ = false;
    bool rearm_pending_ = false;
    bool record_entered_ = false;
    bool record_released_ = false;
    bool cancelled_ = false;
};

class FakeImuRecorder final : public mosas::runtime::IImuRecorder {
public:
    bool start() override {
        ++start_count_;
        return true;
    }
    bool try_record(
        const mosas::domain::ImuKinematicSample&) noexcept override {
        ++accepted_count_;
        return true;
    }
    mosas::runtime::SinkDiagnostics diagnostics() const noexcept override {
        mosas::runtime::SinkDiagnostics result;
        result.enabled = true;
        result.healthy = true;
        result.accepted_count = accepted_count_.load();
        result.queue_capacity = 8;
        return result;
    }
    void cancel() noexcept override { ++cancel_count_; }
    void stop() noexcept override { ++stop_count_; }

    std::size_t start_count() const noexcept { return start_count_.load(); }
    std::size_t accepted_count() const noexcept {
        return accepted_count_.load();
    }
    std::size_t cancel_count() const noexcept { return cancel_count_.load(); }
    std::size_t stop_count() const noexcept { return stop_count_.load(); }

private:
    std::atomic<std::size_t> start_count_{0};
    std::atomic<std::size_t> accepted_count_{0};
    std::atomic<std::size_t> cancel_count_{0};
    std::atomic<std::size_t> stop_count_{0};
};

mosas::runtime::ProcessedFrame make_frame(std::uint64_t id,
                                          std::uint64_t generation) {
    mosas::runtime::ProcessedFrame frame{};
    frame.frame.source_frame_id = id;
    frame.flight_state.launch.generation = generation;
    return frame;
}

}  // namespace

int main() {
    using namespace mosas;
    using namespace std::chrono_literals;

    std::atomic<bool> stop_requested{false};
    std::atomic<std::uint64_t> active_generation{0};
    std::atomic<std::size_t> rearm_count{0};
    std::atomic<std::size_t> published_count{0};
    std::mutex error_mutex;
    std::condition_variable error_condition;
    std::mutex timing_mutex;
    std::vector<runtime::OutputDispatchTimingStage> timing_stages;

    auto video = std::make_unique<FakeVideoRecorder>();
    auto* video_control = video.get();
    auto imu = std::make_unique<FakeImuRecorder>();
    auto* imu_control = imu.get();
    video_control->request_rearm_once();
    runtime::OutputDispatcherCallbacks callbacks;
    callbacks.stop_requested = [&] { return stop_requested.load(); };
    callbacks.active_generation = [&] { return active_generation.load(); };
    callbacks.processing_fps = [] { return 42.0; };
    callbacks.record_timing = [&](runtime::OutputDispatchTimingStage stage,
                                  std::chrono::steady_clock::duration) {
        std::lock_guard<std::mutex> lock(timing_mutex);
        timing_stages.push_back(stage);
    };
    callbacks.record_published_frame = [&] { ++published_count; };
    callbacks.request_rearm = [&] {
        ++active_generation;
        ++rearm_count;
    };
    callbacks.report_fault = [](std::string) {};

    runtime::OutputDispatcher dispatcher(
        std::move(video), nullptr, nullptr, std::move(imu), 2,
        std::move(callbacks));
    dispatcher.reset();
    CHECK(video_control->reset_count() == 1);
    dispatcher.start();
    CHECK(video_control->start_count() == 1);
    CHECK(imu_control->start_count() == 1);
    CHECK(dispatcher.try_record_imu_sample({}));
    CHECK(imu_control->accepted_count() == 1);
    CHECK(dispatcher.enqueue(make_frame(1, 0)));
    CHECK(video_control->wait_for_published(1));
    const auto rearm_deadline = std::chrono::steady_clock::now() + 1s;
    while (rearm_count.load() == 0 &&
           std::chrono::steady_clock::now() < rearm_deadline) {
        std::this_thread::yield();
    }
    CHECK(rearm_count.load() == 1);
    CHECK(active_generation.load() == 1);

    CHECK(dispatcher.enqueue(make_frame(2, 0)));
    CHECK(dispatcher.enqueue(make_frame(3, 1)));
    CHECK(video_control->wait_for_published(2));
    auto published = video_control->published();
    CHECK(published.size() == 2);
    CHECK(published[0].frame_id == 1);
    CHECK(published[0].generation == 0);
    CHECK_NEAR(published[0].processing_fps, 42.0, 1e-12);
    CHECK(published[1].frame_id == 3);
    CHECK(published[1].generation == 1);
    CHECK(published_count.load() == 2);

    dispatcher.cancel();
    CHECK(imu_control->cancel_count() == 0);
    dispatcher.cancel_imu_recorder();
    dispatcher.close_queue();
    dispatcher.join();
    CHECK(dispatcher.stop_sink().empty());
    CHECK(video_control->cancel_count() >= 1);
    CHECK(video_control->stop_count() == 1);
    CHECK(imu_control->cancel_count() == 1);
    CHECK(imu_control->stop_count() == 1);
    {
        std::lock_guard<std::mutex> lock(timing_mutex);
        CHECK(timing_stages.size() >= 6);
    }

    std::atomic<std::uint64_t> queued_generation{0};
    auto queued_video = std::make_unique<FakeVideoRecorder>(false, true);
    auto* queued_video_control = queued_video.get();
    runtime::OutputDispatcherCallbacks queued_callbacks;
    queued_callbacks.stop_requested = [] { return false; };
    queued_callbacks.active_generation = [&] {
        return queued_generation.load();
    };
    queued_callbacks.report_fault = [](std::string) {};
    runtime::OutputDispatcher queued_dispatcher(
        std::move(queued_video), nullptr, nullptr, nullptr, 2,
        std::move(queued_callbacks));
    queued_dispatcher.reset();
    queued_dispatcher.start();
    CHECK(queued_dispatcher.enqueue(make_frame(10, 0)));
    CHECK(queued_video_control->wait_for_record_entered());
    CHECK(queued_dispatcher.enqueue(make_frame(11, 0)));
    queued_generation.store(1);
    queued_video_control->release_record();
    CHECK(queued_video_control->wait_for_published(1));
    CHECK(queued_dispatcher.enqueue(make_frame(12, 1)));
    CHECK(queued_video_control->wait_for_published(2));
    const auto generation_published = queued_video_control->published();
    CHECK(generation_published.size() == 2);
    CHECK(generation_published[0].frame_id == 10);
    CHECK(generation_published[1].frame_id == 12);
    queued_dispatcher.close_queue();
    queued_dispatcher.join();
    CHECK(queued_dispatcher.stop_sink().empty());

    std::atomic<bool> fault_reported{false};
    std::string sink_error;
    auto failing_video = std::make_unique<FakeVideoRecorder>(true);
    runtime::OutputDispatcherCallbacks failing_callbacks;
    failing_callbacks.stop_requested = [] { return false; };
    failing_callbacks.active_generation = [] { return 0; };
    failing_callbacks.report_fault = [&](std::string message) {
        {
            std::lock_guard<std::mutex> lock(error_mutex);
            sink_error = std::move(message);
        }
        fault_reported.store(true);
        error_condition.notify_all();
    };
    runtime::OutputDispatcher failing_dispatcher(
        std::move(failing_video), nullptr, nullptr, nullptr, 1,
        std::move(failing_callbacks));
    failing_dispatcher.reset();
    failing_dispatcher.start();
    CHECK(failing_dispatcher.enqueue(make_frame(4, 0)));
    {
        std::unique_lock<std::mutex> lock(error_mutex);
        CHECK(error_condition.wait_for(lock, 1s,
                                       [&] { return fault_reported.load(); }));
        CHECK(sink_error == "frame output failed");
    }
    failing_dispatcher.cancel();
    failing_dispatcher.close_queue();
    failing_dispatcher.join();
    CHECK(failing_dispatcher.stop_sink() == "injected record failure");
    CHECK(failing_dispatcher.diagnostics().video.failure_count == 1);
    return 0;
}
