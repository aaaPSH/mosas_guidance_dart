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

class FakeFrameSink final : public mosas::runtime::FrameSink {
public:
    struct Published {
        std::uint64_t frame_id = 0;
        std::uint64_t generation = 0;
        double processing_fps = 0.0;
    };

    explicit FakeFrameSink(bool fail_publish = false,
                           bool block_first_publish = false)
        : fail_publish_(fail_publish),
          block_first_publish_(block_first_publish) {}

    bool publish(const mosas::runtime::CameraFrame& frame,
                 const mosas::domain::VisionDetection&,
                 const mosas::domain::FlightStateSnapshot& state,
                 const mosas::domain::GuidanceOutput&,
                 const mosas::domain::GuidanceOverlay& overlay) override {
        {
            std::unique_lock<std::mutex> lock(mutex_);
            if (block_first_publish_ && published_.empty()) {
                publish_entered_ = true;
                condition_.notify_all();
                condition_.wait(
                    lock, [this] { return cancelled_ || publish_released_; });
            }
            published_.push_back({frame.source_frame_id,
                                  state.launch.generation,
                                  overlay.processing_fps});
            if (request_rearm_once_) {
                request_rearm_once_ = false;
                rearm_pending_ = true;
            }
            if (fail_publish_) {
                last_error_ = "injected publish failure";
            }
        }
        condition_.notify_all();
        return !fail_publish_;
    }

    bool try_record_imu_sample(
        const mosas::domain::ImuKinematicSample&) noexcept override {
        std::lock_guard<std::mutex> lock(mutex_);
        ++imu_sample_count_;
        return true;
    }

    void reset() noexcept override {
        std::lock_guard<std::mutex> lock(mutex_);
        ++reset_count_;
        rearm_pending_ = false;
        cancelled_ = false;
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

    bool take_rearm_request() noexcept override {
        std::lock_guard<std::mutex> lock(mutex_);
        const bool pending = rearm_pending_;
        rearm_pending_ = false;
        return pending;
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

    bool wait_for_publish_entered() {
        std::unique_lock<std::mutex> lock(mutex_);
        return condition_.wait_for(lock, std::chrono::seconds(1),
                                   [this] { return publish_entered_; });
    }

    void release_publish() noexcept {
        std::lock_guard<std::mutex> lock(mutex_);
        publish_released_ = true;
        condition_.notify_all();
    }

    std::vector<Published> published() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return published_;
    }

    std::size_t imu_sample_count() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return imu_sample_count_;
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
    const bool fail_publish_;
    const bool block_first_publish_;
    mutable std::mutex mutex_;
    std::condition_variable condition_;
    std::vector<Published> published_;
    std::string last_error_;
    std::size_t imu_sample_count_ = 0;
    std::size_t reset_count_ = 0;
    std::size_t cancel_count_ = 0;
    std::size_t stop_count_ = 0;
    bool request_rearm_once_ = false;
    bool rearm_pending_ = false;
    bool publish_entered_ = false;
    bool publish_released_ = false;
    bool cancelled_ = false;
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

    auto sink = std::make_unique<FakeFrameSink>();
    auto* sink_control = sink.get();
    sink_control->request_rearm_once();
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

    runtime::OutputDispatcher dispatcher(std::move(sink), 2,
                                         std::move(callbacks));
    dispatcher.reset();
    CHECK(sink_control->reset_count() == 1);
    dispatcher.start();
    CHECK(dispatcher.try_record_imu_sample({}));
    CHECK(sink_control->imu_sample_count() == 1);
    CHECK(dispatcher.enqueue(make_frame(1, 0)));
    CHECK(sink_control->wait_for_published(1));
    const auto rearm_deadline = std::chrono::steady_clock::now() + 1s;
    while (rearm_count.load() == 0 &&
           std::chrono::steady_clock::now() < rearm_deadline) {
        std::this_thread::yield();
    }
    CHECK(rearm_count.load() == 1);
    CHECK(active_generation.load() == 1);

    CHECK(dispatcher.enqueue(make_frame(2, 0)));
    CHECK(dispatcher.enqueue(make_frame(3, 1)));
    CHECK(sink_control->wait_for_published(2));
    auto published = sink_control->published();
    CHECK(published.size() == 2);
    CHECK(published[0].frame_id == 1);
    CHECK(published[0].generation == 0);
    CHECK_NEAR(published[0].processing_fps, 42.0, 1e-12);
    CHECK(published[1].frame_id == 3);
    CHECK(published[1].generation == 1);
    CHECK(published_count.load() == 2);

    dispatcher.cancel();
    dispatcher.close_queue();
    dispatcher.join();
    CHECK(dispatcher.stop_sink().empty());
    CHECK(sink_control->cancel_count() >= 1);
    CHECK(sink_control->stop_count() == 1);
    {
        std::lock_guard<std::mutex> lock(timing_mutex);
        CHECK(timing_stages.size() >= 6);
    }

    std::atomic<std::uint64_t> queued_generation{0};
    auto queued_sink = std::make_unique<FakeFrameSink>(false, true);
    auto* queued_sink_control = queued_sink.get();
    runtime::OutputDispatcherCallbacks queued_callbacks;
    queued_callbacks.stop_requested = [] { return false; };
    queued_callbacks.active_generation = [&] {
        return queued_generation.load();
    };
    queued_callbacks.report_fault = [](std::string) {};
    runtime::OutputDispatcher queued_dispatcher(std::move(queued_sink), 2,
                                                std::move(queued_callbacks));
    queued_dispatcher.reset();
    queued_dispatcher.start();
    CHECK(queued_dispatcher.enqueue(make_frame(10, 0)));
    CHECK(queued_sink_control->wait_for_publish_entered());
    CHECK(queued_dispatcher.enqueue(make_frame(11, 0)));
    queued_generation.store(1);
    queued_sink_control->release_publish();
    CHECK(queued_sink_control->wait_for_published(1));
    CHECK(queued_dispatcher.enqueue(make_frame(12, 1)));
    CHECK(queued_sink_control->wait_for_published(2));
    const auto generation_published = queued_sink_control->published();
    CHECK(generation_published.size() == 2);
    CHECK(generation_published[0].frame_id == 10);
    CHECK(generation_published[1].frame_id == 12);
    queued_dispatcher.close_queue();
    queued_dispatcher.join();
    CHECK(queued_dispatcher.stop_sink().empty());

    std::atomic<bool> fault_reported{false};
    std::string sink_error;
    auto failing_sink = std::make_unique<FakeFrameSink>(true);
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
    runtime::OutputDispatcher failing_dispatcher(std::move(failing_sink), 1,
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
    CHECK(failing_dispatcher.stop_sink() == "injected publish failure");
    return 0;
}
