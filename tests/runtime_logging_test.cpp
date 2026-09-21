#include <condition_variable>
#include <cstring>
#include <atomic>
#include <mutex>
#include <ostream>
#include <sstream>
#include <string>
#include <thread>

#include <cassert>

#define private public
#include <mosas/runtime/guidance_runtime.hpp>
#undef private

namespace {

class QuietImuSource final : public mosas::runtime::ImuSource {
public:
    mosas::runtime::SourceResult start() override {
        return {mosas::runtime::SourceStatus::ok, {}};
    }

    bool wait_for_data() noexcept override {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        return !cancelled_.load();
    }

    mosas::runtime::SourceResult read(
        mosas::runtime::ImuSample*) override {
        return {mosas::runtime::SourceStatus::cancelled, {}};
    }

    void cancel() noexcept override { cancelled_.store(true); }

private:
    std::atomic<bool> cancelled_{false};
};

class QuietCameraSource final : public mosas::runtime::CameraSource {
public:
    mosas::runtime::SourceResult configure(
        const mosas::runtime::CameraCaptureConfig&) override {
        return {mosas::runtime::SourceStatus::ok, {}};
    }

    mosas::runtime::SourceResult capture(
        mosas::runtime::CameraFrame*) override {
        return {mosas::runtime::SourceStatus::timeout, {}};
    }

    void cancel() noexcept override {}
};

class StopFailureFrameSink final : public mosas::runtime::FrameSink {
public:
    bool publish(const mosas::runtime::CameraFrame&,
                 const VisionResult&,
                 const mosas::runtime::ImuStateSnapshot&,
                 const PngGuidanceOutput&) override {
        return true;
    }

    void stop() noexcept override { failed_ = true; }

    std::string last_error() const override {
        return failed_ ? "partial recording write failed" : std::string{};
    }

private:
    bool failed_ = false;
};

class GenerationCommandSink final : public mosas::runtime::CommandSink {
public:
    bool try_submit(const mosas::runtime::GuidanceCommand& command) noexcept override {
        last_submitted_generation = command.launch_generation;
        return command.launch_generation == active_generation;
    }

    void set_active_generation(std::uint64_t generation) noexcept override {
        active_generation = generation;
    }

    std::uint64_t active_generation = 0;
    std::uint64_t last_submitted_generation = 0;
};

class OneShotRearmFrameSink final : public mosas::runtime::FrameSink {
public:
    bool publish(const mosas::runtime::CameraFrame&,
                 const VisionResult&,
                 const mosas::runtime::ImuStateSnapshot&,
                 const PngGuidanceOutput&) override {
        ++published_count;
        rearm_pending = published_count == 1;
        return true;
    }

    bool take_rearm_request() noexcept override {
        const bool pending = rearm_pending;
        rearm_pending = false;
        return pending;
    }

    std::size_t published_count = 0;
    bool rearm_pending = false;
};

class BlockingBuffer final : public std::streambuf {
public:
    bool wait_until_worker_blocked() {
        std::unique_lock<std::mutex> lock(mutex_);
        return condition_.wait_for(lock, std::chrono::seconds(1), [this] {
            return worker_blocked_;
        });
    }

    void release_worker() {
        std::lock_guard<std::mutex> lock(mutex_);
        block_worker_ = false;
        condition_.notify_all();
    }

    bool contains(const std::string& value) const {
        std::lock_guard<std::mutex> lock(mutex_);
        return text_.find(value) != std::string::npos;
    }

protected:
    std::streamsize xsputn(const char* data,
                           std::streamsize size) override {
        if (data == nullptr || size <= 0) {
            return 0;
        }
        std::unique_lock<std::mutex> lock(mutex_);
        maybe_block_worker(lock);
        text_.append(data, static_cast<std::size_t>(size));
        return size;
    }

    int_type overflow(int_type character) override {
        std::unique_lock<std::mutex> lock(mutex_);
        maybe_block_worker(lock);
        if (!traits_type::eq_int_type(character, traits_type::eof())) {
            text_.push_back(traits_type::to_char_type(character));
        }
        return traits_type::not_eof(character);
    }

private:
    void maybe_block_worker(std::unique_lock<std::mutex>& lock) {
        const auto current_thread = std::this_thread::get_id();
        if (worker_id_ == std::thread::id{}) {
            worker_id_ = current_thread;
        }
        if (current_thread != worker_id_ || !block_worker_) {
            return;
        }
        worker_blocked_ = true;
        condition_.notify_all();
        condition_.wait(lock, [this] { return !block_worker_; });
    }

    mutable std::mutex mutex_;
    std::condition_variable condition_;
    std::thread::id worker_id_{};
    bool block_worker_ = true;
    bool worker_blocked_ = false;
    std::string text_;
};

void test_runtime_does_not_write_synchronously_when_log_queue_is_full() {
    BlockingBuffer buffer;
    std::ostream output(&buffer);

    mosas::runtime::GuidanceRuntimeConfig config;
    config.info_log_queue_capacity = 1;
    mosas::runtime::GuidanceRuntime runtime(
        std::unique_ptr<mosas::runtime::ImuSource>{},
        std::unique_ptr<mosas::runtime::CameraSource>{},
        std::unique_ptr<mosas::runtime::CommandSink>{}, config,
        std::unique_ptr<mosas::runtime::FrameSink>{}, output);

    assert(runtime.info_logger_.start());
    mosas::runtime::RuntimeLogEvent first{};
    first.kind = mosas::runtime::RuntimeLogEventKind::text;
    std::strcpy(first.text.data(), "first");
    assert(runtime.info_logger_.try_submit(first));
    assert(buffer.wait_until_worker_blocked());

    mosas::runtime::RuntimeLogEvent queued{};
    queued.kind = mosas::runtime::RuntimeLogEventKind::text;
    std::strcpy(queued.text.data(), "queued");
    assert(runtime.info_logger_.try_submit(queued));

    runtime.log("fallback-marker");
    assert(!buffer.contains("fallback-marker"));

    buffer.release_worker();
    runtime.info_logger_.stop();
}

void test_runtime_reports_frame_sink_stop_failure() {
    std::ostringstream output;
    auto sink = std::make_unique<StopFailureFrameSink>();
    mosas::runtime::GuidanceRuntime runtime(
        std::make_unique<QuietImuSource>(),
        std::make_unique<QuietCameraSource>(),
        std::make_unique<mosas::runtime::CommandSink>(),
        mosas::runtime::GuidanceRuntimeConfig{}, std::move(sink), output);

    assert(runtime.start());
    runtime.stop();
    assert(runtime.faulted());
    assert(runtime.last_error().find("partial recording write failed") !=
           std::string::npos);
}

void test_rearm_advances_generation_and_drops_queued_old_frames() {
    std::ostringstream output;
    auto command_sink = std::make_unique<GenerationCommandSink>();
    auto* command_sink_view = command_sink.get();
    auto frame_sink = std::make_unique<OneShotRearmFrameSink>();
    auto* frame_sink_view = frame_sink.get();
    mosas::runtime::GuidanceRuntimeConfig config;
    config.output_queue_capacity = 4;
    mosas::runtime::GuidanceRuntime runtime(
        std::make_unique<QuietImuSource>(),
        std::make_unique<QuietCameraSource>(), std::move(command_sink), config,
        std::move(frame_sink), output);

    mosas::runtime::GuidanceRuntime::ProcessedFrame old_frame{};
    old_frame.imu_state.launch_generation = 0;
    assert(runtime.output_queue_.push(old_frame));
    assert(runtime.output_queue_.push(old_frame));
    runtime.output_queue_.close();
    runtime.output_worker();

    assert(frame_sink_view->published_count == 1);
    assert(runtime.active_launch_generation_.load() == 1);
    assert(command_sink_view->active_generation == 1);
}

}  // namespace

int main() {
    test_runtime_does_not_write_synchronously_when_log_queue_is_full();
    test_runtime_reports_frame_sink_stop_failure();
    test_rearm_advances_generation_and_drops_queued_old_frames();
    return 0;
}
