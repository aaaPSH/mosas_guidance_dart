#include <condition_variable>
#include <cstring>
#include <mutex>
#include <ostream>
#include <string>
#include <thread>

#include <cassert>

#define private public
#include <mosas/runtime/guidance_runtime.hpp>
#undef private

namespace {

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

}  // namespace

int main() {
    test_runtime_does_not_write_synchronously_when_log_queue_is_full();
    return 0;
}
