#include <mosas/runtime/async_info_logger.hpp>

#include <chrono>
#include <algorithm>
#include <cstring>
#include <ostream>
#include <thread>

namespace mosas::runtime {

AsyncInfoLogger::AsyncInfoLogger(std::ostream& output,
                                 std::size_t queue_capacity)
    : output_(&output), queue_(queue_capacity) {}

AsyncInfoLogger::~AsyncInfoLogger() { stop(); }

bool AsyncInfoLogger::start() {
    if (output_ == nullptr || worker_.joinable()) {
        return false;
    }
    stop_requested_.store(false, std::memory_order_release);
    running_.store(true, std::memory_order_release);
    try {
        worker_ = std::thread(&AsyncInfoLogger::worker_loop, this);
    } catch (...) {
        running_.store(false, std::memory_order_release);
        return false;
    }
    return true;
}

bool AsyncInfoLogger::try_submit(const RuntimeLogEvent& event) noexcept {
    if (!running_.load(std::memory_order_acquire) ||
        stop_requested_.load(std::memory_order_acquire)) {
        return false;
    }
    if (!queue_.try_push(event)) {
        dropped_count_.fetch_add(1, std::memory_order_relaxed);
        return false;
    }
    return true;
}

bool AsyncInfoLogger::try_submit_text(TimestampNs timestamp_ns,
                                      const std::string& message) noexcept {
    RuntimeLogEvent event{};
    event.kind = RuntimeLogEventKind::text;
    event.timestamp_ns = timestamp_ns;
    const std::size_t copy_size =
        std::min(message.size(), event.text.size() - std::size_t{1});
    if (copy_size != 0) {
        std::memcpy(event.text.data(), message.data(), copy_size);
    }
    event.text[copy_size] = '\0';
    return try_submit(event);
}

void AsyncInfoLogger::stop() noexcept {
    stop_requested_.store(true, std::memory_order_release);
    if (worker_.joinable()) {
        worker_.join();
    }
    running_.store(false, std::memory_order_release);
}

void AsyncInfoLogger::worker_loop() noexcept {
    RuntimeLogEvent event{};
    while (!stop_requested_.load(std::memory_order_acquire) ||
           !queue_.empty()) {
        if (!queue_.try_pop(&event)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        if (output_ == nullptr) {
            continue;
        }
        if (event.kind == RuntimeLogEventKind::guidance_command) {
            *output_ << "[异步日志] timestamp_ns=" << event.timestamp_ns
                     << " guidance_command=" << event.value0 << ','
                     << event.value1 << ',' << event.value2 << '\n';
        } else if (event.kind == RuntimeLogEventKind::text) {
            *output_ << event.text.data() << '\n';
        } else {
            *output_ << "[异步诊断] timestamp_ns=" << event.timestamp_ns
                     << " count=" << event.count << " values=" << event.value0
                     << ',' << event.value1 << ',' << event.value2 << '\n';
        }
    }
    output_->flush();
}

}  // namespace mosas::runtime
