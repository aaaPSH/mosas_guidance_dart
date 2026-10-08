#include <mosas/runtime/async_info_logger.hpp>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <ostream>
#include <thread>

namespace mosas::runtime {

AsyncInfoLogger::AsyncInfoLogger(std::ostream& output,
                                 std::size_t queue_capacity)
    : output_(&output), queue_(queue_capacity) {}

AsyncInfoLogger::~AsyncInfoLogger() { stop_and_flush(); }

bool AsyncInfoLogger::start() {
    std::lock_guard<std::mutex> producer_lock(producer_mutex_);
    if (output_ == nullptr || worker_.joinable()) {
        return false;
    }
    queue_.reset();
    accepted_count_.store(0, std::memory_order_relaxed);
    dropped_count_.store(0, std::memory_order_relaxed);
    failure_count_.store(0, std::memory_order_relaxed);
    healthy_.store(true, std::memory_order_release);
    {
        std::lock_guard<std::mutex> lock(error_mutex_);
        last_error_.clear();
    }
    stop_requested_.store(false, std::memory_order_release);
    running_.store(true, std::memory_order_release);
    try {
        worker_ = std::thread(&AsyncInfoLogger::worker_loop, this);
    } catch (...) {
        running_.store(false, std::memory_order_release);
        set_error("failed to start asynchronous logger worker");
        return false;
    }
    return true;
}

bool AsyncInfoLogger::try_submit(const RuntimeLogEvent& event) noexcept {
    if (!running_.load(std::memory_order_acquire) ||
        stop_requested_.load(std::memory_order_acquire)) {
        return false;
    }
    try {
        std::lock_guard<std::mutex> lock(producer_mutex_);
        if (!running_.load(std::memory_order_acquire) ||
            stop_requested_.load(std::memory_order_acquire)) {
            return false;
        }
        if (!queue_.try_push(event)) {
            dropped_count_.fetch_add(1, std::memory_order_relaxed);
            return false;
        }
        accepted_count_.fetch_add(1, std::memory_order_relaxed);
        return true;
    } catch (...) {
        dropped_count_.fetch_add(1, std::memory_order_relaxed);
        return false;
    }
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

SinkDiagnostics AsyncInfoLogger::diagnostics() const noexcept {
    SinkDiagnostics diagnostics;
    diagnostics.enabled = output_ != nullptr;
    diagnostics.healthy = healthy_.load(std::memory_order_acquire);
    diagnostics.accepted_count =
        accepted_count_.load(std::memory_order_relaxed);
    diagnostics.dropped_count = dropped_count_.load(std::memory_order_relaxed);
    diagnostics.failure_count = failure_count_.load(std::memory_order_relaxed);
    diagnostics.queue_depth = queue_.size();
    diagnostics.queue_capacity = queue_.capacity();
    try {
        std::lock_guard<std::mutex> lock(error_mutex_);
        diagnostics.last_error = last_error_;
    } catch (...) {
    }
    return diagnostics;
}

void AsyncInfoLogger::stop_and_flush() noexcept {
    stop_requested_.store(true, std::memory_order_release);
    try {
        std::unique_lock<std::mutex> lock(producer_mutex_);
        lock.unlock();
    } catch (...) {
    }
    if (worker_.joinable()) {
        worker_.join();
    }
    running_.store(false, std::memory_order_release);
}

void AsyncInfoLogger::set_error(const char* message) noexcept {
    if (healthy_.exchange(false, std::memory_order_acq_rel)) {
        failure_count_.fetch_add(1, std::memory_order_relaxed);
    }
    try {
        std::lock_guard<std::mutex> lock(error_mutex_);
        if (last_error_.empty()) {
            last_error_ = message;
        }
    } catch (...) {
    }
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
        try {
            if (event.kind == RuntimeLogEventKind::guidance_command) {
                *output_ << "[异步日志] timestamp_ns=" << event.timestamp_ns
                         << " guidance_command=" << event.value0 << ','
                         << event.value1 << ',' << event.value2 << '\n';
            } else if (event.kind == RuntimeLogEventKind::text) {
                *output_ << event.text.data() << '\n';
            } else {
                *output_ << "[异步诊断] timestamp_ns=" << event.timestamp_ns
                         << " count=" << event.count << " values="
                         << event.value0 << ',' << event.value1 << ','
                         << event.value2 << '\n';
            }
            if (!*output_) {
                set_error("asynchronous logger write failed");
            }
        } catch (...) {
            set_error("asynchronous logger write failed");
        }
    }
    if (output_ != nullptr) {
        try {
            output_->flush();
            if (!*output_) {
                set_error("asynchronous logger flush failed");
            }
        } catch (...) {
            set_error("asynchronous logger flush failed");
        }
    }
}

}  // namespace mosas::runtime
