#include "fake_camera_source.hpp"

#include <utility>

namespace mosas::test {

FakeCameraSource::FakeCameraSource(std::vector<runtime::CameraFrame> frames)
    : frames_(std::move(frames)) {}

runtime::SourceResult
FakeCameraSource::configure(const runtime::CameraCaptureConfig&) {
    std::lock_guard<std::mutex> lock(mutex_);
    configured_ = true;
    cancelled_ = false;
    capture_waiting_ = false;
    released_ = false;
    next_frame_ = 0;
    return {runtime::SourceStatus::ok, {}};
}

runtime::SourceResult FakeCameraSource::capture(runtime::CameraFrame* frame) {
    if (frame == nullptr) {
        return {runtime::SourceStatus::fatal, "camera frame output is null"};
    }
    std::unique_lock<std::mutex> lock(mutex_);
    capture_waiting_ = true;
    condition_.notify_all();
    condition_.wait(
        lock, [this] { return cancelled_ || (configured_ && released_); });
    capture_waiting_ = false;
    if (cancelled_) {
        condition_.wait(lock, [this] {
            return !defer_cancelled_capture_return_ ||
                   cancelled_capture_released_;
        });
        return {runtime::SourceStatus::cancelled, {}};
    }
    if (next_frame_ < frames_.size()) {
        *frame = frames_[next_frame_++];
        return {runtime::SourceStatus::ok, {}};
    }
    capture_waiting_ = true;
    condition_.notify_all();
    condition_.wait(lock, [this] { return cancelled_; });
    condition_.wait(lock, [this] {
        return !defer_cancelled_capture_return_ || cancelled_capture_released_;
    });
    capture_waiting_ = false;
    return {runtime::SourceStatus::cancelled, {}};
}

void FakeCameraSource::cancel() noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    cancelled_ = true;
    condition_.notify_all();
}

void FakeCameraSource::release() noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    released_ = true;
    condition_.notify_all();
}

bool FakeCameraSource::wait_until_capture_waiting(
    std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, timeout, [this] {
        return capture_waiting_ && next_frame_ >= frames_.size();
    });
}

void FakeCameraSource::defer_capture_return_after_cancel() noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    defer_cancelled_capture_return_ = true;
    cancelled_capture_released_ = false;
}

void FakeCameraSource::release_cancelled_capture() noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    cancelled_capture_released_ = true;
    condition_.notify_all();
}

}  // namespace mosas::test
