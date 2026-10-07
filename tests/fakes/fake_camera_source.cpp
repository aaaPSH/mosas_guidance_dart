#include "fake_camera_source.hpp"

#include <utility>

namespace mosas::test {

FakeCameraSource::FakeCameraSource(std::vector<runtime::CameraFrame> frames)
    : frames_(std::move(frames)) {}

runtime::SourceResult FakeCameraSource::configure(
    const runtime::CameraCaptureConfig&) {
    std::lock_guard<std::mutex> lock(mutex_);
    configured_ = true;
    cancelled_ = false;
    released_ = false;
    next_frame_ = 0;
    return {runtime::SourceStatus::ok, {}};
}

runtime::SourceResult FakeCameraSource::capture(runtime::CameraFrame* frame) {
    if (frame == nullptr) {
        return {runtime::SourceStatus::fatal, "camera frame output is null"};
    }
    std::unique_lock<std::mutex> lock(mutex_);
    condition_.wait(lock, [this] {
        return cancelled_ || (configured_ && released_);
    });
    if (cancelled_) {
        return {runtime::SourceStatus::cancelled, {}};
    }
    if (next_frame_ < frames_.size()) {
        *frame = std::move(frames_[next_frame_++]);
        return {runtime::SourceStatus::ok, {}};
    }
    condition_.wait(lock, [this] { return cancelled_; });
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

}  // namespace mosas::test
