#include "fake_imu_source.hpp"

#include <utility>

namespace mosas::test {

FakeImuSource::FakeImuSource(
    std::vector<runtime::ImuAngularVelocitySample> samples)
    : samples_(std::move(samples)) {}

runtime::SourceResult FakeImuSource::start() {
    std::lock_guard<std::mutex> lock(mutex_);
    next_sample_ = 0;
    started_ = true;
    released_ = false;
    cancelled_ = false;
    wait_count_ = 0;
    return {runtime::SourceStatus::ok, {}};
}

bool FakeImuSource::try_pop(
    runtime::ImuAngularVelocitySample* sample) noexcept {
    if (sample == nullptr) {
        return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (!started_ || !released_ || cancelled_ ||
        next_sample_ >= samples_.size()) {
        return false;
    }
    *sample = samples_[next_sample_++];
    return true;
}

bool FakeImuSource::wait_for_data() noexcept {
    std::unique_lock<std::mutex> lock(mutex_);
    ++wait_count_;
    condition_.wait(lock, [this] {
        return cancelled_ ||
               (released_ && next_sample_ < samples_.size());
    });
    return started_ && !cancelled_;
}

void FakeImuSource::cancel() noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    cancelled_ = true;
    condition_.notify_all();
}

void FakeImuSource::stop() noexcept { cancel(); }

void FakeImuSource::release() noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    released_ = true;
    condition_.notify_all();
}

std::size_t FakeImuSource::wait_count() const noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    return wait_count_;
}

}  // namespace mosas::test
