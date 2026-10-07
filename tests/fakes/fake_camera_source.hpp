#ifndef MOSAS_TEST_FAKES_FAKE_CAMERA_SOURCE_HPP
#define MOSAS_TEST_FAKES_FAKE_CAMERA_SOURCE_HPP

#include <mosas/runtime/runtime_interfaces.hpp>

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <mutex>
#include <vector>

namespace mosas::test {

class FakeCameraSource final : public runtime::CameraSource {
public:
    explicit FakeCameraSource(std::vector<runtime::CameraFrame> frames);

    runtime::SourceResult
    configure(const runtime::CameraCaptureConfig& config) override;
    runtime::SourceResult capture(runtime::CameraFrame* frame) override;
    void cancel() noexcept override;
    void release() noexcept;
    bool wait_until_capture_waiting(std::chrono::milliseconds timeout);
    void defer_capture_return_after_cancel() noexcept;
    void release_cancelled_capture() noexcept;

private:
    std::mutex mutex_;
    std::condition_variable condition_;
    std::vector<runtime::CameraFrame> frames_;
    std::size_t next_frame_ = 0;
    bool configured_ = false;
    bool released_ = false;
    bool cancelled_ = false;
    bool capture_waiting_ = false;
    bool defer_cancelled_capture_return_ = false;
    bool cancelled_capture_released_ = false;
};

}  // namespace mosas::test

#endif  // MOSAS_TEST_FAKES_FAKE_CAMERA_SOURCE_HPP
