#ifndef MOSAS_APP_NORI_CAMERA_SOURCE_HPP
#define MOSAS_APP_NORI_CAMERA_SOURCE_HPP

#include <mosas/app/app_config.hpp>
#include <mosas/app/nori_camera_sdk.hpp>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <opencv2/core/mat.hpp>

namespace mosas::app {

class NoriSdkCameraSource final : public runtime::CameraSource {
public:
    explicit NoriSdkCameraSource(CameraAppConfig config);
    NoriSdkCameraSource(CameraAppConfig config, std::shared_ptr<NoriSdkApi> sdk);
    ~NoriSdkCameraSource() override;

    NoriSdkCameraSource(const NoriSdkCameraSource&) = delete;
    NoriSdkCameraSource& operator=(const NoriSdkCameraSource&) = delete;

    runtime::SourceResult configure(
        const runtime::CameraCaptureConfig& config) override;
    runtime::SourceResult capture(runtime::CameraFrame* frame) override;
    double capture_fps() const noexcept override;
    runtime::SourceResult prepare(runtime::CameraFrame* frame) override;
    void cancel() noexcept override;
    void stop() noexcept override;

private:
    struct CallbackContext {
        mutable std::mutex mutex;
        std::condition_variable condition;
        NoriSdkCameraSource* owner = nullptr;
        bool callbacks_enabled = false;
        uint32_t callbacks_in_flight = 0;
        std::atomic<bool> cancel_requested{true};
    };

    struct PendingFrame {
        std::vector<uint8_t> bytes;
        uint32_t media_type = 0;
        uint32_t width = 0;
        uint32_t height = 0;
        runtime::TimestampNs timestamp_ns = 0;
    };

    class CaptureGuard {
    public:
        explicit CaptureGuard(NoriSdkCameraSource* owner) noexcept
            : owner_(owner) {}
        ~CaptureGuard() noexcept;

        CaptureGuard(const CaptureGuard&) = delete;
        CaptureGuard& operator=(const CaptureGuard&) = delete;

    private:
        NoriSdkCameraSource* owner_;
    };

    static uint32_t frame_callback(PVOID, FRAME_BUFFER_DATA*, PVOID);
    static CallbackContext* create_callback_context();
    static bool enter_callback(CallbackContext* context,
                               NoriSdkCameraSource** owner);
    uint32_t enqueue_frame(FRAME_BUFFER_DATA* frame);
    void leave_callback() noexcept;
    void wait_for_callbacks() noexcept;
    void detach_callback_owner() noexcept;
    bool configure_exposure();
    bool enter_capture() noexcept;
    void leave_capture() noexcept;
    void wait_for_captures() noexcept;
    std::string error_message() const;
    runtime::SourceResult fatal_result() const;
    void set_error(const std::string& error);
    void close_queue();
    void cancel_locked() noexcept;
    void stop_locked() noexcept;

    CameraAppConfig config_;
    cv::Mat undistort_map_x_;
    cv::Mat undistort_map_y_;
    std::shared_ptr<NoriSdkApi> sdk_;
    NoriSdkSession session_;
    uint32_t device_id_ = 0;
    VIDEO_INFO selected_video_{};
    bool session_acquired_ = false;
    bool device_video_initialized_ = false;
    bool video_started_ = false;
    bool configured_ = false;
    std::atomic<double> sdk_capture_fps_{0.0};

    mutable std::mutex lifecycle_mutex_;
    mutable std::mutex capture_mutex_;
    std::condition_variable capture_condition_;
    std::size_t captures_in_flight_ = 0;
    bool capture_stopping_ = true;
    CallbackContext* callback_context_ = nullptr;
    mutable std::mutex queue_mutex_;
    std::condition_variable queue_condition_;
    std::deque<PendingFrame> pending_frames_;
    bool queue_closed_ = true;

    mutable std::mutex error_mutex_;
    std::string last_error_;
};

}  // mosas::app 命名空间结束

#endif  // MOSAS_APP_NORI_CAMERA_SOURCE_HPP 头文件保护结束
