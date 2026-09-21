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

#define private public
#include <mosas/app/nori_camera_source.hpp>
#undef private

#include <linux/v4l2-controls.h>

#include <algorithm>
#include <cassert>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <sys/time.h>

#include <opencv2/imgcodecs.hpp>

namespace {

class RecordingNoriSdk final : public mosas::app::NoriSdkApi {
public:
    std::vector<std::string> calls;
    std::vector<uint32_t> device_info_ids;
    int32_t exposure_mode = V4L2_EXPOSURE_AUTO;
    PCall_Back_Frame callback = nullptr;
    void* callback_user = nullptr;
    uint32_t init_result = NORI_OK;
    uint32_t video_format = VIDEO_MEDIA_TYPE_MJPG;
    uint32_t video_width = 640;
    uint32_t video_height = 480;
    float video_fps = 30.0F;
    bool block_yuyv_conversion = false;

    uint32_t init(uint32_t, uint32_t* device_count) override {
        calls.emplace_back("init");
        *device_count = 2;
        return init_result;
    }

    uint32_t uninit() override {
        calls.emplace_back("uninit");
        return NORI_OK;
    }

    uint32_t get_device_info(uint32_t device_id, DEVICE_INFO* info) override {
        calls.emplace_back("get_device_info");
        device_info_ids.push_back(device_id);
        info->device[0] = '\0';
        return NORI_OK;
    }

    uint32_t get_device_video_info_size(uint32_t, uint32_t* size) override {
        calls.emplace_back("get_video_info_size");
        *size = 1;
        return NORI_OK;
    }

    uint32_t get_device_video_info(uint32_t, uint32_t, VIDEO_INFO* info) override {
        calls.emplace_back("get_video_info");
        info->u_Format = video_format;
        info->u_Width = video_width;
        info->u_Height = video_height;
        info->f_Fps = video_fps;
        return NORI_OK;
    }

    uint32_t device_video_init(uint32_t, VIDEO_INFO) override {
        calls.emplace_back("video_init");
        return NORI_OK;
    }

    uint32_t device_video_uninit(uint32_t) override {
        calls.emplace_back("video_uninit");
        return NORI_OK;
    }

    uint32_t video_callback(uint32_t, PCall_Back_Frame registered_callback,
                            void* user) override {
        calls.emplace_back("video_callback");
        callback = registered_callback;
        callback_user = user;
        return NORI_OK;
    }

    uint32_t get_trigger_mode(uint32_t, E_TRIGGER_MODE* mode) override {
        calls.emplace_back("get_trigger_mode");
        *mode = NON_TRIIGER_MODE;
        return NORI_OK;
    }

    uint32_t set_trigger_mode(uint32_t, E_TRIGGER_MODE) override {
        calls.emplace_back("set_trigger_mode");
        return NORI_OK;
    }

    uint32_t video_start(uint32_t) override {
        calls.emplace_back("video_start");
        return NORI_OK;
    }

    uint32_t video_stop(uint32_t) override {
        {
            std::lock_guard<std::mutex> lock(conversion_mutex);
            video_stop_called = true;
            if (conversion_active) {
                teardown_during_conversion = true;
            }
        }
        conversion_condition.notify_all();
        calls.emplace_back("video_stop");
        return NORI_OK;
    }

    uint32_t get_processing_unit_control(
        uint32_t, int32_t id, int32_t* current, int32_t*, int32_t*, int32_t*,
        int32_t*, int32_t*) override {
        if (id == V4L2_CID_EXPOSURE_AUTO) {
            calls.emplace_back("get_exposure_mode");
            *current = exposure_mode;
        }
        return NORI_OK;
    }

    uint32_t set_processing_unit_control(uint32_t, int32_t id,
                                         int32_t value) override {
        if (id == V4L2_CID_EXPOSURE_AUTO) {
            calls.emplace_back("set_exposure_mode");
            exposure_mode = value;
        }
        return NORI_OK;
    }

    uint32_t set_sensor_shutter(uint32_t, uint32_t value) override {
        calls.emplace_back("set_shutter_" + std::to_string(value));
        return NORI_OK;
    }

    uint32_t get_sensor_shutter(uint32_t, uint32_t* value) override {
        calls.emplace_back("get_shutter");
        *value = 1000;
        return NORI_OK;
    }

    uint32_t set_sensor_gain(uint32_t, uint32_t value) override {
        calls.emplace_back("set_gain_" + std::to_string(value));
        return NORI_OK;
    }

    uint32_t get_sensor_gain(uint32_t, uint32_t* current, uint32_t* minimum,
                             uint32_t* maximum, uint32_t* step) override {
        calls.emplace_back("get_gain");
        *current = 1;
        *minimum = 1;
        *maximum = 16;
        *step = 1;
        return NORI_OK;
    }

    void yuyv_to_bgr24(uint8_t*, uint8_t*, int, int) override {
        if (!block_yuyv_conversion) {
            return;
        }
        std::unique_lock<std::mutex> lock(conversion_mutex);
        conversion_active = true;
        conversion_started = true;
        conversion_condition.notify_all();
        conversion_condition.wait(lock,
                                  [this] { return allow_conversion; });
        conversion_active = false;
        conversion_condition.notify_all();
    }

    void wait_for_conversion_started() {
        std::unique_lock<std::mutex> lock(conversion_mutex);
        assert(conversion_condition.wait_for(
            lock, std::chrono::milliseconds(500),
            [this] { return conversion_started; }));
    }

    bool wait_for_video_stop(std::chrono::milliseconds timeout) {
        std::unique_lock<std::mutex> lock(conversion_mutex);
        return conversion_condition.wait_for(
            lock, timeout, [this] { return video_stop_called; });
    }

    void release_conversion() {
        {
            std::lock_guard<std::mutex> lock(conversion_mutex);
            allow_conversion = true;
        }
        conversion_condition.notify_all();
    }

    bool teardown_occurred_during_conversion() const {
        std::lock_guard<std::mutex> lock(conversion_mutex);
        return teardown_during_conversion;
    }

private:
    mutable std::mutex conversion_mutex;
    std::condition_variable conversion_condition;
    bool conversion_started = false;
    bool allow_conversion = false;
    bool conversion_active = false;
    bool video_stop_called = false;
    bool teardown_during_conversion = false;
};

std::size_t call_index(const std::vector<std::string>& calls,
                       const std::string& call) {
    const auto iterator = std::find(calls.begin(), calls.end(), call);
    return iterator == calls.end()
               ? calls.size()
               : static_cast<std::size_t>(iterator - calls.begin());
}

void require_call_order(const std::vector<std::string>& calls,
                        const std::string& first,
                        const std::string& second) {
    assert(call_index(calls, first) < call_index(calls, second));
}

void set_current_frame_time(FRAME_BUFFER_DATA* frame) {
    timeval now{};
    assert(gettimeofday(&now, nullptr) == 0);
    frame->Frame_Time.tv_sec = static_cast<int32_t>(now.tv_sec);
    frame->Frame_Time.tv_usec = static_cast<int32_t>(now.tv_usec);
}

void test_callback_queue_capacity_is_configurable() {
    const std::filesystem::path path =
        std::filesystem::temp_directory_path() /
        "mosas_nori_camera_source_test.conf";
    {
        std::ofstream output(path);
        output << "[camera]\ndevice_id=0\ncallback_queue_capacity=3\n"
               << "[imu]\nmeasurement_latency_ns=1234\n"
               << "receive_realtime_priority=42\n"
               << "require_realtime_priority=true\n"
               << "[guidance]\nmax_camera_future_ms=7\n"
               << "max_camera_age_ms=800\n";
    }

    mosas::app::AppConfig config;
    std::string error;
    assert(mosas::app::AppConfigLoader::load(path, &config, &error));
    assert(config.camera.device_id == 0);
    assert(config.camera.callback_queue_capacity == 3);
    assert(config.imu.measurement_latency_ns == 1234);
    assert(config.imu.receive_thread.realtime_priority == 42);
    assert(config.imu.receive_thread.require_realtime_priority);
    assert(config.guidance.max_camera_future_ns == 7'000'000);
    assert(config.guidance.max_camera_age_ns == 800'000'000);
    std::filesystem::remove(path);
}

void test_stop_waits_for_yuyv_conversion() {
    auto sdk = std::make_shared<RecordingNoriSdk>();
    sdk->video_format = VIDEO_MEDIA_TYPE_YUYV;
    sdk->video_width = 2;
    sdk->video_height = 2;
    sdk->video_fps = 30.0F;
    sdk->block_yuyv_conversion = true;

    mosas::app::CameraAppConfig camera_config;
    camera_config.device = "/dev/video0";
    camera_config.width = 2;
    camera_config.height = 2;
    camera_config.fps = 30;
    camera_config.pixel_format = "YUYV";
    camera_config.auto_exposure = true;

    mosas::app::NoriSdkCameraSource source(camera_config, sdk);
    mosas::runtime::CameraCaptureConfig capture_config;
    const auto configure_result = source.configure(capture_config);
    assert(configure_result.status == mosas::runtime::SourceStatus::ok);

    std::vector<uint8_t> yuyv(8, 0);
    FRAME_BUFFER_DATA sdk_frame{};
    sdk_frame.PixFormat.u_Format = VIDEO_MEDIA_TYPE_YUYV;
    sdk_frame.PixFormat.u_Width = 2;
    sdk_frame.PixFormat.u_Height = 2;
    sdk_frame.PixFormat.f_Fps = 30.0F;
    sdk_frame.pBufAddr = yuyv.data();
    sdk_frame.buff_Length = static_cast<uint32_t>(yuyv.size());
    sdk_frame.buff_Offset = 0;
    sdk_frame.buffer.length = static_cast<uint32_t>(yuyv.size());
    set_current_frame_time(&sdk_frame);
    assert(sdk->callback != nullptr);
    assert(sdk->callback(nullptr, &sdk_frame, sdk->callback_user) == NORI_OK);

    mosas::runtime::CameraFrame frame;
    mosas::runtime::SourceResult capture_result;
    std::thread capture_thread([&] {
        capture_result = source.capture(&frame);
    });
    sdk->wait_for_conversion_started();

    source.cancel();
    std::mutex stop_mutex;
    std::condition_variable stop_condition;
    bool stop_started = false;
    std::thread stop_thread([&] {
        {
            std::lock_guard<std::mutex> lock(stop_mutex);
            stop_started = true;
        }
        stop_condition.notify_all();
        source.stop();
    });
    {
        std::unique_lock<std::mutex> lock(stop_mutex);
        assert(stop_condition.wait_for(
            lock, std::chrono::milliseconds(500), [&] { return stop_started; }));
    }

    assert(!sdk->wait_for_video_stop(std::chrono::milliseconds(100)));
    sdk->release_conversion();
    capture_thread.join();
    stop_thread.join();

    assert(capture_result.status == mosas::runtime::SourceStatus::ok);
    assert(!sdk->teardown_occurred_during_conversion());
    require_call_order(sdk->calls, "video_stop", "video_uninit");
    require_call_order(sdk->calls, "video_uninit", "uninit");
}

void test_prepare_rotates_inverted_image() {
    auto sdk = std::make_shared<RecordingNoriSdk>();
    sdk->video_width = 2;
    sdk->video_height = 2;
    sdk->video_fps = 30.0F;

    mosas::app::CameraAppConfig camera_config;
    camera_config.device = "/dev/video0";
    camera_config.width = 2;
    camera_config.height = 2;
    camera_config.fps = 30;
    camera_config.pixel_format = "MJPEG";
    camera_config.auto_exposure = false;

    mosas::app::NoriSdkCameraSource source(camera_config, sdk);
    mosas::runtime::CameraCaptureConfig capture_config;
    const auto configure_result = source.configure(capture_config);
    assert(configure_result.status == mosas::runtime::SourceStatus::ok);

    mosas::runtime::CameraFrame frame;
    frame.image = cv::Mat(2, 2, CV_8UC3);
    frame.image.at<cv::Vec3b>(0, 0) = cv::Vec3b(1, 2, 3);
    frame.image.at<cv::Vec3b>(0, 1) = cv::Vec3b(4, 5, 6);
    frame.image.at<cv::Vec3b>(1, 0) = cv::Vec3b(7, 8, 9);
    frame.image.at<cv::Vec3b>(1, 1) = cv::Vec3b(10, 11, 12);

    const auto prepare_result = source.prepare(&frame);
    assert(prepare_result.status == mosas::runtime::SourceStatus::ok);
    assert(frame.image.at<cv::Vec3b>(0, 0) == cv::Vec3b(10, 11, 12));
    assert(frame.image.at<cv::Vec3b>(0, 1) == cv::Vec3b(7, 8, 9));
    assert(frame.image.at<cv::Vec3b>(1, 0) == cv::Vec3b(4, 5, 6));
    assert(frame.image.at<cv::Vec3b>(1, 1) == cv::Vec3b(1, 2, 3));
    source.stop();
}

}  // 匿名命名空间结束

int main() {
    test_callback_queue_capacity_is_configurable();
    test_stop_waits_for_yuyv_conversion();
    test_prepare_rotates_inverted_image();

    auto sdk = std::make_shared<RecordingNoriSdk>();
    mosas::app::CameraAppConfig camera_config;
    camera_config.device = "/dev/video0";
    camera_config.width = 640;
    camera_config.height = 480;
    camera_config.fps = 30;
    camera_config.pixel_format = "MJPEG";
    camera_config.auto_exposure = false;
    camera_config.exposure_us = 5000.0;
    camera_config.gain = 4.0;

    mosas::app::NoriSdkCameraSource source(camera_config, sdk);
    mosas::runtime::CameraCaptureConfig capture_config;
    const auto configure_result = source.configure(capture_config);
    assert(configure_result.status ==
           mosas::runtime::SourceStatus::ok);
    assert(sdk->device_info_ids.size() == 1);
    assert(sdk->device_info_ids.front() == 0);

    require_call_order(sdk->calls, "get_exposure_mode", "set_exposure_mode");
    require_call_order(sdk->calls, "set_exposure_mode", "get_shutter");
    require_call_order(sdk->calls, "get_shutter", "set_shutter_5000");
    require_call_order(sdk->calls, "set_shutter_5000", "get_gain");
    require_call_order(sdk->calls, "get_gain", "set_gain_4");
    require_call_order(sdk->calls, "video_init", "video_callback");
    require_call_order(sdk->calls, "video_callback", "get_trigger_mode");
    require_call_order(sdk->calls, "get_trigger_mode", "video_start");

    cv::Mat expected(2, 2, CV_8UC3, cv::Scalar(12, 34, 56));
    std::vector<uint8_t> encoded;
    assert(cv::imencode(".jpg", expected, encoded));
    FRAME_BUFFER_DATA sdk_frame{};
    sdk_frame.PixFormat.u_Format = VIDEO_MEDIA_TYPE_MJPG;
    sdk_frame.PixFormat.u_Width = 2;
    sdk_frame.PixFormat.u_Height = 2;
    sdk_frame.PixFormat.f_Fps = 1.0F;
    sdk_frame.pBufAddr = encoded.data();
    sdk_frame.buff_Length = static_cast<uint32_t>(encoded.size());
    sdk_frame.buff_Offset = 0;
    sdk_frame.buffer.length = static_cast<uint32_t>(encoded.size());
    set_current_frame_time(&sdk_frame);
    assert(sdk->callback != nullptr);
    assert(sdk->callback(nullptr, &sdk_frame, sdk->callback_user) == NORI_OK);
    assert(source.capture_fps() == 0.0);
    std::fill(encoded.begin(), encoded.end(), 0);

    mosas::runtime::CameraFrame frame;
    const auto capture_result = source.capture(&frame);
    assert(capture_result.status == mosas::runtime::SourceStatus::ok);
    assert(frame.timestamp_ns >= 0);
    assert(frame.received_timestamp_ns >= 0);
    assert(frame.timestamp_source ==
           mosas::runtime::CameraTimestampSource::sdk_realtime_converted);
    assert(frame.image.size() == expected.size());
    assert(frame.image.type() == CV_8UC3);

    const auto null_result = source.capture(nullptr);
    assert(null_result.status == mosas::runtime::SourceStatus::fatal);

    source.cancel();
    const auto cancelled_result = source.capture(&frame);
    assert(cancelled_result.status ==
           mosas::runtime::SourceStatus::cancelled);
    assert(call_index(sdk->calls, "video_stop") == sdk->calls.size());
    assert(call_index(sdk->calls, "video_uninit") == sdk->calls.size());
    assert(call_index(sdk->calls, "uninit") == sdk->calls.size());
    source.stop();
    require_call_order(sdk->calls, "video_stop", "video_uninit");
    require_call_order(sdk->calls, "video_uninit", "uninit");
    std::cout << "nori_camera_source_test passed\n";
    return 0;
}
