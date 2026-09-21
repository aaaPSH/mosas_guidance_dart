#include <mosas/app/nori_camera_diagnostic.hpp>

#include <cassert>
#include <cstdint>
#include <sstream>
#include <string>
#include <vector>

namespace {

class DiagnosticNoriSdk final : public mosas::app::NoriSdkApi {
public:
    PCall_Back_Frame callback = nullptr;
    void* callback_user = nullptr;
    uint32_t selected_device_id = 99;
    uint32_t selected_video_index = 99;
    VIDEO_INFO initialized_video{};
    std::vector<VIDEO_INFO> video_modes{
        VIDEO_INFO{VIDEO_MEDIA_TYPE_MJPG, 640, 480, 30.0F},
        VIDEO_INFO{VIDEO_MEDIA_TYPE_YUYV, 1280, 960, 5.0F},
    };
    bool started = false;

    uint32_t init(uint32_t, uint32_t* device_count) override {
        *device_count = 1;
        return NORI_OK;
    }
    uint32_t uninit() override { return NORI_OK; }
    uint32_t get_device_info(uint32_t device_id, DEVICE_INFO* info) override {
        selected_device_id = device_id;
        info->idVendor = 0x1234;
        info->idProduct = 0x5678;
        return NORI_OK;
    }
    uint32_t get_device_video_info_size(uint32_t, uint32_t* size) override {
        *size = static_cast<uint32_t>(video_modes.size());
        return NORI_OK;
    }
    uint32_t get_device_video_info(uint32_t, uint32_t index,
                                   VIDEO_INFO* info) override {
        selected_video_index = index;
        *info = video_modes.at(index);
        return NORI_OK;
    }
    uint32_t device_video_init(uint32_t device_id, VIDEO_INFO info) override {
        selected_device_id = device_id;
        initialized_video = info;
        return NORI_OK;
    }
    uint32_t device_video_uninit(uint32_t) override { return NORI_OK; }
    uint32_t video_callback(uint32_t device_id, PCall_Back_Frame registered,
                            void* user) override {
        selected_device_id = device_id;
        callback = registered;
        callback_user = user;
        return NORI_OK;
    }
    uint32_t get_trigger_mode(uint32_t, E_TRIGGER_MODE* mode) override {
        *mode = NON_TRIIGER_MODE;
        return NORI_OK;
    }
    uint32_t set_trigger_mode(uint32_t, E_TRIGGER_MODE) override {
        return NORI_OK;
    }
    uint32_t video_start(uint32_t) override {
        started = true;
        uint8_t payload = 0;
        FRAME_BUFFER_DATA frame{};
        frame.PixFormat.u_Format = VIDEO_MEDIA_TYPE_MJPG;
        frame.PixFormat.u_Width = 640;
        frame.PixFormat.u_Height = 480;
        frame.PixFormat.f_Fps = 30.0F;
        frame.pBufAddr = &payload;
        frame.buff_Length = 1;
        assert(callback != nullptr);
        callback(nullptr, &frame, callback_user);
        return NORI_OK;
    }
    uint32_t video_stop(uint32_t) override {
        started = false;
        return NORI_OK;
    }
    uint32_t get_processing_unit_control(
        uint32_t, int32_t, int32_t* current, int32_t* flags, int32_t* step,
        int32_t* minimum, int32_t* maximum, int32_t* default_value) override {
        *current = V4L2_EXPOSURE_AUTO;
        *flags = 0;
        *step = 1;
        *minimum = 0;
        *maximum = 3;
        *default_value = V4L2_EXPOSURE_AUTO;
        return NORI_OK;
    }
    uint32_t set_processing_unit_control(uint32_t, int32_t, int32_t) override {
        return NORI_OK;
    }
    uint32_t set_sensor_shutter(uint32_t, uint32_t) override { return NORI_OK; }
    uint32_t get_sensor_shutter(uint32_t, uint32_t* value) override {
        *value = 1000;
        return NORI_OK;
    }
    uint32_t set_sensor_gain(uint32_t, uint32_t) override { return NORI_OK; }
    uint32_t get_sensor_gain(uint32_t, uint32_t* current, uint32_t* minimum,
                             uint32_t* maximum, uint32_t* step) override {
        *current = 1;
        *minimum = 1;
        *maximum = 16;
        *step = 1;
        return NORI_OK;
    }
    void yuyv_to_bgr24(uint8_t*, uint8_t*, int, int) override {}
};

void test_diagnostic_reports_stream() {
    auto sdk = std::make_shared<DiagnosticNoriSdk>();
    sdk->video_modes = {
        VIDEO_INFO{VIDEO_MEDIA_TYPE_MJPG, 640, 480, 30.0F},
    };
    mosas::app::NoriCameraDiagnosticOptions options;
    options.device_id = 0;
    options.video_index = 0;
    options.duration_ms = 0;

    std::ostringstream output;
    assert(mosas::app::run_nori_camera_diagnostic(sdk, options, output));
    assert(sdk->selected_device_id == 0);
    assert(sdk->selected_video_index == 0);
    assert(!sdk->started);
    assert(output.str().find("device_id=0") != std::string::npos);
    assert(output.str().find("callback_frames=1") != std::string::npos);
}

void test_diagnostic_selects_video_by_parameters() {
    auto sdk = std::make_shared<DiagnosticNoriSdk>();
    mosas::app::NoriCameraDiagnosticOptions options;
    options.device_id = 0;
    options.duration_ms = 0;
    options.select_video_by_parameters = true;
    options.video_format = VIDEO_MEDIA_TYPE_YUYV;
    options.video_width = 1280;
    options.video_height = 960;
    options.video_fps = 5.0F;

    std::ostringstream output;
    assert(mosas::app::run_nori_camera_diagnostic(sdk, options, output));
    assert(sdk->initialized_video.u_Format == VIDEO_MEDIA_TYPE_YUYV);
    assert(sdk->initialized_video.u_Width == 1280);
    assert(sdk->initialized_video.u_Height == 960);
    assert(sdk->initialized_video.f_Fps == 5.0F);
    assert(output.str().find("selected_video_index=1") != std::string::npos);
}

void test_diagnostic_rejects_unmatched_video_parameters() {
    auto sdk = std::make_shared<DiagnosticNoriSdk>();
    mosas::app::NoriCameraDiagnosticOptions options;
    options.device_id = 0;
    options.duration_ms = 0;
    options.select_video_by_parameters = true;
    options.video_format = VIDEO_MEDIA_TYPE_YUYV;
    options.video_width = 640;
    options.video_height = 480;
    options.video_fps = 30.0F;

    std::ostringstream output;
    assert(!mosas::app::run_nori_camera_diagnostic(sdk, options, output));
    assert(!sdk->started);
    assert(output.str().find("video_parameters=unmatched") !=
           std::string::npos);
}

}  // 匿名命名空间结束

int main() {
    test_diagnostic_reports_stream();
    test_diagnostic_selects_video_by_parameters();
    test_diagnostic_rejects_unmatched_video_parameters();
    return 0;
}
