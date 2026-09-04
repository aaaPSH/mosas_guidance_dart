#include <mosas/app/nori_camera_sdk.hpp>

#include <iomanip>
#include <mutex>
#include <sstream>
#include <utility>

namespace mosas::app {
namespace {

std::mutex g_session_mutex;
std::shared_ptr<NoriSdkApi> g_active_sdk;
uint32_t g_device_count = 0;
std::size_t g_session_references = 0;

void set_error(std::string* error, const std::string& message) {
    if (error != nullptr) {
        *error = message;
    }
}

std::string init_error(uint32_t result) {
    std::ostringstream message;
    message << "Nori_Xvision_Init 初始化失败，返回码 0x" << std::uppercase
            << std::hex << result;
    return message.str();
}

std::string uninit_error(uint32_t result) {
    std::ostringstream message;
    message << "Nori_Xvision_UnInit 反初始化失败，返回码 0x"
            << std::uppercase << std::hex << result;
    return message.str();
}

}  // 匿名命名空间结束

uint32_t ProductionNoriSdkApi::init(uint32_t layer, uint32_t* device_count) {
    return Nori_Xvision_Init(layer, device_count);
}

uint32_t ProductionNoriSdkApi::uninit() {
    return Nori_Xvision_UnInit();
}

uint32_t ProductionNoriSdkApi::get_device_info(uint32_t device_id,
                                               DEVICE_INFO* info) {
    return Nori_Xvision_GetDeviceInfo(device_id, info);
}

uint32_t ProductionNoriSdkApi::get_device_video_info_size(uint32_t device_id,
                                                          uint32_t* size) {
    return Nori_Xvision_GetDeviceVideoInfoSize(device_id, size);
}

uint32_t ProductionNoriSdkApi::get_device_video_info(uint32_t device_id,
                                                     uint32_t index,
                                                     VIDEO_INFO* info) {
    return Nori_Xvision_GetDeviceVideoInfo(device_id, index, info);
}

uint32_t ProductionNoriSdkApi::device_video_init(uint32_t device_id,
                                                 VIDEO_INFO info) {
    return Nori_Xvision_DeviceVideoInit(device_id, info);
}

uint32_t ProductionNoriSdkApi::device_video_uninit(uint32_t device_id) {
    return Nori_Xvision_DeviceVideoUnInit(device_id);
}

uint32_t ProductionNoriSdkApi::video_callback(uint32_t device_id,
                                              PCall_Back_Frame callback,
                                              void* user) {
    return Nori_Xvision_VideoCallBack(device_id, callback, user);
}

uint32_t ProductionNoriSdkApi::get_trigger_mode(uint32_t device_id,
                                                E_TRIGGER_MODE* mode) {
    return Nori_Xvision_GetTriggerMode(device_id, mode);
}

uint32_t ProductionNoriSdkApi::set_trigger_mode(uint32_t device_id,
                                                E_TRIGGER_MODE mode) {
    return Nori_Xvision_SetTriggerMode(device_id, mode);
}

uint32_t ProductionNoriSdkApi::video_start(uint32_t device_id) {
    return Nori_Xvision_VideoStart(device_id);
}

uint32_t ProductionNoriSdkApi::video_stop(uint32_t device_id) {
    return Nori_Xvision_VideoStop(device_id);
}

uint32_t ProductionNoriSdkApi::get_processing_unit_control(
    uint32_t device_id, int32_t id, int32_t* current, int32_t* flags,
    int32_t* step, int32_t* minimum, int32_t* maximum,
    int32_t* default_value) {
    return Nori_Xvision_GetProcessingUnitControl(
        device_id, id, current, flags, step, minimum, maximum, default_value);
}

uint32_t ProductionNoriSdkApi::set_processing_unit_control(uint32_t device_id,
                                                           int32_t id,
                                                           int32_t value) {
    return Nori_Xvision_SetProcessingUnitControl(device_id, id, value);
}

uint32_t ProductionNoriSdkApi::set_sensor_shutter(uint32_t device_id,
                                                  uint32_t value) {
    return Nori_Xvision_SetSensorShutter(device_id, value);
}

uint32_t ProductionNoriSdkApi::get_sensor_shutter(uint32_t device_id,
                                                  uint32_t* value) {
    return Nori_Xvision_GetSensorShutter(device_id, value);
}

uint32_t ProductionNoriSdkApi::set_sensor_gain(uint32_t device_id,
                                               uint32_t value) {
    return Nori_Xvision_SetSensorGain(device_id, value);
}

uint32_t ProductionNoriSdkApi::get_sensor_gain(uint32_t device_id,
                                               uint32_t* current,
                                               uint32_t* minimum,
                                               uint32_t* maximum,
                                               uint32_t* step) {
    return Nori_Xvision_GetSensorGain(device_id, current, minimum, maximum,
                                      step);
}

void ProductionNoriSdkApi::yuyv_to_bgr24(uint8_t* input, uint8_t* output,
                                         int width, int height) {
    Nori_Xvision_YuyvToBGR24(input, output, width, height);
}

std::shared_ptr<NoriSdkApi> make_nori_sdk_api() {
    static const std::shared_ptr<NoriSdkApi> sdk =
        std::make_shared<ProductionNoriSdkApi>();
    return sdk;
}

NoriSdkSession::NoriSdkSession(std::shared_ptr<NoriSdkApi> sdk)
    : sdk_(std::move(sdk)) {}

NoriSdkSession::~NoriSdkSession() { (void)release(); }

bool NoriSdkSession::acquire(uint32_t* device_count, std::string* error) {
    std::lock_guard<std::mutex> lock(g_session_mutex);

    if (acquired_) {
        if (device_count != nullptr) {
            *device_count = device_count_;
        }
        set_error(error, "");
        return true;
    }
    if (sdk_ == nullptr) {
        set_error(error, "Nori_Xvision_Init 初始化失败，SDK API 为空");
        return false;
    }
    if (g_session_references != 0 && g_active_sdk.get() != sdk_.get()) {
        set_error(error, "Nori_Xvision_Init 初始化失败，进程中已有其他 SDK 会话");
        return false;
    }

    if (g_session_references == 0) {
        uint32_t initialized_device_count = 0;
        const uint32_t result =
            sdk_->init(NORI_USB_DEVICE, &initialized_device_count);
        if (result != NORI_OK) {
            set_error(error, init_error(result));
            return false;
        }
        g_active_sdk = sdk_;
        g_device_count = initialized_device_count;
    }

    ++g_session_references;
    acquired_ = true;
    device_count_ = g_device_count;
    if (device_count != nullptr) {
        *device_count = device_count_;
    }
    set_error(error, "");
    return true;
}

bool NoriSdkSession::release(std::string* error) {
    std::lock_guard<std::mutex> lock(g_session_mutex);
    if (!acquired_) {
        set_error(error, "");
        return true;
    }

    if (g_session_references > 1) {
        --g_session_references;
        acquired_ = false;
        set_error(error, "");
        return true;
    }

    if (g_active_sdk == nullptr) {
        set_error(error, "Nori_Xvision_UnInit 反初始化失败，SDK API 为空");
        return false;
    }

    const uint32_t result = g_active_sdk->uninit();
    if (result != NORI_OK) {
        set_error(error, uninit_error(result));
        return false;
    }

    acquired_ = false;
    g_session_references = 0;
    g_active_sdk.reset();
    g_device_count = 0;
    set_error(error, "");
    return true;
}

}  // mosas::app 命名空间结束
