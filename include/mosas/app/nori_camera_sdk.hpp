#ifndef MOSAS_APP_NORI_CAMERA_SDK_HPP
#define MOSAS_APP_NORI_CAMERA_SDK_HPP

#include <Nori_Xvision_API/Nori_Xvision_API.h>

#include <cstdint>
#include <cstddef>
#include <memory>
#include <string>

namespace mosas::app {

class NoriSdkApi {
public:
    virtual ~NoriSdkApi() = default;

    virtual uint32_t init(uint32_t layer, uint32_t* device_count) = 0;
    virtual uint32_t uninit() = 0;
    virtual uint32_t get_device_info(uint32_t device_id,
                                     DEVICE_INFO* info) = 0;
    virtual uint32_t get_device_video_info_size(uint32_t device_id,
                                                uint32_t* size) = 0;
    virtual uint32_t get_device_video_info(uint32_t device_id,
                                           uint32_t index,
                                           VIDEO_INFO* info) = 0;
    virtual uint32_t device_video_init(uint32_t device_id,
                                       VIDEO_INFO info) = 0;
    virtual uint32_t device_video_uninit(uint32_t device_id) = 0;
    virtual uint32_t video_callback(uint32_t device_id,
                                    PCall_Back_Frame callback,
                                    void* user) = 0;
    virtual uint32_t get_trigger_mode(uint32_t device_id,
                                      E_TRIGGER_MODE* mode) = 0;
    virtual uint32_t set_trigger_mode(uint32_t device_id,
                                      E_TRIGGER_MODE mode) = 0;
    virtual uint32_t video_start(uint32_t device_id) = 0;
    virtual uint32_t video_stop(uint32_t device_id) = 0;
    virtual uint32_t get_processing_unit_control(
        uint32_t device_id, int32_t id, int32_t* current, int32_t* flags,
        int32_t* step, int32_t* minimum, int32_t* maximum,
        int32_t* default_value) = 0;
    virtual uint32_t set_processing_unit_control(uint32_t device_id,
                                                 int32_t id,
                                                 int32_t value) = 0;
    virtual uint32_t set_sensor_shutter(uint32_t device_id,
                                        uint32_t value) = 0;
    virtual uint32_t get_sensor_shutter(uint32_t device_id,
                                        uint32_t* value) = 0;
    virtual uint32_t set_sensor_gain(uint32_t device_id,
                                     uint32_t value) = 0;
    virtual uint32_t get_sensor_gain(uint32_t device_id, uint32_t* current,
                                    uint32_t* minimum, uint32_t* maximum,
                                    uint32_t* step) = 0;
    virtual void yuyv_to_bgr24(uint8_t* yuyv, uint8_t* out_bgr,
                               int width, int height) = 0;
};

class ProductionNoriSdkApi final : public NoriSdkApi {
public:
    uint32_t init(uint32_t layer, uint32_t* device_count) override;
    uint32_t uninit() override;
    uint32_t get_device_info(uint32_t device_id,
                             DEVICE_INFO* info) override;
    uint32_t get_device_video_info_size(uint32_t device_id,
                                        uint32_t* size) override;
    uint32_t get_device_video_info(uint32_t device_id, uint32_t index,
                                   VIDEO_INFO* info) override;
    uint32_t device_video_init(uint32_t device_id, VIDEO_INFO info) override;
    uint32_t device_video_uninit(uint32_t device_id) override;
    uint32_t video_callback(uint32_t device_id, PCall_Back_Frame callback,
                            void* user) override;
    uint32_t get_trigger_mode(uint32_t device_id,
                              E_TRIGGER_MODE* mode) override;
    uint32_t set_trigger_mode(uint32_t device_id,
                              E_TRIGGER_MODE mode) override;
    uint32_t video_start(uint32_t device_id) override;
    uint32_t video_stop(uint32_t device_id) override;
    uint32_t get_processing_unit_control(
        uint32_t device_id, int32_t id, int32_t* current, int32_t* flags,
        int32_t* step, int32_t* minimum, int32_t* maximum,
        int32_t* default_value) override;
    uint32_t set_processing_unit_control(uint32_t device_id, int32_t id,
                                         int32_t value) override;
    uint32_t set_sensor_shutter(uint32_t device_id, uint32_t value) override;
    uint32_t get_sensor_shutter(uint32_t device_id,
                                uint32_t* value) override;
    uint32_t set_sensor_gain(uint32_t device_id, uint32_t value) override;
    uint32_t get_sensor_gain(uint32_t device_id, uint32_t* current,
                            uint32_t* minimum, uint32_t* maximum,
                            uint32_t* step) override;
    void yuyv_to_bgr24(uint8_t* yuyv, uint8_t* out_bgr, int width,
                       int height) override;
};

std::shared_ptr<NoriSdkApi> make_nori_sdk_api();

class NoriSdkSession {
public:
    explicit NoriSdkSession(std::shared_ptr<NoriSdkApi> sdk);
    ~NoriSdkSession();

    NoriSdkSession(const NoriSdkSession&) = delete;
    NoriSdkSession& operator=(const NoriSdkSession&) = delete;

    bool acquire(uint32_t* device_count, std::string* error);
    bool release(std::string* error = nullptr);

private:
    std::shared_ptr<NoriSdkApi> sdk_;
    uint32_t device_count_ = 0;
    bool acquired_ = false;
};

}  // mosas::app 命名空间结束

#endif  // MOSAS_APP_NORI_CAMERA_SDK_HPP 头文件保护结束
