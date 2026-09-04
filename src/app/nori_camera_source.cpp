#include <mosas/app/nori_camera_source.hpp>

#include <chrono>
#include <cmath>
#include <cstring>
#include <exception>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <sstream>
#include <utility>

#include <linux/v4l2-controls.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace mosas::app {
namespace {

constexpr uint32_t kMaxCallbackFrameBytes = 64U * 1024U * 1024U;

runtime::TimestampNs steady_timestamp_ns() {
    const auto timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
                               std::chrono::steady_clock::now().time_since_epoch())
                               .count();
    return timestamp < 0 ? 0 : timestamp;
}

std::string sdk_error(const char* api, const std::string& device,
                      uint32_t device_id, uint32_t result) {
    std::ostringstream message;
    message << api << " 失败，设备 " << device << " (索引 " << device_id
            << ")，返回码 0x" << std::uppercase << std::hex << result;
    return message.str();
}

bool is_supported_format(uint32_t format) {
    return format == VIDEO_MEDIA_TYPE_MJPG || format == VIDEO_MEDIA_TYPE_YUYV;
}

uint32_t requested_format(const std::string& format) {
    // 配置使用标准名称 MJPEG，SDK 枚举名称为 MJPG。
    if (format == "MJPEG" || format == "MJPG") {
        return VIDEO_MEDIA_TYPE_MJPG;
    }
    if (format == "YUYV") {
        return VIDEO_MEDIA_TYPE_YUYV;
    }
    return 0;
}

bool same_video(const VIDEO_INFO& video, const CameraAppConfig& config,
                uint32_t format) {
    return video.u_Format == format && video.u_Width ==
               static_cast<uint32_t>(config.width) &&
           video.u_Height == static_cast<uint32_t>(config.height) &&
           video.f_Fps == static_cast<float>(config.fps);
}

std::string device_path(const DEVICE_INFO& info) {
    const auto length = strnlen(info.device, sizeof(info.device));
    return std::string(info.device, length);
}

bool same_device_path(const std::string& configured_path,
                      const std::string& discovered_path) {
    if (configured_path == discovered_path) {
        return true;
    }

    std::error_code error;
    return std::filesystem::equivalent(configured_path, discovered_path,
                                       error) &&
           !error;
}

}  // 匿名命名空间结束

NoriSdkCameraSource::NoriSdkCameraSource(CameraAppConfig config)
    : NoriSdkCameraSource(std::move(config), make_nori_sdk_api()) {}

NoriSdkCameraSource::NoriSdkCameraSource(CameraAppConfig config,
                                         std::shared_ptr<NoriSdkApi> sdk)
    : config_(std::move(config)),
      sdk_(std::move(sdk)),
      session_(sdk_),
      callback_context_(create_callback_context()) {
    std::lock_guard<std::mutex> lock(callback_context_->mutex);
    callback_context_->owner = this;
}

NoriSdkCameraSource::~NoriSdkCameraSource() {
    cancel();
    detach_callback_owner();
}

NoriSdkCameraSource::CallbackContext*
NoriSdkCameraSource::create_callback_context() {
    // SDK 没有文档化的回调注销接口，因此 context 由进程级容器永久保留。
    static std::mutex contexts_mutex;
    static std::vector<std::unique_ptr<CallbackContext>> contexts;
    std::lock_guard<std::mutex> lock(contexts_mutex);
    contexts.push_back(std::make_unique<CallbackContext>());
    return contexts.back().get();
}

bool NoriSdkCameraSource::configure(
    const runtime::CameraCaptureConfig& capture_config) {
    std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
    cancel_locked();
    if (session_acquired_ || device_video_initialized_ || video_started_) {
        if (last_error().empty()) {
            set_error("上一次相机停止未完成，设备 " + config_.device);
        }
        return false;
    }
    set_error("");

    config_.capture_mode = capture_config.mode;
    if (config_.device.empty() || config_.width <= 0 || config_.height <= 0 ||
        config_.fps <= 0 || config_.callback_queue_capacity == 0 ||
        (config_.pixel_format != "auto" &&
         requested_format(config_.pixel_format) == 0)) {
        set_error("视频配置无效，设备 " + config_.device);
        return false;
    }

    if (config_.undistort) {
        try {
            const cv::Mat camera_matrix =
                (cv::Mat_<double>(3, 3) << config_.camera_matrix.m00,
                 config_.camera_matrix.m01, config_.camera_matrix.m02,
                 config_.camera_matrix.m10, config_.camera_matrix.m11,
                 config_.camera_matrix.m12, config_.camera_matrix.m20,
                 config_.camera_matrix.m21, config_.camera_matrix.m22);
            const cv::Mat distortion =
                (cv::Mat_<double>(1, 5) << config_.distortion.k1,
                 config_.distortion.k2, config_.distortion.p1,
                 config_.distortion.p2, config_.distortion.k3);
            cv::initUndistortRectifyMap(
                camera_matrix, distortion, cv::Mat(), camera_matrix,
                cv::Size(config_.width, config_.height), CV_32FC1,
                undistort_map_x_, undistort_map_y_);
        } catch (const std::exception& error) {
            set_error("初始化去畸变映射失败：" + std::string(error.what()) +
                      "，设备 " + config_.device);
            undistort_map_x_.release();
            undistort_map_y_.release();
            return false;
        } catch (...) {
            set_error("初始化去畸变映射发生未知异常，设备 " + config_.device);
            undistort_map_x_.release();
            undistort_map_y_.release();
            return false;
        }
    }

    uint32_t device_count = 0;
    std::string session_error;
    if (!session_.acquire(&device_count, &session_error)) {
        set_error(session_error + "，设备 " + config_.device);
        return false;
    }
    session_acquired_ = true;

    bool device_found = false;
    uint32_t fallback_device_id = 0;
    std::vector<std::string> discovered_devices;
    for (uint32_t device_id = 0; device_id < device_count; ++device_id) {
        DEVICE_INFO info{};
        const uint32_t result = sdk_->get_device_info(device_id, &info);
        if (result != NORI_OK) {
            set_error(sdk_error("Nori_Xvision_GetDeviceInfo", config_.device,
                                device_id, result));
            cancel_locked();
            return false;
        }
        const std::string discovered_path = device_path(info);
        if (!discovered_path.empty()) {
            discovered_devices.push_back(discovered_path);
        }
        if (same_device_path(config_.device, discovered_path)) {
            device_id_ = device_id;
            device_found = true;
        }
        if (device_id == 0) {
            fallback_device_id = device_id;
        }
    }
    if (!device_found && device_count == 1) {
        // SDK Sample 直接使用唯一枚举设备的 device_id，不依赖路径文本。
        device_id_ = fallback_device_id;
        device_found = true;
    }
    if (!device_found) {
        std::ostringstream message;
        message << "Nori_Xvision_GetDeviceInfo 未找到匹配设备 "
                << config_.device << "，SDK 枚举数量 " << device_count
                << "，枚举结果 ";
        if (discovered_devices.empty()) {
            message << "为空";
        } else {
            for (std::size_t index = 0; index < discovered_devices.size();
                 ++index) {
                if (index != 0) {
                    message << ", ";
                }
                message << discovered_devices[index];
            }
        }
        message << "，返回码 0x0";
        set_error(message.str());
        cancel_locked();
        return false;
    }

    uint32_t video_info_size = 0;
    uint32_t result = sdk_->get_device_video_info_size(device_id_,
                                                       &video_info_size);
    if (result != NORI_OK) {
        set_error(sdk_error("Nori_Xvision_GetDeviceVideoInfoSize", config_.device,
                            device_id_, result));
        cancel_locked();
        return false;
    }

    const bool auto_format = config_.pixel_format == "auto";
    const uint32_t desired_format = requested_format(config_.pixel_format);
    bool video_found = false;
    for (uint32_t index = 0; index < video_info_size; ++index) {
        VIDEO_INFO video{};
        result = sdk_->get_device_video_info(device_id_, index, &video);
        if (result != NORI_OK) {
            set_error(sdk_error("Nori_Xvision_GetDeviceVideoInfo", config_.device,
                                device_id_, result));
            cancel_locked();
            return false;
        }
        if ((auto_format && is_supported_format(video.u_Format)) ||
            (!auto_format && same_video(video, config_, desired_format))) {
            selected_video_ = video;
            video_found = true;
            break;
        }
    }
    if (!video_found) {
        std::ostringstream message;
        message << "Nori_Xvision_GetDeviceVideoInfo 未找到匹配视频配置，设备 "
                << config_.device << "，请求格式 " << config_.pixel_format
                << "、分辨率 " << config_.width << "x" << config_.height
                << "、帧率 " << config_.fps << "，返回码 0x0";
        set_error(message.str());
        cancel_locked();
        return false;
    }

    result = sdk_->device_video_init(device_id_, selected_video_);
    if (result != NORI_OK) {
        set_error(sdk_error("Nori_Xvision_DeviceVideoInit", config_.device,
                            device_id_, result));
        cancel_locked();
        return false;
    }
    device_video_initialized_ = true;

    result = sdk_->video_callback(device_id_, &NoriSdkCameraSource::frame_callback,
                                  callback_context_);
    if (result != NORI_OK) {
        set_error(sdk_error("Nori_Xvision_VideoCallBack", config_.device,
                            device_id_, result));
        cancel_locked();
        return false;
    }

    const E_TRIGGER_MODE desired_trigger_mode =
        capture_config.mode == runtime::CameraCaptureMode::hardware_trigger
            ? HARDWARE_TRIGGER_MODE
            : NON_TRIIGER_MODE;
    E_TRIGGER_MODE trigger_mode = NON_TRIIGER_MODE;
    result = sdk_->get_trigger_mode(device_id_, &trigger_mode);
    if (result != NORI_OK) {
        set_error(sdk_error("Nori_Xvision_GetTriggerMode", config_.device,
                            device_id_, result));
        cancel_locked();
        return false;
    }
    if (trigger_mode != desired_trigger_mode) {
        result = sdk_->set_trigger_mode(device_id_, desired_trigger_mode);
    }
    if (result != NORI_OK) {
        set_error(sdk_error("Nori_Xvision_SetTriggerMode", config_.device,
                            device_id_, result));
        cancel_locked();
        return false;
    }

    {
        std::lock_guard<std::mutex> queue_lock(queue_mutex_);
        pending_frames_.clear();
        queue_closed_ = false;
    }
    {
        std::lock_guard<std::mutex> callback_lock(callback_context_->mutex);
        callback_context_->cancel_requested.store(false,
                                                   std::memory_order_release);
        callback_context_->callbacks_enabled = true;
    }
    result = sdk_->video_start(device_id_);
    if (result != NORI_OK) {
        set_error(sdk_error("Nori_Xvision_VideoStart", config_.device,
                            device_id_, result));
        cancel_locked();
        return false;
    }
    video_started_ = true;

    if (!configure_exposure()) {
        cancel_locked();
        return false;
    }

    configured_ = true;
    return true;
}

bool NoriSdkCameraSource::configure_exposure() {
    int32_t current_mode = 0;
    int32_t flags = 0;
    int32_t step = 0;
    int32_t minimum = 0;
    int32_t maximum = 0;
    int32_t default_value = 0;
    uint32_t result = sdk_->get_processing_unit_control(
        device_id_, V4L2_CID_EXPOSURE_AUTO, &current_mode, &flags, &step,
        &minimum, &maximum, &default_value);
    if (result != NORI_OK) {
        set_error(sdk_error("Nori_Xvision_GetProcessingUnitControl",
                            config_.device, device_id_, result));
        return false;
    }

    const int32_t desired_mode =
        config_.auto_exposure ? V4L2_EXPOSURE_AUTO : V4L2_EXPOSURE_MANUAL;
    if (current_mode != desired_mode) {
        result = sdk_->set_processing_unit_control(
            device_id_, V4L2_CID_EXPOSURE_AUTO, desired_mode);
        if (result != NORI_OK) {
            set_error(sdk_error("Nori_Xvision_SetProcessingUnitControl",
                                config_.device, device_id_, result));
            return false;
        }
    }
    if (config_.auto_exposure) {
        return true;
    }

    if (!std::isfinite(config_.exposure_us) || config_.exposure_us <= 0.0 ||
        config_.exposure_us >
            static_cast<double>(std::numeric_limits<uint32_t>::max())) {
        set_error("Nori_Xvision_SetSensorShutter 参数无效，设备 " +
                  config_.device + "，返回码 0x0");
        return false;
    }
    uint32_t current_shutter = 0;
    result = sdk_->get_sensor_shutter(device_id_, &current_shutter);
    if (result != NORI_OK) {
        set_error(sdk_error("Nori_Xvision_GetSensorShutter", config_.device,
                            device_id_, result));
        return false;
    }
    result = sdk_->set_sensor_shutter(
        device_id_, static_cast<uint32_t>(config_.exposure_us));
    if (result != NORI_OK) {
        set_error(sdk_error("Nori_Xvision_SetSensorShutter", config_.device,
                            device_id_, result));
        return false;
    }
    if (config_.gain > 0.0) {
        if (!std::isfinite(config_.gain) ||
            config_.gain >
                static_cast<double>(std::numeric_limits<uint32_t>::max())) {
            set_error("Nori_Xvision_SetSensorGain 参数无效，设备 " +
                      config_.device + "，返回码 0x0");
            return false;
        }
        uint32_t current_gain = 0;
        uint32_t minimum_gain = 0;
        uint32_t maximum_gain = 0;
        uint32_t gain_step = 0;
        result = sdk_->get_sensor_gain(device_id_, &current_gain,
                                       &minimum_gain, &maximum_gain,
                                       &gain_step);
        if (result != NORI_OK) {
            set_error(sdk_error("Nori_Xvision_GetSensorGain", config_.device,
                                device_id_, result));
            return false;
        }
        result = sdk_->set_sensor_gain(device_id_,
                                       static_cast<uint32_t>(config_.gain));
        if (result != NORI_OK) {
            set_error(sdk_error("Nori_Xvision_SetSensorGain", config_.device,
                                device_id_, result));
            return false;
        }
    }
    return true;
}

bool NoriSdkCameraSource::capture(runtime::CameraFrame* frame) {
    if (frame == nullptr) {
        set_error("capture 参数为空，设备 " + config_.device);
        return false;
    }

    frame->image.release();
    frame->timestamp_ns = 0;

    PendingFrame pending;
    {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        queue_condition_.wait(lock, [this] {
            return queue_closed_ || !pending_frames_.empty();
        });
        if (queue_closed_ || pending_frames_.empty()) {
            return false;
        }
        pending = std::move(pending_frames_.front());
        pending_frames_.pop_front();
    }

    const auto max_int = static_cast<uint32_t>(std::numeric_limits<int>::max());
    if (pending.bytes.empty()) {
        set_error("capture 收到空帧，设备 " + config_.device);
        return false;
    }
    if (pending.width == 0 || pending.height == 0 || pending.width > max_int ||
        pending.height > max_int) {
        set_error("capture 收到无效帧尺寸 " + std::to_string(pending.width) +
                  "x" + std::to_string(pending.height) + "，设备 " +
                  config_.device);
        return false;
    }

    try {
        if (pending.media_type == VIDEO_MEDIA_TYPE_MJPG) {
            if (pending.bytes.size() >
                static_cast<std::size_t>(std::numeric_limits<int>::max())) {
                set_error("capture 收到过大的 MJPG 帧，设备 " + config_.device);
                return false;
            }
            const cv::Mat encoded(1, static_cast<int>(pending.bytes.size()),
                                  CV_8UC1, pending.bytes.data());
            cv::Mat decoded = cv::imdecode(encoded, cv::IMREAD_COLOR);
            if (decoded.empty() || decoded.type() != CV_8UC3 ||
                decoded.cols != static_cast<int>(pending.width) ||
                decoded.rows != static_cast<int>(pending.height)) {
                set_error("capture 解码 MJPG 帧失败或尺寸不匹配，设备 " +
                          config_.device);
                return false;
            }
            frame->image = std::move(decoded);
        } else if (pending.media_type == VIDEO_MEDIA_TYPE_YUYV) {
            const uint64_t pixel_count = static_cast<uint64_t>(pending.width) *
                                          static_cast<uint64_t>(pending.height);
            if (pixel_count > std::numeric_limits<uint64_t>::max() / 2U) {
                set_error("capture YUYV 输入尺寸溢出，设备 " + config_.device);
                return false;
            }
            const uint64_t required_bytes = pixel_count * 2U;
            if (required_bytes > std::numeric_limits<std::size_t>::max() ||
                pending.bytes.size() < static_cast<std::size_t>(required_bytes)) {
                set_error("capture YUYV 帧长度不足，设备 " + config_.device);
                return false;
            }
            if (pixel_count >
                std::numeric_limits<std::size_t>::max() / sizeof(uint8_t) / 3U) {
                set_error("capture YUYV 输出尺寸溢出，设备 " + config_.device);
                return false;
            }
            cv::Mat bgr(static_cast<int>(pending.height),
                        static_cast<int>(pending.width), CV_8UC3);
            sdk_->yuyv_to_bgr24(pending.bytes.data(), bgr.data,
                                static_cast<int>(pending.width),
                                static_cast<int>(pending.height));
            if (bgr.empty() || bgr.type() != CV_8UC3) {
                set_error("capture YUYV 转 BGR 失败，设备 " + config_.device);
                return false;
            }
            frame->image = std::move(bgr);
        } else {
            set_error("capture 收到不支持的视频格式，设备 " + config_.device);
            return false;
        }
    } catch (const std::exception& error) {
        set_error("capture 图像转换异常：" + std::string(error.what()) +
                  "，设备 " + config_.device);
        return false;
    } catch (...) {
        set_error("capture 图像转换发生未知异常，设备 " + config_.device);
        return false;
    }

    frame->timestamp_ns = pending.timestamp_ns;
    return !frame->image.empty() && frame->image.type() == CV_8UC3;
}

bool NoriSdkCameraSource::prepare(runtime::CameraFrame* frame) {
    if (frame == nullptr || frame->image.empty() ||
        frame->image.type() != CV_8UC3) {
        return false;
    }
    std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
    try {
        if (frame->image.cols != config_.width ||
            frame->image.rows != config_.height) {
            cv::resize(frame->image, frame->image,
                       cv::Size(config_.width, config_.height));
        }
        if (config_.undistort) {
            if (undistort_map_x_.empty() || undistort_map_y_.empty() ||
                undistort_map_x_.size() != cv::Size(config_.width, config_.height) ||
                undistort_map_y_.size() != cv::Size(config_.width, config_.height)) {
                set_error("去畸变映射未按目标尺寸配置，设备 " + config_.device);
                return false;
            }
            cv::Mat undistorted_frame;
            cv::remap(frame->image, undistorted_frame, undistort_map_x_,
                      undistort_map_y_, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
            frame->image = std::move(undistorted_frame);
        }
    } catch (const std::exception& error) {
        set_error("相机帧预处理失败：" + std::string(error.what()) +
                  "，设备 " + config_.device);
        return false;
    } catch (...) {
        set_error("相机帧预处理发生未知异常，设备 " + config_.device);
        return false;
    }
    return !frame->image.empty() && frame->image.type() == CV_8UC3;
}

void NoriSdkCameraSource::cancel() noexcept {
    std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
    cancel_locked();
}

void NoriSdkCameraSource::cancel_locked() noexcept {
    undistort_map_x_.release();
    undistort_map_y_.release();

    {
        std::lock_guard<std::mutex> callback_lock(callback_context_->mutex);
        callback_context_->callbacks_enabled = false;
        callback_context_->cancel_requested.store(true,
                                                   std::memory_order_release);
    }
    configured_ = false;

    if (video_started_) {
        const uint32_t result = sdk_->video_stop(device_id_);
        if (result != NORI_OK) {
            set_error(sdk_error("Nori_Xvision_VideoStop", config_.device,
                                device_id_, result));
            wait_for_callbacks();
            close_queue();
            return;
        }
        video_started_ = false;
    }

    // SDK 当前没有文档化的注销回调接口；VideoStop 返回后 SDK 不再发起新回调，
    // 此等待屏障覆盖 VideoStop 返回前已经通过准入检查的回调。
    wait_for_callbacks();
    close_queue();

    if (device_video_initialized_) {
        const uint32_t result = sdk_->device_video_uninit(device_id_);
        if (result != NORI_OK) {
            set_error(sdk_error("Nori_Xvision_DeviceVideoUnInit", config_.device,
                                device_id_, result));
            return;
        }
        device_video_initialized_ = false;
    }

    if (session_acquired_) {
        std::string error;
        if (!session_.release(&error) && !error.empty()) {
            set_error(error + "，设备 " + config_.device);
            return;
        }
        session_acquired_ = false;
    }
}

void NoriSdkCameraSource::close_queue() {
    {
        std::lock_guard<std::mutex> queue_lock(queue_mutex_);
        queue_closed_ = true;
        pending_frames_.clear();
    }
    queue_condition_.notify_all();
}

uint32_t NoriSdkCameraSource::frame_callback(PVOID, FRAME_BUFFER_DATA* frame,
                                              PVOID user) {
    if (user == nullptr) {
        return NORI_OK;
    }
    auto* context = static_cast<CallbackContext*>(user);
    NoriSdkCameraSource* source = nullptr;
    if (!enter_callback(context, &source)) {
        return NORI_OK;
    }
    try {
        const uint32_t result = source->enqueue_frame(frame);
        source->leave_callback();
        return result;
    } catch (const std::exception&) {
        source->leave_callback();
        return NORI_OK;
    } catch (...) {
        source->leave_callback();
        return NORI_OK;
    }
}

uint32_t NoriSdkCameraSource::enqueue_frame(FRAME_BUFFER_DATA* frame) {
    if (frame == nullptr || frame->pBufAddr == nullptr ||
        callback_context_->cancel_requested.load(std::memory_order_acquire)) {
        return NORI_OK;
    }

    const uint32_t buffer_length = frame->buffer.length;
    if (buffer_length == 0 || buffer_length > kMaxCallbackFrameBytes ||
        frame->buff_Length == 0 ||
        frame->buff_Length > kMaxCallbackFrameBytes ||
        frame->buff_Offset > buffer_length ||
        frame->buff_Length > buffer_length - frame->buff_Offset) {
        return NORI_OK;
    }

    const auto* source = static_cast<const uint8_t*>(frame->pBufAddr);
    PendingFrame pending;
    pending.bytes.assign(source + frame->buff_Offset,
                         source + frame->buff_Offset + frame->buff_Length);
    pending.media_type = frame->PixFormat.u_Format;
    pending.width = frame->PixFormat.u_Width;
    pending.height = frame->PixFormat.u_Height;
    pending.timestamp_ns = steady_timestamp_ns();

    {
        std::lock_guard<std::mutex> queue_lock(queue_mutex_);
        if (queue_closed_ ||
            callback_context_->cancel_requested.load(std::memory_order_acquire)) {
            return NORI_OK;
        }
        if (pending_frames_.size() >= config_.callback_queue_capacity) {
            pending_frames_.pop_front();
        }
        pending_frames_.push_back(std::move(pending));
    }
    queue_condition_.notify_one();
    return NORI_OK;
}

bool NoriSdkCameraSource::enter_callback(CallbackContext* context,
                                          NoriSdkCameraSource** owner) {
    std::lock_guard<std::mutex> lock(context->mutex);
    if (!context->callbacks_enabled ||
        context->cancel_requested.load(std::memory_order_acquire) ||
        context->owner == nullptr) {
        return false;
    }
    *owner = context->owner;
    ++context->callbacks_in_flight;
    return true;
}

void NoriSdkCameraSource::leave_callback() noexcept {
    std::lock_guard<std::mutex> lock(callback_context_->mutex);
    if (callback_context_->callbacks_in_flight != 0) {
        --callback_context_->callbacks_in_flight;
    }
    if (callback_context_->callbacks_in_flight == 0) {
        callback_context_->condition.notify_all();
    }
}

void NoriSdkCameraSource::wait_for_callbacks() noexcept {
    std::unique_lock<std::mutex> lock(callback_context_->mutex);
    callback_context_->condition.wait(
        lock, [this] { return callback_context_->callbacks_in_flight == 0; });
}

void NoriSdkCameraSource::detach_callback_owner() noexcept {
    std::lock_guard<std::mutex> lock(callback_context_->mutex);
    if (callback_context_->owner == this) {
        callback_context_->owner = nullptr;
    }
    callback_context_->callbacks_enabled = false;
    callback_context_->cancel_requested.store(true, std::memory_order_release);
}

void NoriSdkCameraSource::set_error(const std::string& error) {
    std::lock_guard<std::mutex> lock(error_mutex_);
    last_error_ = error;
}

std::string NoriSdkCameraSource::last_error() const {
    std::lock_guard<std::mutex> lock(error_mutex_);
    return last_error_;
}

}  // mosas::app 命名空间结束
