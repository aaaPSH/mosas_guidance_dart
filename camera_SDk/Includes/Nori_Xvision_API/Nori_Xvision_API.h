/**
 * @mainpage Nori Xvision development guide (Linux V4L2)
 *
 * @~chinese
 * @section 概述
 * 相机SDK的功能包括：
 * - SDK 初始化与反初始化
 * - 设备信息获取
 * - 视频流控制
 * - 触发控制
 * - UVC PU 控制
 * - ISP 图像参数控制
 * - Sensor 控制
 * - 用户自定义数据读写
 * - 设备固件更新
 * - GPIO控制
 * - Image Decode
 * - 固件控制
 * 
 * @section 平台支持
 * 当前版本已在以下平台验证：
 * - Linux x86 (32-bit)
 * - Linux x86_64 (64-bit)
 * - Linux ARM32
 * - Linux ARM64
 * 
 * @section 使用说明
 * 文件夹目录结构：
 * - Documentations（SDK开发指南和Demo使用说明等）
 * - Includes（头文件） 
 * - Libraries（动态库） 
 * - Samples（示例代码） 
 * 
 * @~english
 * @section Overview
 * The functions of the camera SDK include:
 * - SDK initialization and deinitialization
 * - Device information access
 * - Video stream control
 * - Trigger control
 * - UVC processing unit control
 * - ISP image parameter control
 * - Sensor control
 * - User data read/write
 * - Device firmware update
 * - GPIO control
 * - Image Decode
 * - Firmware control
 * 
 * @~english
 * @section Supported Platform
 * Current version has been tested on:
 * - Linux x86 (32-bit)
 * - Linux x86_64 (64-bit)
 * - Linux ARM32
 * - Linux ARM64
 * 
 *@section usage instructions
 *Folder directory structure:
 *- Documents (SDK development guidelines and demo usage instructions, etc.)
 *- Includes (header file)
 *- Libraries (dynamic libraries)
 *- Samples (sample code)
 */

#ifndef NORI_XVISION_API_H
#define NORI_XVISION_API_H

#include "Nori_Public.h"
#include "Nori_Error_Define.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup Device Device
 * @brief
 * @~chinese 设备控制
 * @~english Device Control
 *@{
 */

/************************** SDK Init & Uninit **************************/

/**
 * @defgroup SDK_Init_Uninit SDK_Init_Uninit
 * @ingroup Device
 * @brief
 * @~chinese 初始化、反初始化、设备枚举
 * @~english Init & Uninit, Device Enumeration
 * @{
 */

/**
 * @ingroup SDK_Init_Uninit
 * @~chinese
 * @brief   初始化SDK并枚举设备
 * @param   uLayerType          [IN]        枚举传输层类型（参见 Nori_Error_Define.h）
 * @param   pDeviceNum          [IN][OUT]   返回发现的设备数量
 * @return  成功返回 NORI_OK；失败返回错误码
 * @remarks 必须在调用其它接口前调用此函数
 * @~english
 * @brief   Initialize SDK and enumerate devices
 * @param   uLayerType          [IN]        Transport layer type (see Nori_Error_Define.h)
 * @param   pDeviceNum          [IN][OUT]   Returns the number of detected devices
 * @return  NORI_OK on success; error code on failure
 * @remarks This must be called before any other SDK APIs.
 */
NORI_CAMCTRL_API uint32_t Nori_Xvision_Init(IN uint32_t uLayerType, IN OUT uint32_t* pDeviceNum);

/**
 * @ingroup SDK_Init_Uninit
 * @~chinese
 * @brief   反初始化SDK并释放资源
 * @return  成功返回 NORI_OK；失败返回错误码
 * @remarks 应在所有 SDK 操作结束后调用
 * @~english
 * @brief   Uninitialize SDK and release resources
 * @return  NORI_OK on success; error code on failure
 * @remarks Should be called when all SDK operations are finished.
 */
NORI_CAMCTRL_API uint32_t Nori_Xvision_UnInit();

/** @} endgroup SDK_Init_Uninit */


/************************** Device Info **************************/

/**
 * @defgroup Device_Info Device_Info
 * @ingroup Device
 * @brief
 * @~chinese 获取设备相关信息
 * @~chinese - 设备版本信息
 * @~chinese - 设备描述信息
 * @~chinese - 视频流描述信息
 * @~english Get device information
 * @~english - Device version information
 * @~english - Device description information
 * @~english - Video stream description information
 * @{
 */

/**
 * @ingroup Device_Info
 * @~chinese
 * @brief   获取设备版本信息
 * @param   uDeviceID       [IN]        设备索引
 * @param   pVersion        [IN][OUT]   输出版本信息结构体（见 VERSION_INFO）
 * @return  成功返回 NORI_OK；失败返回错误码
 * @~english
 * @brief   Get device version information
 * @param   uDeviceID       [IN]        Device index
 * @param   pVersion        [IN][OUT]   Output version information (see VERSION_INFO)
 * @return  NORI_OK on success; error code on failure
 */
NORI_CAMCTRL_API uint32_t Nori_Xvision_GetVersion(IN uint32_t uDeviceID, IN OUT VERSION_INFO* pVersion);

/**
 * @ingroup Device_Info
 * @~chinese
 * @brief   获取设备描述信息
 * @param   uDeviceID       [IN]        设备索引
 * @param   pDeviceInfo     [IN][OUT]   输出设备描述（见 DEVICE_INFO）
 * @return  成功返回 NORI_OK；失败返回错误码
 * @~english
 * @brief   Get device description information
 * @param   uDeviceID       [IN]        Device index
 * @param   pDeviceInfo     [IN][OUT]   Output device description (see DEVICE_INFO)
 * @return  NORI_OK on success; error code on failure
 */
NORI_CAMCTRL_API uint32_t Nori_Xvision_GetDeviceInfo(IN uint32_t uDeviceID, IN OUT DEVICE_INFO* pDeviceInfo);

/**
 * @ingroup Device_Info
 * @~chinese
 * @brief   获取支持的视频格式/分辨率/帧率个数
 * @param   uDeviceID       [IN]        设备索引
 * @param   pNum            [IN][OUT]   输出视频格式信息数量
 * @return  成功返回 NORI_OK；失败返回错误码
 * @~english
 * @brief   Get number of supported video formats/resolutions/frame rates
 * @param   uDeviceID       [IN]        Device index
 * @param   pNum            [IN][OUT]   Output number of video format entries
 * @return  NORI_OK on success; error code on failure
 */
NORI_CAMCTRL_API uint32_t Nori_Xvision_GetDeviceVideoInfoSize(IN uint32_t uDeviceID, IN OUT uint32_t* pNum);

/**
 * @ingroup Device_Info
 * @~chinese
 * @brief   获取指定索引的视频格式/分辨率/帧率信息
 * @param   uDeviceID       [IN]        设备索引
 * @param   uVideoInfoIndex [IN]        视频信息索引（0..n-1）
 * @param   pVideoInfo      [IN][OUT]   输出视频信息（见 VIDEO_INFO）
 * @return  成功返回 NORI_OK；失败返回错误码
 * @~english
 * @brief   Get video format/resolution/frame rate information by index
 * @param   uDeviceID       [IN]        Device index
 * @param   uVideoInfoIndex [IN]        Video info index (0..n-1)
 * @param   pVideoInfo      [IN][OUT]   Output video info (see VIDEO_INFO)
 * @return  NORI_OK on success; error code on failure
 */
NORI_CAMCTRL_API uint32_t Nori_Xvision_GetDeviceVideoInfo(IN uint32_t uDeviceID, IN uint32_t uVideoInfoIndex, IN OUT VIDEO_INFO *pVideoInfo);

/** @} endgroup Device_Info */


/************************** Video Stream **************************/

/**
 * @defgroup Video_Stream Video_Stream
 * @ingroup Device
 * @brief
 * @~chinese 视频流控制
 * @~chinese - 初始化视频采集
 * @~chinese - 注册视频帧回调函数
 * @~chinese - 反初始化视频采集
 * @~chinese - 查询视频流状态
 * @~chinese - 启动视频采集
 * @~chinese - 停止视频采集
 * @~chinese - 主动获取帧数据
 * @~chinese - 帧资源回收 
 * @~english Video stream
 * @~english - Initialize video capture
 * @~english - Register the video frame callback function
 * @~english - Reverse Initialization Video Capture
 * @~english - Query the video stream status
 * @~english - Start video capture
 * @~english - Stop video capture
 * @~english - Proactively retrieve frame data
 * @~english - Frame Resource Recycling
 * @{
 */

/**
 * @ingroup Video_Stream
 * @~chinese
 * @brief   初始化视频采集，配置像素格式/分辨率/帧率
 * @param   uDeviceID       [IN]        设备索引
 * @param   PixInfo         [IN]        视频信息（格式、宽、高、fps）
 * @return  成功返回 NORI_OK；失败返回错误码
 * @remarks 该接口在开始采集前调用
 * @~english
 * @brief   Initialize video capture with pixel format/resolution/frame rate
 * @param   uDeviceID       [IN]        Device index
 * @param   PixInfo         [IN]        Video information (format, width, height, fps)
 * @return  NORI_OK on success; error code on failure
 * @remarks Call before starting capture.
 */
NORI_CAMCTRL_API uint32_t Nori_Xvision_DeviceVideoInit(IN uint32_t uDeviceID, IN VIDEO_INFO PixInfo);

/**
 * @ingroup Video_Stream
 * @~chinese
 * @brief   注册视频帧回调函数
 * @param   uDeviceID       [IN]        设备索引
 * @param   pexe            [IN]        回调函数，函数签名见 PCall_Back_Frame
 * @param   pUser           [IN]        回调时传递的用户数据指针
 * @return  成功返回 NORI_OK；失败返回错误码
 * @~english
 * @brief   Register video frame callback
 * @param   uDeviceID       [IN]        Device index
 * @param   pexe            [IN]        Callback function pointer (see PCall_Back_Frame)
 * @param   pUser           [IN]        User pointer passed to callback
 * @return NORI_OK on success; error code on failure
 */
NORI_CAMCTRL_API uint32_t Nori_Xvision_VideoCallBack(IN uint32_t uDeviceID, IN PCall_Back_Frame pexe, IN void * pUser);

/**
 * @ingroup Video_Stream
 * @~chinese
 * @brief   反初始化视频采集，释放相关资源
 * @param   uDeviceID       [IN]        设备索引
 * @return  成功返回 NORI_OK；失败返回错误码
 * @~english
 * @brief   Uninitialize video capture and free resources
 * @param   uDeviceID       [IN]        Device index
 * @return  NORI_OK on success; error code on failure
 */
NORI_CAMCTRL_API uint32_t Nori_Xvision_DeviceVideoUnInit(IN uint32_t uDeviceID);

/**
 * @ingroup Video_Stream
 * @~chinese
 * @brief   查询/等待视频流状态
 * @param   uDeviceID       [IN]        设备索引
 * @param   block           [IN]        是否阻塞等待
 * @param   uMsec           [IN]        阻塞超时时间（毫秒）
 * @return  成功返回 NORI_OK；超时或失败返回错误码
 * @~english
 * @brief   Query or wait for video stream status (frame arrival)
 * @param   uDeviceID       [IN]        Device index
 * @param   block           [IN]        Blocking flag
 * @param   uMsec           [IN]        Timeout in milliseconds
 * @return  NORI_OK on success; error code on timeout/failure
 */
NORI_CAMCTRL_API uint32_t Nori_Xvision_VideoStreamStatus(IN uint32_t uDeviceID, IN bool block, IN uint32_t uMsec);

/**
 * @ingroup Video_Stream
 * @~chinese
 * @brief   启动视频采集（开始流）
 * @param   uDeviceID       [IN]        设备索引
 * @return  成功返回 NORI_OK；失败返回错误码
 * @~english
 * @brief   Start video streaming
 * @param   uDeviceID       [IN]        Device index
 * @return  NORI_OK on success; error code on failure
 */
NORI_CAMCTRL_API uint32_t Nori_Xvision_VideoStart(IN uint32_t uDeviceID);

/**
 * @ingroup Video_Stream
 * @~chinese
 * @brief   停止视频采集（停止流）
 * @param   uDeviceID       [IN]        设备索引
 * @return  成功返回 NORI_OK；失败返回错误码
 * @~english
 * @brief   Stop video streaming
 * @param   uDeviceID       [IN]        Device index
 * @return  NORI_OK on success; error code on failure
 */
NORI_CAMCTRL_API uint32_t Nori_Xvision_VideoStop(IN uint32_t uDeviceID);

/**
 * @ingroup Video_Stream
 * @~chinese
 * @brief   从内部队列获取一帧图像缓冲
 * @param   uDeviceID       [IN]        设备索引
 * @param   block           [IN]        阻塞标志，true 阻塞等待
 * @param   uMsec           [IN]        阻塞超时（毫秒）
 * @return  成功返回 FRAME_BUFFER_DATA* 指针；超时或失败返回 NULL
 * @~english
 * @brief   Get a frame buffer from internal queue
 * @param   uDeviceID       [IN]        Device index
 * @param   block           [IN]        Blocking flag, true to wait
 * @param   uMsec           [IN]        Timeout in milliseconds
 * @return  FRAME_BUFFER_DATA* on success; NULL on timeout or failure
 */
NORI_CAMCTRL_API FRAME_BUFFER_DATA* Nori_Xvision_GetFrameBuff(IN uint32_t uDeviceID, IN bool block, IN uint32_t uMsec);

/**
 * @ingroup Video_Stream
 * @~chinese
 * @brief   释放一帧图像缓冲，通知内部池可重用
 * @param   uDeviceID       [IN]        设备索引
 * @param   pFrame          [IN]        要释放的帧指针
 * @return  成功返回 NORI_OK；失败返回错误码
 * @~english
 * @brief   Free a frame buffer, return it to internal pool
 * @param   uDeviceID       [IN]        Device index
 * @param   pFrame          [IN]        Frame pointer to free
 * @return  NORI_OK on success; error code on failure
 */
NORI_CAMCTRL_API uint32_t Nori_Xvision_FreeFrameBuff(IN uint32_t uDeviceID, IN FRAME_BUFFER_DATA* pFrame);

/** @} endgroup Video_Stream */


/**
 * @defgroup Trigger_Control Trigger_Control
 * @ingroup Device
 * @brief
 * @~chinese 触发控制（软件/硬件/指令）
 * @~chinese - 触发模式配置
 * @~chinese  - 软件触发：先设置软件触发模式，后设置软件触发频率
 * @~chinese  - 硬件触发：先设置硬件触发模式，后外部提供脉冲信号
 * @~chinese  - 指令触发：先设置指令触发模式，后续每调用一次指令触发指令函数，输出一帧图像数据
 * @~chinese - 软件触发频率配置
 * @~chinese - 指令触发指令
 * @~english Trigger control (software/hardware/command/button)
 * @~english - Trigger Mode Configuration
 * @~english  - Software triggering: First set the software triggering mode, then set the software triggering frequency
 * @~english  - Hardware triggering: Set the hardware triggering mode first, and then provide pulse signals externally
 * @~english  - Instruction triggering: First set the instruction triggering mode, and then every time the instruction triggering function is called, output one frame of image data
 * @~english - Software trigger frequency configuration
 * @~english - Instruction triggers instruction
 * @{
 */

/**
 * @ingroup Trigger_Control
 * @~chinese
 * @brief   获取触发模式
 * @param   uDeviceID       [IN]        设备索引
 * @param   pValue          [IN][OUT]   返回当前触发模式（参见 E_TRIGGER_MODE）
 * @return  成功返回 NORI_OK；失败返回错误码
 * @~english
 * @brief   Get current trigger mode
 * @param   uDeviceID       [IN]        Device index
 * @param   pValue [IN][OUT] Returns current trigger mode (see E_TRIGGER_MODE)
 * @return  NORI_OK on success; error code on failure
 */
NORI_CAMCTRL_API uint32_t Nori_Xvision_GetTriggerMode(IN uint32_t uDeviceID, IN OUT E_TRIGGER_MODE *pValue);

/**
 * @ingroup Trigger_Control
 * @~chinese
 * @brief   设置触发模式
 * @param   uDeviceID       [IN]        设备索引
 * @param   uValue          [IN]        要设置的触发模式（参见 E_TRIGGER_MODE）
 * @return  成功返回 NORI_OK；失败返回错误码
 * @~english
 * @brief   Set trigger mode
 * @param   uDeviceID       [IN]        Device index
 * @param   uValue          [IN]        Trigger mode to set (see E_TRIGGER_MODE)
 * @return NORI_OK on success; error code on failure
 */
NORI_CAMCTRL_API uint32_t Nori_Xvision_SetTriggerMode(IN uint32_t uDeviceID, IN E_TRIGGER_MODE uValue);

/**
 * @ingroup Trigger_Control
 * @~chinese
 * @brief   获取软件触发频率（单位：Hz）
 * @param   uDeviceID       [IN]        设备索引
 * @param   pValue          [IN][OUT]   返回当前软件触发频率值
 * @return  成功返回 NORI_OK；失败返回错误码
 * @~english
 * @brief   Get software trigger frequency (unit: Hz or implementation-specific)
 * @param   uDeviceID       [IN]        Device index
 * @param   pValue          [IN][OUT]   Returns current software trigger frequency value
 * @return  NORI_OK on success; error code on failure
 */
NORI_CAMCTRL_API uint32_t Nori_Xvision_GetSoftTriggerFrequency(IN uint32_t uDeviceID, IN OUT uint32_t *pValue);

/**
 * @ingroup Trigger_Control
 * @~chinese
 * @brief   设置软件触发频率（单位：Hz）
 * @param   uDeviceID       [IN]        设备索引
 * @param   uValue          [IN]        要设置的软件触发频率值
 * @return  成功返回 NORI_OK；失败返回错误码
 * @~english
 * @brief   Set software trigger frequency (unit: Hz or implementation-specific)
 * @param   uDeviceID       [IN]        Device index
 * @param   uValue          [IN]        Frequency value to set
 * @return  NORI_OK on success; error code on failure
 */
NORI_CAMCTRL_API uint32_t Nori_Xvision_SetSoftTriggerFrequency(IN uint32_t uDeviceID, IN uint32_t uValue);

/**
 * @ingroup Trigger_Control
 * @~chinese
 * @brief   发送一次指令触发（一次调用触发一帧）
 * @param   uDeviceID       [IN]        设备索引
 * @return  成功返回 NORI_OK；失败返回错误码
 * @remarks 使用指令触发前，请先通过 Nori_Xvision_SetTriggerMode 设置为 COMMAND_TRIGGER_MODE
 * @~english
 * @brief   Send a command trigger (one call triggers one frame)
 * @param   uDeviceID       [IN]        Device index
 * @return  NORI_OK on success; error code on failure
 * @remarks Before using command trigger, set mode to COMMAND_TRIGGER_MODE via Nori_Xvision_SetTriggerMode
 */
NORI_CAMCTRL_API uint32_t Nori_Xvision_SetTriggerCommand(IN uint32_t uDeviceID);

/** @} endgroup Trigger_Control */


/************************** UVC Processing Unit Control **************************/

/**
 * @defgroup UVC_PU UVC_PU
 * @ingroup Device
 * @brief
 * @~chinese UVC Processing Unit 控制（亮度/对比度/增益/曝光等）
 * @~chinese - 标准UVC Processing Unit 控制
 * @~chinese  - V4L2_CID_BRIGHTNESS					        亮度
 * @~chinese  - V4L2_CID_CONTRAST					          对比度									
 * @~chinese  - V4L2_CID_HUE						            色调
 * @~chinese  - V4L2_CID_SATURATION					        饱和度
 * @~chinese  - V4L2_CID_SHARPNESS					        清晰度
 * @~chinese  - V4L2_CID_GAMMA						          伽玛
 * @~chinese  - V4L2_CID_AUTO_WHITE_BALANCE			    自动白平衡 开关(0 手动模式,1 自动模式)
 * @~chinese  - V4L2_CID_WHITE_BALANCE_TEMPERATURE	手动白平衡 值
 * @~chinese  - V4L2_CID_BACKLIGHT_COMPENSATION		  逆光对比
 * @~chinese  - V4L2_CID_GAIN						            增益
 * @~chinese  - V4L2_CID_POWER_LINE_FREQUENCY		    电力线频率
 * @~chinese  - V4L2_CID_ZOOM_ABSOLUTE				      缩放
 * @~chinese  - V4L2_CID_FOCUS_AUTO					        自动焦点 开关(0 手动模式,1 自动模式)
 * @~chinese  - V4L2_CID_FOCUS_ABSOLUTE				      手动焦点 值
 * @~chinese  - V4L2_CID_EXPOSURE_AUTO				      自动曝光 开关(1 手动模式,3 自动模式)
 * @~chinese  - V4L2_CID_EXPOSURE_ABSOLUTE			    手动曝光 值 
 * @~chinese  - V4L2_CID_PAN_ABSOLUTE				        全景
 * @~chinese  - V4L2_CID_TILT_ABSOLUTE				      倾斜
 * @~chinese  - V4L2_CID_EXPOSURE_AUTO_PRIORITY		  低亮度补偿(0 保持固定帧率,1 低光时会自动降帧) 
 * @~chinese  - 滚动（roll）没办法通过PU控制，可通过Nori_Xvision_SetSensorMirrorFlip/Nori_Xvision_GetSensorMirrorFlip实现滚动映射的功能
 * @~english UVC Processing Unit Controls (brightness/contrast/gain/exposure/etc.)
 * @~english - Standard UVC Processing Unit Controls
 * @~english  - V4L2_CID_BRIGHTNESS                       Brightness
 * @~english  - V4L2_CID_CONTRAST                         Contrast
 * @~english  - V4L2_CID_HUE                              Hue
 * @~english  - V4L2_CID_SATURATION                       Saturation
 * @~english  - V4L2_CID_SHARPNESS                        Sharpness
 * @~english  - V4L2_CID_GAMMA                            Gamma
 * @~english  - V4L2_CID_AUTO_WHITE_BALANCE               Auto white balance (0 = manual, 1 = auto)
 * @~english  - V4L2_CID_WHITE_BALANCE_TEMPERATURE        Manual white balance value
 * @~english  - V4L2_CID_BACKLIGHT_COMPENSATION           Backlight compensation
 * @~english  - V4L2_CID_GAIN                             Gain
 * @~english  - V4L2_CID_POWER_LINE_FREQUENCY             Power line frequency
 * @~english  - V4L2_CID_ZOOM_ABSOLUTE                    Zoom
 * @~english  - V4L2_CID_FOCUS_AUTO                       Auto focus (0 = manual, 1 = auto)
 * @~english  - V4L2_CID_FOCUS_ABSOLUTE                   Manual focus value
 * @~english  - V4L2_CID_EXPOSURE_AUTO                    Auto exposure (1 = manual, 3 = auto)
 * @~english  - V4L2_CID_EXPOSURE_ABSOLUTE                Manual exposure value
 * @~english  - V4L2_CID_PAN_ABSOLUTE                     Pan
 * @~english  - V4L2_CID_TILT_ABSOLUTE                    Tilt
 * @~english  - V4L2_CID_EXPOSURE_AUTO_PRIORITY           Low-light compensation (0 = fixed FPS, 1 = auto FPS drop in low light)
 * @~english  - Roll control cannot be set via PU; rolling effect can be achieved using Nori_Xvision_SetSensorMirrorFlip / Nori_Xvision_GetSensorMirrorFlip
 * @{
 */

/**
 * @ingroup UVC_PU
 * @~chinese
 * @brief   获取 Processing Unit 控件信息（当前值/范围/步长等）
 * @param   uDeviceID       [IN]        相机索引
 * @param   id              [IN]        控件 ID(参见 v4l2-control.h 中的 V4L2_CID_*)                             
 * @param   pCur            [IN][OUT]   返回当前值
 * @param   pFlags          [IN][OUT]   返回标志/使能信息
 * @param   pStep           [IN][OUT]   返回步长
 * @param   pMin            [IN][OUT]   返回最小值
 * @param   pMax            [IN][OUT]   返回最大值
 * @param   pDefault        [IN][OUT]   返回默认值
 * @return  成功返回 NORI_OK；失败返回错误码
 * @~english
 * @brief   Get Processing Unit control current value and range info
 * @param   uDeviceID       [IN]        Device index
 * @param   id              [IN]        Control ID (see V4L2_CID_* in v4l2-control.h)
 * @param   pCur            [IN][OUT]   Returns current value
 * @param   pFlags          [IN][OUT]   Returns flags/enable info
 * @param   pStep           [IN][OUT]   Returns step
 * @param   pMin            [IN][OUT]   Returns minimum
 * @param   pMax            [IN][OUT]   Returns maximum
 * @param   pDefault        [IN][OUT]   Returns default value
 * @return NORI_OK on success; error code on failure
 */
NORI_CAMCTRL_API uint32_t Nori_Xvision_GetProcessingUnitControl(IN uint32_t uDeviceID, IN int32_t id, IN OUT int32_t *pCur, IN OUT int32_t *pFlags, IN OUT int32_t *pStep, IN OUT int32_t *pMin, IN OUT int32_t *pMax, IN OUT int32_t *pDefault);

/**
 * @ingroup UVC_PU
 * @~chinese
 * @brief   设置 Processing Unit 控件目标值
 * @param   uDeviceID       [IN]        相机索引
 * @param   id              [IN]        控件 ID（参见 v4l2-control.h）
 * @param   val             [IN]        目标值
 * @return  成功返回 NORI_OK；失败返回错误码
 * @~english
 * @brief   Set Processing Unit control target value
 * @param   uDeviceID       [IN]        Device index
 * @param   id              [IN]        Control ID (see v4l2-control.h)
 * @param   val             [IN]        Target value
 * @return NORI_OK on success; error code on failure
 */
NORI_CAMCTRL_API uint32_t Nori_Xvision_SetProcessingUnitControl(IN uint32_t uDeviceID, IN int32_t id, IN int32_t val);

/** @} endgroup UVC_PU */


/************************** ISP / Image Parameter Control **************************/

/**
 * @defgroup ISP_Control ISP_Control
 * @ingroup Device
 * @brief
 * @~chinese ISP 图像参数控制与功能
 * @~chinese - 视频类型
 * @~chinese - 多路视频图像位置互换
 * @~chinese - 镜头矫正功能
 * @~chinese - 单次自动白平衡
 * @~chinese - 单次自动聚焦
 * @~chinese - 单次自动曝光
 * @~chinese - 图像偏移
 * @~chinese - 白平衡增益
 * @~english ISP image parameter control and features
 * @~english - Video type
 * @~english - Multi channel video location exchange
 * @~english - Lens correction function
 * @~english - Single Automatic White Balance
 * @~english - Single autofocus
 * @~english - Single automatic exposure
 * @~english - Image offset
 * @~english - White Balance Gain
 * @{
 */

/**
 * @ingroup ISP_Control
 * @~chinese
 * @brief   获取视频类型（RGB/MONO/RAW）
 * @param   uDeviceID       [IN]        设备索引
 * @param   pType           [IN][OUT]   返回视频类型（见 E_VIDEO_TYPE）
 * @return  成功返回 NORI_OK；失败返回错误码
 * @~english
 * @brief   Get video type (RGB/MONO/RAW)
 * @param   uDeviceID       [IN]        Device index
 * @param   pType           [IN][OUT]   Returns video type (see E_VIDEO_TYPE)
 * @return NORI_OK on success; error code on failure
 */
NORI_CAMCTRL_API uint32_t Nori_Xvision_GetVideoType(IN uint32_t uDeviceID, IN OUT E_VIDEO_TYPE *pType);

/**
 * @ingroup ISP_Control
 * @~chinese
 * @brief   设置视频类型（RGB/MONO/RAW）
 * @param   uDeviceID       [IN]        设备索引
 * @param   Type            [IN]        要设置的视频类型（见 E_VIDEO_TYPE）
 * @remarks 此接口必须在Nori_Xvision_VideoStart前使用
 * @return  成功返回 NORI_OK；失败返回错误码
 * @~english
 * @brief   Set video type (RGB/MONO/RAW)
 * @param   uDeviceID       [IN]        Device index
 * @param   Type            [IN]        Video type to set (see E_VIDEO_TYPE)
 * @remarks This interface must be used before Nori_Xview_VideoStart
 * @return  NORI_OK on success; error code on failure
 */
NORI_CAMCTRL_API uint32_t Nori_Xvision_SetVideoType(IN uint32_t uDeviceID, IN E_VIDEO_TYPE Type);

/**
 * @ingroup ISP_Control
 * @~chinese
 * @brief   获取多路视频图像位置互换状态
 * @param   uDeviceID       [IN]        设备索引
 * @param   pValue          [IN][OUT]   0=正常, 1=左右互换
 * @return  成功返回 NORI_OK；失败返回错误码
 * @~english
 * @brief   Get multi-video swap status 
 * @param   uDeviceID       [IN]        Device index
 * @param   pValue          [IN][OUT]   Returns 0 (normal) or 1 (swapped)
 * @return  NORI_OK on success; error code on failure
 */
NORI_CAMCTRL_API uint32_t Nori_Xvision_GetMultipleVideoSwap(IN uint32_t uDeviceID, IN OUT uint32_t *pValue);

/**
 * @ingroup ISP_Control
 * @~chinese
 * @brief   设置多路视频图像位置互换状态
 * @param   uDeviceID       [IN]        设备索引
 * @param   uValue          [IN]        0=正常, 1=左右互换
 * @return 成功返回 NORI_OK；失败返回错误码
 * @~english
 * @brief   Set multi-video swap status
 * @param   uDeviceID       [IN]        Device index
 * @param   uValue          [IN]        0=normal, 1=swapped
 * @return  NORI_OK on success; error code on failure
 */
NORI_CAMCTRL_API uint32_t Nori_Xvision_SetMultipleVideoSwap(IN uint32_t uDeviceID, IN uint32_t uValue);

/**
 * @ingroup ISP_Control
 * @~chinese
 * @brief   获取镜头矫正功能是否开启
 * @param   uDeviceID       [IN]        设备索引
 * @param   pValue          [IN][OUT]   1=开启, 0=关闭
 * @return 成功返回 NORI_OK；失败返回错误码
 * @~english
 * @brief   Get lens correction enable status
 * @param   uDeviceID       [IN]        Device index
 * @param   pValue          [IN][OUT]   1=enable, 0=disable
 * @return NORI_OK on success; error code on failure
 */
NORI_CAMCTRL_API uint32_t Nori_Xvision_GetLensCorrectionEnable(IN uint32_t uDeviceID, IN OUT uint32_t *pValue);

/**
 * @ingroup ISP_Control
 * @~chinese
 * @brief   设置镜头矫正功能开启/关闭
 * @param   uDeviceID       [IN]        设备索引
 * @param   uValue          [IN]        1=开启, 0=关闭
 * @return  成功返回 NORI_OK；失败返回错误码
 * @~english
 * @brief   Set lens correction enable/disable
 * @param   uDeviceID       [IN]        Device index
 * @param   uValue          [IN]        1=enable, 0=disable
 * @return NORI_OK on success; error code on failure
 */
NORI_CAMCTRL_API uint32_t Nori_Xvision_SetLensCorrectionEnable(IN uint32_t uDeviceID, IN uint32_t uValue);

/**
 * @ingroup ISP_Control
 * @~chinese
 * @brief   触发一次自动白平衡（一次性）
 * @param   uDeviceID       [IN]        设备索引
 * @return  成功返回 NORI_OK；失败返回错误码
 * @remarks 调用一次后设备进入稳定白平衡模式
 * @~english
 * @brief   Trigger a one-time auto white balance
 * @param   uDeviceID       [IN]        Device index
 * @return  NORI_OK on success; error code on failure
 * @remarks After one-time AWB, device may enter stable mode and stop continuous adjustments
 */
NORI_CAMCTRL_API uint32_t Nori_Xvision_SetSingleAutoWhiteBalance(IN uint32_t uDeviceID);

/**
 * @ingroup ISP_Control
 * @~chinese
 * @brief   获取一次性自动白平衡状态（0: 完成, 1: 设置中）
 * @param   uDeviceID       [IN]        设备索引
 * @param   pValue          [IN][OUT]   返回状态
 * @return  成功返回 NORI_OK；失败返回错误码
 * @~english
 * @brief   Get single auto white balance status (0: done, 1: in progress)
 * @param   uDeviceID       [IN]        Device index
 * @param   pValue          [IN][OUT]   Returns status
 * @return  NORI_OK on success; error code on failure
 */
NORI_CAMCTRL_API uint32_t Nori_Xvision_GetSingleAutoWhiteBalance(IN uint32_t uDeviceID, IN OUT uint32_t *pValue);

/**
 * @ingroup ISP_Control
 * @~chinese
 * @brief   触发一次自动聚焦（一次性）
 * @param   uDeviceID       [IN]        设备索引
 * @return  成功返回 NORI_OK；失败返回错误码
 * @remarks 调用一次后进入稳定模式
 * @~english
 * @brief   Trigger a one-time auto focus
 * @param   uDeviceID       [IN]        Device index
 * @return  NORI_OK on success; error code on failure
 * @remarks After call, device may enter stable focus mode
 */
NORI_CAMCTRL_API uint32_t Nori_Xvision_SetSingleAutoFocus(IN uint32_t uDeviceID);

/**
 * @ingroup ISP_Control
 * @~chinese
 * @brief   获取一次性自动聚焦状态（0: 完成, 1: 设置中）
 * @param   uDeviceID       [IN]        设备索引
 * @param   pValue          [IN][OUT]   返回状态
 * @return  成功返回 NORI_OK；失败返回错误码
 * @~english
 * @brief   Get single auto focus status (0: done, 1: in progress)
 * @param   uDeviceID       [IN]        Device index
 * @param   pValue          [IN][OUT]   Returns status
 * @return  NORI_OK on success; error code on failure
 */
NORI_CAMCTRL_API uint32_t Nori_Xvision_GetSingleAutoFocus(IN uint32_t uDeviceID, IN OUT uint32_t *pValue);

/**
 * @ingroup ISP_Control
 * @~chinese
 * @brief   触发一次自动曝光（一次性）
 * @param   uDeviceID       [IN]        设备索引
 * @return  成功返回 NORI_OK；失败返回错误码
 * @remarks 调用一次后进入稳定曝光
 * @~english
 * @brief   Trigger a one-time auto exposure
 * @param   uDeviceID       [IN]        Device index
 * @return  NORI_OK on success; error code on failure
 * @remarks After call, device may enter stable exposure mode
 */
NORI_CAMCTRL_API uint32_t Nori_Xvision_SetSingleAutoExposure(IN uint32_t uDeviceID);

/**
 * @ingroup ISP_Control
 * @~chinese
 * @brief   获取一次性自动曝光状态（0: 完成, 1: 设置中）
 * @param   uDeviceID       [IN]        设备索引
 * @param   pValue          [IN][OUT]   返回状态
 * @return  成功返回 NORI_OK；失败返回错误码
 * @~english
 * @brief   Get single auto exposure status (0: done, 1: in progress)
 * @param   uDeviceID       [IN]        Device index
 * @param   pValue          [IN][OUT]   Returns status
 * @return NORI_OK on success; error code on failure
 */
NORI_CAMCTRL_API uint32_t Nori_Xvision_GetSingleAutoExposure(IN uint32_t uDeviceID, IN OUT uint32_t *pValue);

/**
 * @ingroup ISP_Control
 * @~chinese
 * @brief   获取图像偏移（ROI 偏移）
 * @param   uDeviceID       [IN]        设备索引
 * @param   pOffset         [IN][OUT]   返回偏移信息（见 XY_OFFSET）
 * @return  成功返回 NORI_OK；失败返回错误码
 * @~english
 * @brief   Get image offset (ROI offset)
 * @param   uDeviceID       [IN]        Device index
 * @param   pOffset         [IN][OUT]   Returns offset info (see XY_OFFSET)
 * @return  NORI_OK on success; error code on failure
 */
NORI_CAMCTRL_API uint32_t Nori_Xvision_GetImageOffset(IN uint32_t uDeviceID, IN OUT XY_OFFSET *pOffset);

/**
 * @ingroup ISP_Control
 * @~chinese
 * @brief   设置图像偏移（ROI）
 * @param   uDeviceID       [IN]        设备索引
 * @param   Offset          [IN]        偏移值（见 XY_OFFSET）
 * @return  成功返回 NORI_OK；失败返回错误码
 * @remarks 偏移值需为2的倍数
 * @~english
 * @brief   Set image offset (ROI)
 * @param   uDeviceID       [IN]        Device index
 * @param   Offset          [IN]        Offset values (see XY_OFFSET)
 * @return  NORI_OK on success; error code on failure
 * @remarks The offset value must be a multiple of 2
 */
NORI_CAMCTRL_API uint32_t Nori_Xvision_SetImageOffset(IN uint32_t uDeviceID, IN XY_OFFSET Offset);

/**
 * @ingroup ISP_Control
 * @~chinese
 * @brief   获取白平衡增益
 * @param   uDeviceID       [IN]        设备索引
 * @param   pGain           [IN][OUT]   返回白平衡增益（见 WB_RGB_GAIN）
 * @return  成功返回 NORI_OK；失败返回错误码
 * @~english
 * @brief   Get white balance gain
 * @param   uDeviceID       [IN]        Device index
 * @param   pGain           [IN][OUT]   Returns white balance gain (see WB_RGB_GAIN)
 * @return NORI_OK on success; error code on failure
 */
NORI_CAMCTRL_API uint32_t Nori_Xvision_GetWhiteBalanceGain(IN uint32_t uDeviceID, IN OUT WB_RGB_GAIN* pGain);

/**
 * @ingroup ISP_Control
 * @~chinese
 * @brief   设置白平衡增益（手动）
 * @param   uDeviceID       [IN]        设备索引
 * @param   Gain            [IN]        增益值（见 WB_RGB_GAIN）
 * @return  成功返回 NORI_OK；失败返回错误码
 * @remarks 在设置前应通过Nori_Xvision_SetProcessingUnitControl接口将白平衡设为手动模式
 * @~english
 * @brief   Set white balance gain (manual)
 * @param   uDeviceID       [IN]        Device index
 * @param   Gain            [IN]        White balance gain (see WB_RGB_GAIN)
 * @return  NORI_OK on success; error code on failure
 * @remarks Before setting up, the white balance should be set to manual mode via the Nori_Xvision_SetProcessingUnitControl interface
 */
NORI_CAMCTRL_API uint32_t Nori_Xvision_SetWhiteBalanceGain(IN uint32_t uDeviceID, IN WB_RGB_GAIN Gain);

/** @} endgroup ISP_Control */


/************************** Sensor Control **************************/

/**
 * @defgroup Sensor_Control Sensor_Control
 * @ingroup Device
 * @brief
 * @~chinese Sensor 级别控制
 * @~chinese - raw 属性
 * @~chinese - 黑电平
 * @~chinese - 快门时间
 * @~chinese - 增益倍数
 * @~chinese - 镜像/翻转
 * @~english Sensor-level control
 * @~english - Raw attribute
 * @~english - Black level
 * @~english - Shutter Time
 * @~english - Gain multiplier
 * @~english - Mirror/Flip
 * @{
 */

/**
 * @ingroup Sensor_Control
 * @~chinese
 * @brief   获取 Sensor RAW 描述信息（深度、颜色编码）
 * @param   uDeviceID       [IN]        设备索引
 * @param   pRawInfo        [IN][OUT]   返回 RAW 描述（见 SENSOR_RAW_INFO）
 * @return  成功返回 NORI_OK；失败返回错误码
 * @~english
 * @brief   Get sensor RAW description (depth, color coding)
 * @param   uDeviceID       [IN]        Device index
 * @param   pRawInfo        [IN][OUT]   Returns RAW description (see SENSOR_RAW_INFO)
 * @return  NORI_OK on success; error code on failure
 */
NORI_CAMCTRL_API uint32_t Nori_Xvision_GetSensorRawInfo(IN uint32_t uDeviceID, IN OUT SENSOR_RAW_INFO *pRawInfo);

/**
 * @ingroup Sensor_Control
 * @~chinese
 * @brief   获取黑电平（Black Level）
 * @param   uDeviceID       [IN]        设备索引
 * @param   pValue          [IN][OUT]   返回黑电平值
 * @return  成功返回 NORI_OK；失败返回错误码
 * @~english
 * @brief   Get sensor black level
 * @param   uDeviceID       [IN]        Device index
 * @param   pValue          [IN][OUT]   Returns black level value
 * @return NORI_OK on success; error code on failure
 */
NORI_CAMCTRL_API uint32_t Nori_Xvision_GetSensorBlackLevel(IN uint32_t uDeviceID, IN OUT uint32_t *pValue);

/**
 * @ingroup Sensor_Control
 * @~chinese
 * @brief   设置黑电平（Black Level）
 * @param   uDeviceID       [IN]        设备索引
 * @param   uValue          [IN]        黑电平值
 * @return  成功返回 NORI_OK；失败返回错误码
 * @~english
 * @brief   Set sensor black level
 * @param   uDeviceID       [IN]        Device index
 * @param   uValue          [IN]        Black level value
 * @return  NORI_OK on success; error code on failure
 */
NORI_CAMCTRL_API uint32_t Nori_Xvision_SetSensorBlackLevel(IN uint32_t uDeviceID, IN uint32_t uValue);

/**
 * @ingroup Sensor_Control
 * @~chinese
 * @brief   获取传感器增益(单位: 倍数)
 * @param   uDeviceID       [IN]        设备索引
 * @param   pCur            [IN][OUT]   当前增益倍数
 * @param   pMin            [IN][OUT]   最小增益倍数
 * @param   pMax            [IN][OUT]   最大增益倍数
 * @param   pStep           [IN][OUT]   增益倍数步长
 * @return  成功返回 NORI_OK；失败返回错误码
 * @~english
 * @brief   Get sensor gain (Unit: Multiple)
 * @param   uDeviceID       [IN]        Device index
 * @param   pCur            [IN][OUT]   Current gain multiple
 * @param   pMin            [IN][OUT]   Minimum gain multiple
 * @param   pMax            [IN][OUT]   Maximum gain multiple
 * @param   pStep           [IN][OUT]   Gain multiplier step size
 * @return  NORI_OK on success; error code on failure
 */
NORI_CAMCTRL_API uint32_t Nori_Xvision_GetSensorGain(IN uint32_t uDeviceID, IN OUT uint32_t *pCur, uint32_t *pMin,	uint32_t *pMax,	uint32_t *pStep);

/**
 * @ingroup Sensor_Control
 * @~chinese
 * @brief 设置传感器增益(单位: 倍数)
 * @param   uDeviceID       [IN]        设备索引
 * @param   uValue          [IN]        目标增益
 * @return  成功返回 NORI_OK；失败返回错误码
 * @remarks 在设置前需通过Nori_Xvision_SetProcessingUnitControl接口设置曝光模式为手动模式
 * @~english
 * @brief   Set sensor gain (Unit: Multiple)
 * @param   uDeviceID       [IN]        Device index
 * @param   uValue          [IN]        Target gain value
 * @return  NORI_OK on success; error code on failure
 * @remarks Before setting the gain, it is necessary to set the exposure mode to manual mode through the Nori_Xvision_SetProcessingUnitControl interface
 */
NORI_CAMCTRL_API uint32_t Nori_Xvision_SetSensorGain(IN uint32_t uDeviceID, IN uint32_t uValue);

/**
 * @ingroup Sensor_Control
 * @~chinese
 * @brief   获取快门/曝光时间（单位：微秒）
 * @param   uDeviceID       [IN]        设备索引
 * @param   pValue          [IN][OUT]   返回曝光时间
 * @return  成功返回 NORI_OK；失败返回错误码
 * @~english
 * @brief   Get sensor shutter/exposure time microsecond
 * @param   uDeviceID       [IN]        Device index
 * @param   pValue          [IN][OUT]   Returns exposure time
 * @return  NORI_OK on success; error code on failure
 */
NORI_CAMCTRL_API uint32_t Nori_Xvision_GetSensorShutter(IN uint32_t uDeviceID, IN OUT uint32_t *pValue);

/**
 * @ingroup Sensor_Control
 * @~chinese
 * @brief   设置快门/曝光时间（单位：微秒）
 * @param   uDeviceID       [IN]        设备索引
 * @param   uValue          [IN]        曝光时间值
 * @return  成功返回 NORI_OK；失败返回错误码
 * @remarks 在设置前需通过Nori_Xvision_SetProcessingUnitControl接口设置曝光模式为手动模式
 * @~english
 * @brief   Set sensor shutter/exposure time (microsecond)
 * @param   uDeviceID       [IN]        Device index
 * @param   uValue          [IN]        Exposure time value
 * @return  NORI_OK on success; error code on failure
 * @remarks Before setting the shutter, it is necessary to set the exposure mode to manual mode through the Nori_Xvision_SetProcessingUnitControl interface
 */
NORI_CAMCTRL_API uint32_t Nori_Xvision_SetSensorShutter(IN uint32_t uDeviceID, IN uint32_t uValue);

/**
 * @ingroup Sensor_Control
 * @~chinese
 * @brief   获取相机镜像与翻转设置
 * @param   uDeviceID       [IN]        设备索引
 * @param   pMirrorFlip     [IN][OUT]   返回镜像/翻转状态（见 E_SENSOR_MIRROR_FLIP）
 * @return  成功返回 NORI_OK；失败返回错误码
 * @~english
 * @brief   Get sensor mirror & flip settings
 * @param   uDeviceID       [IN]        Device index
 * @param   pMirrorFlip     [IN][OUT]   Returns mirror/flip state (see E_SENSOR_MIRROR_FLIP)
 * @return  NORI_OK on success; error code on failure
 */
NORI_CAMCTRL_API uint32_t Nori_Xvision_GetSensorMirrorFlip(IN uint32_t uDeviceID, IN OUT E_SENSOR_MIRROR_FLIP* pMirrorFlip);

/**
 * @ingroup Sensor_Control
 * @~chinese
 * @brief   设置相机镜像与翻转
 * @param   uDeviceID       [IN]        设备索引
 * @param   MirrorFlip      [IN]        要设置的镜像/翻转状态（见 E_SENSOR_MIRROR_FLIP）
 * @return  成功返回 NORI_OK；失败返回错误码
 * @~english
 * @brief Set sensor mirror & flip
 * @param   uDeviceID       [IN]        Device index
 * @param   MirrorFlip      [IN]        Mirror/flip state to set (see E_SENSOR_MIRROR_FLIP)
 * @return NORI_OK on success; error code on failure
 */
NORI_CAMCTRL_API uint32_t Nori_Xvision_SetSensorMirrorFlip(IN uint32_t uDeviceID, IN E_SENSOR_MIRROR_FLIP MirrorFlip);

/** @} endgroup Sensor_Control */


/************************** User Data / ESN **************************/

/**
 * @defgroup User_Data_Control User_Data_Control
 * @ingroup Device
 * @brief
 * @~chinese 用户自定义数据读写与ESN
 * @~chinese - 用户自定义数据
 * @~chinese  - 数据只能通过接口函数读写
 * @~chinese  - Gen60数据分256个区域，每个区域可存储4096个字节，共计1MB，数据存储在非易失性存储区域
 * @~chinese  - Gen56数据分64个区域，每个区域可存储4096个字节，共计256KB，数据存储在非易失性存储区域
 * @~chinese  - 设备重新上电/设备固件更新不影响用户自定义写入的数据
 * @~chinese - ESN
 * @~chinese  - 只能通过固件更新的方式写入设备
 * @~chinese  - 设备固件更新,原有设备里面的ESN数据会丢失，新的ESN数据为固件里的ESN数据
 * @~chinese - 建议用户将数据保存在自定义数据区
 * @~english User data read/write and ESN
 * @~english - User defined data
 * @~english  - Data can only be read and written through interface functions
 * @~english  - Gen60 The data is divided into 256 regions, each of which can store 4096 bytes, totaling 1MB. The data is stored in non-volatile storage areas
 * @~english  - Gen56 The data is divided into 64 regions, each of which can store 4096 bytes, totaling 256KB. The data is stored in non-volatile storage areas
 * @~english  - Device re powering on/Device firmware update does not affect user-defined data writing
 * @~english - ESN
 * @~english  - can only be written to the device through firmware updates
 * @~english  - Device firmware update, the ESN data in the original device will be lost, and the new ESN data will be the ESN data in the firmware
 * @~english - It is recommended that users save their data in a custom data area
 *  @{
 */

/**
 * @ingroup User_Data_Control
 * @~chinese
 * @brief   读取用户自定义数据块
 * @param   uDeviceID       [IN]        设备索引
 * @param   iPieceIndex     [IN]        数据块索引（Gen60支持0~255;Gen56支持0~63）
 * @param   pbuff           [IN][OUT]   返回数据缓冲（调用方分配,内存大小4096Bytes）
 * @return  成功返回 NORI_OK；失败返回错误码
 * @~english
 * @brief   Read user-defined data block
 * @param   uDeviceID       [IN]        Device index
 * @param   iPieceIndex     [IN]        Block index (Gen60 support 0~255;Gen56 support 0~63)
 * @param   pbuff           [IN][OUT]   Buffer to receive data (caller allocated,Memory size 4096bytes)
 * @return  NORI_OK on success; error code on failure
 */
NORI_CAMCTRL_API uint32_t Nori_Xvision_GetUserData(IN uint32_t uDeviceID, IN uint32_t iPieceIndex, IN OUT BYTE* pbuff);

/**
 * @ingroup User_Data_Control
 * @~chinese
 * @brief   写入用户自定义数据块
 * @param   uDeviceID       [IN]        设备索引
 * @param   iPieceIndex     [IN]        数据块索引（Gen60支持0~255;Gen56支持0~63）
 * @param   pbuff           [IN]        提供的数据缓冲(内存大小4096Bytes)
 * @return  成功返回 NORI_OK；失败返回错误码
 * @~english
 * @brief   Write user-defined data block
 * @param   uDeviceID       [IN]        Device index
 * @param   iPieceIndex     [IN]        Block index (Gen60 support 0~255;Gen56 support 0~63)
 * @param   pbuff           [IN]        Data buffer to write(Memory size 4096bytes)
 * @return  NORI_OK on success; error code on failure
 */
NORI_CAMCTRL_API uint32_t Nori_Xvision_SetUserData(IN uint32_t uDeviceID, IN uint32_t iPieceIndex, IN BYTE* pbuff);

/**
 * @ingroup User_Data_Control
 * @~chinese
 * @brief   读取用户设备序列号（ESN）
 * @param   uDeviceID       [IN]        设备索引
 * @param   pbuff           [IN][OUT]   返回 ESN 字节数组（调用方分配,64 Bytes）
 * @return  成功返回 NORI_OK；失败返回错误码
 * @~english
 * @brief   Get user ESN (serial number)
 * @param   uDeviceID       [IN]        Device index
 * @param   pbuff           [IN][OUT]   Buffer to receive ESN (caller allocated)
 * @return  NORI_OK on success; error code on failure
 */
NORI_CAMCTRL_API uint32_t Nori_Xvision_GetUserESN(IN uint32_t uDeviceID, IN OUT BYTE* pbuff);

/**
 * @ingroup User_Data_Control
 * @~chinese
 * @brief   设置用户设备序列号（ESN）
 * @param   uDeviceID       [IN]        设备索引
 * @param   pbuff           [IN]        要写入的 ESN 字节数组
 * @return  成功返回 NORI_OK；失败返回错误码
 * @remarks Gen60不支持写入
 * @~english
 * @brief   Set user ESN (serial number)
 * @param   uDeviceID       [IN]        Device index
 * @param   pbuff           [IN]        ESN byte array to write
 * @return  NORI_OK on success; error code on failure
 * @remarks Gen60 not support writing ESN.
 */
NORI_CAMCTRL_API uint32_t Nori_Xvision_SetUserESN(IN uint32_t uDeviceID, IN BYTE* pbuff);


/**
 * @ingroup User_Data_Control
 * @~chinese
*	@brief		用户自定义握手交互	
*	@param		uDeviceID				[IN] 			设备索引
*	@param		pData						[IN] 			用户自定义数据（buf位数只能为0x08）
*	@param		len							[IN] 			用户自定义数据长度（buf位数只能为0x08）
*	@return		成功，返回NORI_OK；错误，返回错误码

 * @~english
*	@brief		User APP Check Data
*	@param		uDeviceID				[IN]			Device index
*	@param		pData						[IN] 			User  Data（Data size must be 0x08）
*	@param		len							[IN] 			User  Data Len（Data size must be 0x08）
*	@return		Success, return NORI_OK; Error, return error code
 */
NORI_CAMCTRL_API uint32_t Nori_Xvision_DeviceCheck(IN uint32_t uDeviceID, IN uint8_t *pData, IN uint32_t len);
/** @} endgroup User_Data_Control */


/************************** Firmware Update **************************/

/**
 * @defgroup Firmware_Update Firmware_Update
 * @ingroup Device
 * @brief
 * @~chinese 设备固件更新
 * @~english Device firmware update
 * @{
 */

/**
 * @ingroup Firmware_Update
 * @~chinese
 * @brief   升级设备固件
 * @param   uDeviceID       [IN]        设备索引
 * @param   pbyFW           [IN]        固件二进制缓冲
 * @param   uSize           [IN]        固件缓冲大小（字节）
 * @param   pf_Schedule     [IN][OUT]   更新进度（0.0 - 100.0）
 * @return  成功返回 NORI_OK；失败返回错误码
 * @remarks 调用者可传入非 NULL 的进度指针以获取进度
 * @~english
 * @brief   Update device firmware using provided buffer
 * @param   uDeviceID       [IN]        Device index
 * @param   pbyFW           [IN]        Firmware binary buffer
 * @param   uSize           [IN]        Size of firmware buffer (bytes)
 * @param   pf_Schedule     [IN][OUT]   Progress value (0.0 - 100.0)
 * @return  NORI_OK on success; error code on failure
 * @remarks Pass a non-NULL progress pointer to receive progress updates.
 */
NORI_CAMCTRL_API uint32_t Nori_Xvision_FirmWareUpdate(IN uint32_t uDeviceID, IN BYTE *pbyFW, IN uint32_t uSize, IN OUT float *pf_Schedule);

/** @} endgroup Firmware_Update */


/************************** GPIO / IO Control **************************/

/**
 * @defgroup Gpio_Control Gpio_Control
 * @ingroup Device
 * @brief
 * @~chinese GPIO 控制
 * @~chinese - GPIO口普通控制
 * @~chinese - GPIO口PWM控制
 * @~english GPIO control
 * @~english - GPIO port ordinary control
 * @~english - GPIO Port PWM Control
 * @{
 */

/**
 * @ingroup Gpio_Control
 * @~chinese
 * @brief   设置 IO 引脚模式与电平
 * @param   uDeviceID       [IN]        设备索引
 * @param   IO_ID           [IN]        引脚 ID（参见 E_GPIO_ID）
 * @param   IoControl       [IN]        IO 配置（见 IO_CONTORL）
 * @return  成功返回 NORI_OK；失败返回错误码
 * @~english
 * @brief   Set IO pin mode and level
 * @param   uDeviceID       [IN]        Device index
 * @param   IO_ID           [IN]        Pin ID (see E_GPIO_ID)
 * @param   IoControl       [IN]        IO control values (see IO_CONTORL)
 * @return  NORI_OK on success; error code on failure
 */
NORI_CAMCTRL_API uint32_t Nori_Xvision_SetIO(IN uint32_t uDeviceID, IN E_GPIO_ID IO_ID, IN IO_CONTORL IoControl);

/**
 * @ingroup Gpio_Control
 * @~chinese
 * @brief   获取 IO 引脚模式与电平
 * @param   uDeviceID       [IN]        设备索引
 * @param   IO_ID           [IN]        引脚 ID（参见 E_GPIO_ID）
 * @param   pIoControl      [IN][OUT]   返回 IO 配置
 * @return  成功返回 NORI_OK；失败返回错误码
 * @~english
 * @brief   Get IO pin mode and level
 * @param   uDeviceID       [IN]        Device index
 * @param   IO_ID           [IN]        Pin ID (see E_GPIO_ID)
 * @param   pIoControl      [IN][OUT]   Returns IO control values
 * @return  NORI_OK on success; error code on failure
 */
NORI_CAMCTRL_API uint32_t Nori_Xvision_GetIO(IN uint32_t uDeviceID, IN E_GPIO_ID IO_ID, IN OUT IO_CONTORL *pIoControl);

/**
 * @ingroup Gpio_Control
 * @~chinese
 * @brief   获取 PWM 频率与占空比
 * @param   uDeviceID       [IN]        设备索引
 * @param   pValue          [IN][OUT]   返回 PWM 频率与占空比（见 FREQUENCY_RATIO）
 * @return  成功返回 NORI_OK；失败返回错误码
 * @~english
 * @brief   Get PWM frequency and duty cycle
 * @param   uDeviceID       [IN]        Device index
 * @param   pValue          [IN][OUT]   Returns PWM frequency and duty cycle (see FREQUENCY_RATIO)
 * @return NORI_OK on success; error code on failure
 */
NORI_CAMCTRL_API uint32_t Nori_Xvision_GetPWMFrequencyRatio(IN uint32_t uDeviceID, IN OUT FREQUENCY_RATIO *pValue);

/**
 * @ingroup Gpio_Control
 * @~chinese
 * @brief   设置 PWM 频率与占空比
 * @param   uDeviceID       [IN]        设备索引
 * @param   uValue          [IN]        要设置的频率/占空比（见 FREQUENCY_RATIO）
 * @return  成功返回 NORI_OK；失败返回错误码
 * @~english
 * @brief   Set PWM frequency and duty cycle
 * @param   uDeviceID       [IN]        Device index
 * @param   uValue          [IN]        Frequency and duty cycle to set (see FREQUENCY_RATIO)
 * @return  NORI_OK on success; error code on failure
 */
NORI_CAMCTRL_API uint32_t Nori_Xvision_SetPWMFrequencyRatio(IN uint32_t uDeviceID, IN FREQUENCY_RATIO uValue);

/** @} endgroup Gpio_Control */

/** @}*/


/**
 * @defgroup Image Image
 * @brief
 * @~chinese 图像解码
 * @~english Image decode
 *@{
 */

 /**
 * @ingroup Image
 * @~chinese
 * @brief   raw10解码成BGR24数据
 * @param   raw             [IN]        raw数据
 * @param   width           [IN]        图像宽度
 * @param   height          [IN]        图像高度
 * @param   out_bgr         [IN][OUT]   BGR24数据缓冲（调用方分配,缓冲区大小:width*height*3）
 * @param   pattern         [IN]        颜色排列方式
 * @param   black_level     [IN]        黑电平
 * @note    BGR24数据按从上到下排列（第一行是图像顶部）
 * @return  无
 * @~english
 * @brief   decoded raw10 to BGR24 data
 * @param   raw             [IN]        raw data
 * @param   width           [IN]        image width
 * @param   height          [IN]        image height
 * @param   out_bgr         [IN][OUT]   BGR24 buff（caller allocated,buffer size:width*height*3）
 * @param   pattern         [IN]        Color arrangement
 * @param   black_level     [IN]        black level
 * @note    BGR24 Data is arranged from top to bottom (the first row is at the top of the image)
 * @return  无
 */
NORI_IMAGECTRL_API void Nori_Xvision_Raw10ToBGR24(IN const uint16_t* raw, IN uint32_t width, IN uint32_t height, IN OUT uint8_t* out_bgr, IN E_SENSOR_COLOR_CODING pattern, IN uint32_t black_level);

  /**
 * @ingroup Image
 * @~chinese
 * @brief   YUYV解码成BGR24数据
 * @param   raw             [IN]        yuyv数据
 * @param   out_bgr         [IN][OUT]   BGR24数据缓冲（调用方分配,缓冲区大小:width*height*3）
 * @param   width           [IN]        图像宽度
 * @param   height          [IN]        图像高度
 * @note    BGR24数据按从上到下排列（第一行是图像顶部）
 * @return  无
 * @~english
 * @brief   decoded raw10 to BGR24 data
 * @param   raw             [IN]        raw data
 * @param   out_bgr         [IN][OUT]   BGR24 buff（caller allocated,buffer size:width*height*3）
 * @param   width           [IN]        image width
 * @param   height          [IN]        image height
 * @note    BGR24 Data is arranged from top to bottom (the first row is at the top of the image)
 * @return  无
 */
NORI_IMAGECTRL_API void Nori_Xvision_YuyvToBGR24(IN uint8_t* yuyv, IN OUT uint8_t* out_bgr, IN int width, IN int height);

/**
* @ingroup Image
* @~chinese
* @brief   保存BRG24到本地BMP
* @param   filename        [IN]        文件名
* @param   bgr24           [IN]        BGR24数据
* @param   width           [IN]        图像宽度
* @param   height          [IN]        图像高度
* @param   topDown         [IN]   指示传入的 BGR24 像素数据的行序是否为自上而下（top-down）存储。
*- true  : 数据按从上到下排列（第一行是图像顶部）
*- false : 数据按从下到上排列（第一行是图像底部）
* @return  false:open file false; true: success
* @~english
* @brief   save BGR24 to local BMP
* @param   filename        [IN]        file name
* @param   bgr24           [IN]        BGR24 data
* @param   width           [IN]        image width
* @param   height          [IN]        image height
* @param   topDown		     [IN]		     indicates whether the row order of the incoming BGR24 pixel data is stored in a top-down manner.
*- true: Data is arranged from top to bottom (the first row is at the top of the image)
*- false: The data is arranged from bottom to top (the first row is at the bottom of the image)
* @return  false:文件打开失败；true：成功
*/
NORI_IMAGECTRL_API bool Nori_Xvision_SaveBGR24ToBMP(IN const char* filename, IN const uint8_t* bgr24, IN uint32_t width, IN uint32_t height,IN bool topDown);


/** @}*/


#ifdef __cplusplus
}
#endif

#endif /* NORI_XVISION_API_H */
