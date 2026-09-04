#ifndef NORI_PUBLIC_H
#define NORI_PUBLIC_H

#include <stdio.h>
#include <stdint.h>
#include <float.h>
#include <math.h>
#include <linux/videodev2.h>
#include <memory.h>
#include <chrono>
#include <ctime>

#ifndef IN
#define IN
#endif

#ifndef OUT
#define OUT
#endif

#define NORI_CAMCTRL_API  __attribute__((visibility("default")))

#define NORI_IMAGECTRL_API  __attribute__((visibility("default")))

#define NORI_FIRMWARECTRL_API  __attribute__((visibility("default")))

typedef unsigned char       BYTE;
typedef unsigned short      WORD;
typedef unsigned long       DWORD;
typedef unsigned long ULONG;
typedef void *PVOID;
#define VOID void

#define DELETE_UNIT(_p_)     if((_p_) != NULL){delete    (_p_); (_p_) = NULL;}
#define DELETES_UNIT(_p_)    if((_p_) != NULL){delete[]  (_p_); (_p_) = NULL;}

#define MAX_PATH (0x100)

#define CHECK_RET(expr, ...) \
    do { \
        int _ret = (expr); \
        struct timespec ts; \
        clock_gettime(CLOCK_MONOTONIC, &ts); \
        double sec = ts.tv_sec + ts.tv_nsec / 1e9; \
        if (_ret != 0) { \
            if (sizeof(#__VA_ARGS__) > 1) { \
                fprintf(stderr, "\033[31m[ %10.6f] [ERROR] %s(%s) failed: ret=0x%x \033[33margs: ", \
                        sec, __func__, #expr, _ret); \
                fprintf(stderr, __VA_ARGS__); \
                fprintf(stderr, "\033[0m\n"); \
            } else { \
                fprintf(stderr, "\033[31m[ %10.6f] [ERROR] %s(%s) failed: ret=0x%x\033[0m\n", \
                        sec, __func__, #expr, _ret); \
            } \
        } else { \
            if (sizeof(#__VA_ARGS__) > 1) { \
                fprintf(stderr, "\033[32m[ %10.6f] [ OK  ] %s(%s) success: ret=0x%x \033[33margs: ", \
                        sec, __func__, #expr, _ret); \
                fprintf(stderr, __VA_ARGS__); \
                fprintf(stderr, "\033[0m\n"); \
            } else { \
                fprintf(stderr, "\033[32m[ %10.6f] [ OK  ] %s(%s) success: ret=0x%x\033[0m\n", \
                        sec, __func__, #expr, _ret); \
            } \
        } \
    } while (0)

#define NORI_DEBUG_PRINT(fmt, ...) \
    do { \
        struct timespec ts; \
        clock_gettime(CLOCK_MONOTONIC, &ts); \
        double sec = ts.tv_sec + ts.tv_nsec / 1e9; \
        fprintf(stderr, "\033[37m[ %10.6f] [DEBUG] " fmt "\033[0m", sec, ##__VA_ARGS__); \
    } while (0)

/**
 * @brief
 * @~chinese 媒体类型定义
 * @~english Media Reserved definition
 */
typedef enum _E_VIDEO_MEDIA_TYPE_
{
    /**
     * @brief
     * @~chinese MJPEG jpg 数据
     * @~english MJPEG jpg data
     */
    VIDEO_MEDIA_TYPE_MJPG = 0x47504a4d,

    /**
     * @brief
     * @~chinese YUYV YUV 数据
     * @~english YUYV YUV data
     */
    VIDEO_MEDIA_TYPE_YUYV = 0x56595559,
} E_VIDEO_MEDIA_TYPE;

/**
 * @brief
 * @~chinese 视频图像描述信息
 * @~english Video image description
 */

typedef struct _VIDEO_INFO_
{
    /**
     * @brief
     * @~chinese 视频格式
     * @~english Video format
     * @see _E_VIDEO_MEDIA_TYPE_
     */
    uint32_t u_Format;

    /**
     * @brief
     * @~chinese 视频图像宽度
     * @~english Video image width
     */
    uint32_t u_Width;

    /**
     * @brief
     * @~chinese 视频图像高度
     * @~english Video image height
     */
    uint32_t u_Height;

    /**
     * @brief
     * @~chinese 视频帧率
     * @~english Video frame rate
     */
    float f_Fps;
} VIDEO_INFO;

typedef struct timeval32 {
    int32_t tv_sec;
    int32_t tv_usec;
} timeval32;

/**
 * @brief
 * @~chinese 图像帧数据
 * @~english Frame buffer data
 */
typedef struct _FRAME_BUFFER_DATA_
{    
    /**
     * @brief
     * @~chinese 图像像素格式信息
     * @~english Pixel format info
     */
    VIDEO_INFO PixFormat;

    /**
     * @brief
     * @~chinese 图像采集时间戳
     * @~english Timestamp when frame was captured
     */
    struct timeval32 Frame_Time;

    /**
     * @brief
     * @~chinese 图像数据缓存指针
     * @~english Pointer to image data buffer
     */
    void* pBufAddr;

    /**
     * @brief
     * @~chinese 图像数据长度
     * @~english Length of image data in bytes
     */
    uint32_t buff_Length;

    /**
     * @brief
     * @~chinese 图像数据偏移
     * @~english Offset of image data in buffer
     */
    uint32_t buff_Offset;

    /**
     * @brief
     * @~chinese 图像数据索引
     * @~english Index of the frame buffer
     */
    uint32_t index;

    /**
     * @brief
     * @~chinese v4l2 缓存
     * @~english v4l2 buffer
     */
    struct v4l2_buffer buffer;
} __attribute__((packed, aligned(4))) FRAME_BUFFER_DATA;

/**
 * @brief
 * @~chinese 设备信息
 * @~english Device information
 */
typedef struct _DEVICE_INFO_
{
    unsigned short idVendor;            /**< @~chinese 供应商ID / @~english Vendor ID */
    unsigned short idProduct;           /**< @~chinese 产品ID / @~english Product ID */
    unsigned short bcdDevice;           /**< @~chinese 设备版本号 / @~english Device version number */

    unsigned char iManufacturer[MAX_PATH]; /**< @~chinese 制造商字符串 / @~english Manufacturer string */
    unsigned char iProduct[MAX_PATH];      /**< @~chinese 产品名称 / @~english Product name */
    unsigned char iSerialNumber[MAX_PATH]; /**< @~chinese 序列号 / @~english Serial number */

    unsigned short bcdUSB;                 /**< @~chinese USB版本 / @~english USB version */
    char deviceID[MAX_PATH];               /**< @~chinese 设备ID / @~english Device ID */
    char device[MAX_PATH];                 /**< @~chinese 设备路径 / @~english Device path */
    uint32_t busnum;                       /**< @~chinese 总线号 / @~english Bus number */
    uint32_t devnum;                       /**< @~chinese 设备号 / @~english Device number */
    char location[MAX_PATH];               /**< @~chinese 设备位置 / @~english Device location */

    uint32_t Reserved;                     /**< @~chinese 保留 / @~english Reserved */
} DEVICE_INFO;

/**
 * @brief
 * @~chinese SDK及设备版本信息
 * @~english SDK and device version info
 */
typedef struct _VERSION_INFO_
{
    unsigned char SDKVersion[MAX_PATH];    /**< @~chinese SDK版本 / @~english SDK version */
    unsigned char DeviceType[MAX_PATH];    /**< @~chinese 设备类型 / @~english Device type */
    unsigned char ISPVersion[MAX_PATH];    /**< @~chinese ISP版本 / @~english ISP version */
    unsigned char FPGAVersion[MAX_PATH];   /**< @~chinese FPGA版本 / @~english FPGA version */
} VERSION_INFO;

/**
 * @brief
 * @~chinese 帧回调函数类型定义
 * @~english Frame callback function type
 */
typedef uint32_t(*PCall_Back_Frame)(PVOID, FRAME_BUFFER_DATA*, PVOID);

/**
 * @brief
 * @~chinese 传感器位深
 * @~english Sensor bit depth
 */
typedef enum _E_SENSOR_DEPTH_
{
    BAYER_8bit = 0,   /**< @~chinese 8位 / @~english 8-bit */
    BAYER_10bit,      /**< @~chinese 10位 / @~english 10-bit */
    BAYER_12bit,      /**< @~chinese 12位 / @~english 12-bit */
    BAYER_14bit,      /**< @~chinese 14位 / @~english 14-bit */
    BAYER_16bit,      /**< @~chinese 16位 / @~english 16-bit */
    BAYER_24bit,      /**< @~chinese 24位 / @~english 24-bit */
} E_SENSOR_DEPTH;

/**
 * @brief
 * @~chinese RAW模式下颜色排列方式
 * @~english Color arrangement in RAW mode
 */
typedef enum _E_SENSOR_COLOR_CODING_
{
    BAYER_GRBG = 0, /**< @~chinese GRBG / @~english GRBG */
    BAYER_RGGB,     /**< @~chinese RGGB / @~english RGGB */
    BAYER_BGGR,     /**< @~chinese BGGR / @~english BGGR */
    BAYER_GBRG,     /**< @~chinese GBRG / @~english GBRG */
} E_SENSOR_COLOR_CODING;

/**
 * @brief
 * @~chinese GPIO编号
 * @~english GPIO ID
 */
typedef enum _E_GPIO_ID_
{
	//Gen56
    GPIO_0 = 0, /**< @~chinese GPIO0 / @~english GPIO0 */
    GPIO_1 = 1, /**< @~chinese GPIO1 / @~english GPIO1 */

	//Gen60
    GPIO_PIN7 = 1, /**< @~chinese PIN7 / @~english PIN7 */
    GPIO_PIN6 = 2, /**< @~chinese PIN6 / @~english PIN6 */
    GPIO_PIN34 = 4, /**< @~chinese PIN34 / @~english PIN34 */
} E_GPIO_ID;

/**
 * @brief
 * @~chinese 视频类型
 * @~english Video type
 */
typedef enum _E_VIDEO_TYPE_
{
    RBG_TYPE = 0, /**< @~chinese 彩色 / @~english RGB color */
    MONO_TYPE,    /**< @~chinese 单色 / @~english Monochrome */
    RAW_TYPE,     /**< @~chinese RAW / @~english RAW */
} E_VIDEO_TYPE;

/**
 * @brief
 * @~chinese 触发模式
 * @~english Trigger mode
 */
typedef enum _E_TRIGGER_MODE_
{
    NON_TRIIGER_MODE = 0,     /**< @~chinese 非触发 / @~english Non-trigger */
    SOFTWARE_TRIIGER_MODE,    /**< @~chinese 软件触发 / @~english Software trigger */
    HARDWARE_TRIGGER_MODE,    /**< @~chinese 硬件触发 / @~english Hardware trigger */
    COMMAND_TRIGGER_MODE,     /**< @~chinese 命令触发 / @~english Command trigger */
} E_TRIGGER_MODE;

/**
 * @brief
 * @~chinese 图像镜像翻转模式
 * @~english Sensor mirror/flip mode
 */
typedef enum _E_SENSOR_MIRROR_FLIP_
{
    Normal = 0,         /**< @~chinese 正常 / @~english Normal */
    MIRROR_EN,          /**< @~chinese 镜像 / @~english Mirror */
    FLIP_EN,            /**< @~chinese 翻转 / @~english Flip */
    MIRROR_FLIP_EN,     /**< @~chinese 镜像+翻转 / @~english Mirror+Flip */
} E_SENSOR_MIRROR_FLIP;

/**
 * @brief
 * @~chinese XY方向偏移
 * @~english XY offset
 */
typedef struct _XY_OFFSET_
{
    uint32_t width_offset;   /**< @~chinese 宽度偏移 / @~english Width offset */
    uint32_t height_offset;  /**< @~chinese 高度偏移 / @~english Height offset */
} XY_OFFSET;

/**
 * @brief
 * @~chinese 白平衡RGB增益
 * @~english White balance RGB gain
 */
typedef struct _WB_RGB_GAIN_
{
    uint32_t gain_r; /**< @~chinese R通道增益 / @~english Red channel gain */
    uint32_t gain_g; /**< @~chinese G通道增益 / @~english Green channel gain */
    uint32_t gain_b; /**< @~chinese B通道增益 / @~english Blue channel gain */
} WB_RGB_GAIN;

/**
 * @brief
 * @~chinese RAW传感器信息
 * @~english RAW sensor info
 */
typedef struct _SENSOR_RAW_INFO_
{
    E_SENSOR_DEPTH raw_depth;                /**< @~chinese RAW位深 / @~english RAW bit depth */
    E_SENSOR_COLOR_CODING raw_color_coding; /**< @~chinese RAW颜色排列 / @~english RAW color coding */
} SENSOR_RAW_INFO;

/**
 * @brief
 * @~chinese GPIO模式
 * @~english GPIO mode
 */
typedef enum _E_GPIO_MODE_
{
    OUTPUT_MODE = 0x04, /**< @~chinese 输出模式 / @~english Output mode */
    INPUT_MODE  = 0x00, /**< @~chinese 输入模式 / @~english Input mode */
} E_GPIO_MODE;

/**
 * @brief
 * @~chinese GPIO电平
 * @~english GPIO level
 */
typedef enum _E_GPIO_LEVEL_
{
    LOW  = 0x00, /**< @~chinese 低电平 / @~english Low level */
    HIGH = 0x08, /**< @~chinese 高电平 / @~english High level */
} E_GPIO_LEVEL;

/**
 * @brief
 * @~chinese GPIO控制结构
 * @~english GPIO control
 */
typedef struct _IO_CONTORL_
{
    E_GPIO_MODE gpio_mode;   /**< @~chinese GPIO模式 / @~english GPIO mode */
    E_GPIO_LEVEL gpio_level; /**< @~chinese GPIO电平 / @~english GPIO level */
} IO_CONTORL;

/**
 * @brief
 * @~chinese 频率与比例
 * @~english Frequency ratio
 */
typedef struct _FREQUENCY_RATIO_
{
    uint32_t frequency; /**< @~chinese 频率 / @~english Frequency */
    uint32_t ratio;     /**< @~chinese 比例 / @~english Ratio */
} FREQUENCY_RATIO;

#endif // NORI_PUBLIC_H
