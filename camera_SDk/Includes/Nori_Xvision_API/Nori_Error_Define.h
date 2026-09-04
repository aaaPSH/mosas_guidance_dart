#ifndef NORI_ERROR_DEFINE_H
#define NORI_ERROR_DEFINE_H

/********************************************************************/
///  \name CN:正确码定义   EN:Success Code Definitions
///  @{
#define NORI_OK 0x0000  /**< @~chinese 成功，无错误 / @~english Success, no error */
/// @}

/********************************************************************/
///  \name CN:错误码定义   EN:Error Code Definitions
///  @{
#define NORI_E_INIT                    0x8000  /**< @~chinese SDK 未初始化 / @~english SDK not initialized */
#define NORI_E_REPEAT                  0x8001  /**< @~chinese 重复调用 / @~english Duplicate call */
#define NORI_E_UNINT                   0x8002  /**< @~chinese 资源已释放 / @~english Resource released */
#define NORI_E_UNKOWN                  0x8003  /**< @~chinese 未知错误 / @~english Unknown error */

#define NORI_E_HANDEL                  0x8004  /**< @~chinese 设备不存在 / @~english Device not found */
#define NORI_E_CONTROL                  0x8005  /**< @~chinese 操作失败 / @~english Operation failed */
#define NORI_E_CONTROL_NOT_SUPPORT      0x8006  /**< @~chinese 操作不支持 / @~english Operation not supported */

#define NORI_E_NOBUFF                   0x8007  /**< @~chinese 没有可输出的缓存 / @~english No buffer available */
#define NORI_E_MEIA_NOT_SUPPORT         0x8008  /**< @~chinese 当前媒体类型不支持 BYPASS ISP / @~english Current media Reserved does not support BYPASS ISP */
#define NORI_E_DEVICE_NOT_OPEN          0x8009  /**< @~chinese 设备未打开 / @~english Device not opened */
#define NORI_E_START_DEVICE             0x800A  /**< @~chinese 开始取流失败 / @~english Device start failed */
#define NORI_E_STOP_DEVICE              0x800B  /**< @~chinese 停止取流失败 / @~english Device stop failed */
#define NORI_STOP_GRAB                  0x800C  /**< @~chinese 已停止取流 / @~english Grab stopped */

#define NORI_E_UPG_EXIT                 0x8100  /**< @~chinese 异常退出 / @~english Abnormal exit */
#define NORI_E_UPG_FW_NOT_SUPPORT_DEVICE 0x8101 /**< @~chinese 设备与固件不匹配 / @~english Device and firmware mismatch */
#define NORI_E_UPG_TIME_OUT             0x8102  /**< @~chinese 烧录超时 / @~english Flashing timeout */
#define NORI_E_UPG_UNKNOWN_FLASH        0x8103  /**< @~chinese FLASH型号未知 / @~english Unknown FLASH model */
#define NORI_E_UPG_NOT_CLOSE_PREVIEW    0x8104  /**< @~chinese 未关闭预览 / @~english Preview not closed */

#define NORI_E_DEVICE_CHECK				0x8200 ///< \~chinese 握手校验失败		\~english Device Check Fail

#define NORI_E_UNKOWN_DEVICE            0x9000  /**< @~chinese 未知设备 / @~english Unknown device */
/// @}

/********************************************************************/
///  \name CN:设备类型定义   EN:Device Type Definitions
///  @{
#define NORI_UNKOWN_DEVICE              0x0000  /**< @~chinese 未知设备 / @~english Unknown device */
#define NORI_USB_DEVICE                  0x0001  /**< @~chinese USB设备 / @~english USB device */
#define NORI_UBOOT_DEVICE                0x0100  /**< @~chinese UBOOT设备 / @~english UBOOT device */
/// @}

#endif // NORI_ERROR_DEFINE_H
