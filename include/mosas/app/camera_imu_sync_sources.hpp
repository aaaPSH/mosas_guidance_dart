#ifndef MOSAS_APP_CAMERA_IMU_SYNC_SOURCES_HPP
#define MOSAS_APP_CAMERA_IMU_SYNC_SOURCES_HPP

#include <mosas/app/camera_imu_sync_session.hpp>

#include <memory>

namespace mosas::app {

/** @brief 持有原输入路径及不能执行设备 I/O 的命令接收端。 */
struct CameraImuSyncPorts {
    std::unique_ptr<runtime::CameraSource> camera;
    std::unique_ptr<runtime::ImuSource> imu;
    std::unique_ptr<runtime::CommandSink> command;
};

/**
 * @brief 装饰既有数据源以旁路记录，不重复打开设备或修改原数据。
 * @param camera 唯一相机源；原样转发预处理及生命周期调用。
 * @param imu 唯一的全速 IMU 源；只复制已解码的原始样本，不增加第二个消费者。
 * @param session 已启动的记录会话，由两个装饰器共同保持其生命周期。
 * @return 拥有型输入装饰器及内存命令接收端，不持有串口发送器。
 * @throws std::invalid_argument 任一依赖为空时抛出。
 */
CameraImuSyncPorts make_camera_imu_sync_ports(
    std::unique_ptr<runtime::CameraSource> camera,
    std::unique_ptr<runtime::ImuSource> imu,
    std::shared_ptr<CameraImuSyncSession> session);

}  // namespace mosas::app
#endif
