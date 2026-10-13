#ifndef MOSAS_DOMAIN_IMU_TIME_MODEL_HPP
#define MOSAS_DOMAIN_IMU_TIME_MODEL_HPP

#include <mosas/domain/time.hpp>

#include <cstdint>

namespace mosas::domain {

/**
 * @brief 用串口首字节接收时间和测得延迟估算 IMU 样本时刻。
 * @param first_byte_receive_monotonic_ns 串口首字节到达主机的单调时钟纳秒值。
 * @param measurement_latency_ns 测量时刻相对接收时刻的补偿延迟，单位 ns；必须来自标定或配置。
 * @return 标记为 estimated 的单调时间；设备原始时间和不确定度保持未知。
 * @note IMU 协议不含设备时间戳或样本序号，此结果不得解释为 MCU 精确采样时间。
 */
SensorTime estimate_imu_sample_time(
    std::int64_t first_byte_receive_monotonic_ns,
    std::int64_t measurement_latency_ns) noexcept;

}  // namespace mosas::domain

#endif  // MOSAS_DOMAIN_IMU_TIME_MODEL_HPP
