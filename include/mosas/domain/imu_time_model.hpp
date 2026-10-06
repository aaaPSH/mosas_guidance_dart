#ifndef MOSAS_DOMAIN_IMU_TIME_MODEL_HPP
#define MOSAS_DOMAIN_IMU_TIME_MODEL_HPP

#include <mosas/domain/time.hpp>

#include <cstdint>

namespace mosas::domain {

SensorTime estimate_imu_sample_time(
    std::int64_t first_byte_receive_monotonic_ns,
    std::int64_t measurement_latency_ns) noexcept;

}  // namespace mosas::domain

#endif  // MOSAS_DOMAIN_IMU_TIME_MODEL_HPP
