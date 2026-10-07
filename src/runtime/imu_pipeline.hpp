#ifndef MOSAS_RUNTIME_IMU_PIPELINE_HPP
#define MOSAS_RUNTIME_IMU_PIPELINE_HPP

#include <mosas/ports/flight_state_estimator.hpp>
#include <mosas/runtime/attitude_history.hpp>
#include <mosas/runtime/imu_angular_velocity_history.hpp>
#include <mosas/runtime/runtime_interfaces.hpp>

#include <cstddef>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

namespace mosas::runtime {

class LaunchGenerationCoordinator;

struct ImuPipelineCallbacks {
    std::function<bool()> stop_requested;
    std::function<bool(const domain::ImuKinematicSample&)> try_record_sample;
    std::function<void(std::string)> report_fault;
};

class ImuPipeline final {
public:
    ImuPipeline(std::unique_ptr<ImuSource> source,
                std::unique_ptr<ports::IFlightStateEstimator> estimator,
                std::size_t history_capacity,
                LaunchGenerationCoordinator& generation_coordinator,
                ImuPipelineCallbacks callbacks);
    ~ImuPipeline();

    ImuPipeline(const ImuPipeline&) = delete;
    ImuPipeline& operator=(const ImuPipeline&) = delete;

    bool valid() const noexcept;
    SourceResult start_source();
    void reset() noexcept;
    void start_worker();
    void cancel_source() noexcept;
    void join_worker() noexcept;
    void stop_source() noexcept;

    std::optional<ImuStateSnapshot> latest_state() const;
    std::optional<ImuAngularVelocityLookup> angular_velocity_at(
        TimestampNs timestamp_ns) const noexcept;
    std::optional<AttitudeLookup> attitude_at(
        TimestampNs timestamp_ns) const noexcept;
    std::size_t angular_velocity_history_size() const noexcept;
    ImuSourceDiagnostics diagnostics() const noexcept;

private:
    void worker_loop() noexcept;
    void report_fault(std::string message) noexcept;

    std::unique_ptr<ImuSource> source_;
    std::unique_ptr<ports::IFlightStateEstimator> estimator_;
    LaunchGenerationCoordinator& generation_coordinator_;
    ImuPipelineCallbacks callbacks_;
    ImuAngularVelocityHistory angular_velocity_history_;
    AttitudeHistory attitude_history_;
    mutable std::mutex state_mutex_;
    std::optional<ImuStateSnapshot> latest_state_;
    std::thread worker_;
};

}  // namespace mosas::runtime

#endif  // MOSAS_RUNTIME_IMU_PIPELINE_HPP
