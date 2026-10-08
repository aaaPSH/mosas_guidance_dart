#ifndef MOSAS_RUNTIME_RUNTIME_SUPERVISOR_HPP
#define MOSAS_RUNTIME_RUNTIME_SUPERVISOR_HPP

#include <mosas/domain/algorithm_results.hpp>
#include <mosas/ports/clock.hpp>
#include <mosas/ports/flight_state_estimator.hpp>
#include <mosas/ports/guidance_law.hpp>
#include <mosas/ports/los_estimator.hpp>
#include <mosas/ports/los_rate_filter.hpp>
#include <mosas/ports/vision_detector.hpp>
#include <mosas/runtime/output_ports.hpp>
#include <mosas/runtime/runtime_interfaces.hpp>
#include <mosas/runtime/runtime_types.hpp>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

namespace mosas::runtime {

struct RuntimeStatistics {
    double capture_fps = 0.0;
    double processing_fps = 0.0;
    double output_fps = 0.0;
};

struct RuntimeTimingStage {
    std::uint64_t samples = 0;
    double minimum_ms = 0.0;
    double average_ms = 0.0;
    double p50_ms = 0.0;
    double p95_ms = 0.0;
    double p99_ms = 0.0;
    double maximum_ms = 0.0;
};

struct RuntimeTiming {
    RuntimeTimingStage capture;
    RuntimeTimingStage capture_queue_wait;
    RuntimeTimingStage capture_queue_delay;
    RuntimeTimingStage prepare;
    RuntimeTimingStage vision;
    RuntimeTimingStage line_of_sight;
    RuntimeTimingStage guidance;
    RuntimeTimingStage command;
    RuntimeTimingStage processing_total;
    RuntimeTimingStage output_queue_wait;
    RuntimeTimingStage output_queue_delay;
    RuntimeTimingStage output;
    std::uint64_t capture_queue_dropped = 0;
    std::uint64_t output_queue_dropped = 0;
};

struct RuntimeSupervisorConfig {
    double launch_speed_mps = 100.0;
    std::size_t history_capacity = 256;
    TimestampNs max_imu_age_ns = 50000000;
    TimestampNs max_camera_future_ns = 50000000;
    TimestampNs max_camera_age_ns = 2000000000;
    domain::CameraIntrinsics camera_intrinsics{1.0, 1.0, 0.0, 0.0};
    domain::Rotation3<domain::DartFrameTag, domain::CameraFrameTag>
        dart_from_camera{};
    CameraCaptureConfig camera_capture{};
    std::size_t capture_queue_capacity = 1;
    std::size_t output_queue_capacity = 1;
    std::size_t info_log_queue_capacity = 256;
};

struct RuntimeSupervisorDependencies {
    std::unique_ptr<ports::IVisionDetector> vision_detector;
    std::unique_ptr<ports::IFlightStateEstimator> flight_state_estimator;
    std::unique_ptr<ports::ILosEstimator> los_estimator;
    std::unique_ptr<ports::ILosRateFilter> los_rate_filter;
    std::unique_ptr<ports::IGuidanceLaw> guidance_law;
    std::unique_ptr<ports::IMonotonicClock> monotonic_clock;
};

class RuntimeSupervisor {
public:
    RuntimeSupervisor(std::unique_ptr<ImuSource> imu_source,
                      std::unique_ptr<CameraSource> camera_source,
                      std::unique_ptr<CommandSink> command_sink,
                      const RuntimeSupervisorConfig& config,
                      RuntimeSupervisorDependencies dependencies,
                      RuntimeOutputPorts output_ports);
    ~RuntimeSupervisor();

    RuntimeSupervisor(const RuntimeSupervisor&) = delete;
    RuntimeSupervisor& operator=(const RuntimeSupervisor&) = delete;

    bool start();
    void cancel() noexcept;
    void stop() noexcept;

    bool running() const noexcept;
    bool faulted() const noexcept;
    std::string last_error() const;
    std::optional<ImuStateSnapshot> latest_state() const;
    RuntimeStatistics statistics() const;
    RuntimeTiming timing() const;
    RuntimeOutputDiagnostics output_diagnostics() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace mosas::runtime

#endif  // MOSAS_RUNTIME_RUNTIME_SUPERVISOR_HPP
