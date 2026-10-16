#pragma once
#include <mosas/adapters/simulation_session.hpp>
#include <mosas/ports/clock.hpp>
#include <atomic>

namespace mosas::adapters {
/** @brief Session simulation time, never the host receive clock. */
class SimulationClock final:public ports::IMonotonicClock {
public:
    explicit SimulationClock(std::shared_ptr<SimulationSession> session):session_(std::move(session)){}
    std::int64_t now_ns()const noexcept override{return session_->now_ns();}
private:std::shared_ptr<SimulationSession> session_;
};
/** @brief Cancelable FIFO of coordinate-converted, paired IMU samples. */
class SimulationImuSource final:public runtime::ImuSource {
public:
    explicit SimulationImuSource(std::shared_ptr<SimulationSession> session):session_(std::move(session)){}
    runtime::SourceResult start()override;
    bool try_pop(runtime::ImuAngularVelocitySample* sample)noexcept override;
    bool wait_for_data()noexcept override;
    void stop()noexcept override;
    void cancel()noexcept override;
    runtime::ImuSourceDiagnostics diagnostics()const noexcept override;
private:std::shared_ptr<SimulationSession> session_;
};
/** @brief Latest owning RGB capture; prepare converts RGB to BGR without geometry changes.
 * Capture waits for a valid processed-IMU watermark, not arrival/dequeue counts. A host
 * timeout retains the pending frame; only advancing simulation time spends sync budget.
 * Uses CameraSource's neutral capture_fps(): Runtime measures accepted host-frame
 * throughput rather than reporting the nominal handshake rate as an observation.
 */
class SimulationCameraSource final:public runtime::CameraSource {
public:
    explicit SimulationCameraSource(std::shared_ptr<SimulationSession> session,std::int64_t sync_budget_ns=10'000'000);
    runtime::SourceResult configure(const runtime::CameraCaptureConfig& config)override;
    runtime::SourceResult capture(runtime::CameraFrame* frame)override;
    runtime::SourceResult prepare(runtime::CameraFrame* frame)override;
    void cancel()noexcept override;
    void stop()noexcept override;
    std::uint64_t sync_timeouts()const noexcept{return sync_timeouts_.load();}
private:
    std::shared_ptr<SimulationSession> session_;
    const std::int64_t budget_;
    std::optional<dart_sim_transport::Message> pending_;
    std::atomic<std::uint64_t> sync_timeouts_{0};
};
} // namespace mosas::adapters
