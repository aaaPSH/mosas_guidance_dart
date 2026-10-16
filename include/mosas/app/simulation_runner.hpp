#pragma once
#include <mosas/app/simulation_config.hpp>
#include <mosas/adapters/simulation_sources.hpp>
#include <mosas/adapters/simulation_pre_acquisition.hpp>
#include <mosas/adapters/simulation_audit.hpp>
#include <set>

namespace mosas::app {
/** @brief Host-thread session lifecycle. Uses the original algorithm factory/runtime;
 * each accepted socket session gets a fresh supervisor. Disconnected sessions cannot resume.
 */
class SimulationRunner {
public:
    using Log=std::function<void(const std::string&)>;
    /** @param config Original wrapper/AppConfig. @param log Lifecycle reporter.
     * @param audit_options Explicit optional CLI diagnostics; disabled by default.
     */
    SimulationRunner(SimulationConfig config,Log log,
        std::optional<adapters::SimulationAuditOptions> audit_options = {});
    ~SimulationRunner();
    /** @brief Advance connection/lifecycle work (up to 25 ms host handshake wait). */
    void step();
    /** @brief Stop/join supervisor before clearing its watermark, then join transport. */
    void stop();
    bool running()const noexcept;
    std::uint64_t session_id()const noexcept;
    const std::string& last_error()const noexcept{return error_;}
    /** @brief Current pipeline timings, or empty statistics without a supervisor. @return A thread-safe snapshot. */
    runtime::RuntimeTiming timing() const;
    /** @brief Current independent sink diagnostics; query from the lifecycle host thread. @return Empty diagnostics when stopped. */
    runtime::RuntimeOutputDiagnostics output_diagnostics() const;
private:
    void finish(std::string error);
    void report(const std::string& message);
    SimulationConfig config_;Log log_;
    std::optional<adapters::SimulationAuditOptions> audit_options_;
    std::shared_ptr<adapters::SimulationAudit> audit_;
    std::shared_ptr<adapters::SimulationSession> session_;
    std::unique_ptr<runtime::RuntimeSupervisor> supervisor_;
    std::unique_ptr<adapters::SimulationPreAcquisitionPolicy> pre_acquisition_;
    adapters::SimulationCameraSource* camera_=nullptr;
    std::set<std::uint64_t> used_;
    std::chrono::steady_clock::time_point retry_{};
    std::string error_;bool request_reported_=false,launch_reported_=false,release_reported_=false;
};
} // namespace mosas::app
