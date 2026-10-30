#pragma once
#include <dart_sim_transport/protocol.hpp>
#include <mosas/domain/algorithm_results.hpp>
#include <mosas/guidance/png_calculation_trace.hpp>
#include <mosas/runtime/runtime_interfaces.hpp>
#include <array>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace mosas::adapters {
/** @brief Explicit CLI-only audit destination and trial identity, not algorithm settings. */
struct SimulationAuditOptions {std::filesystem::path directory;std::string trial_id;};
/** @brief Parse only optional simulation CLI arguments; never extends AppConfig.
 * @param arguments Options after the original simulation.conf argument.
 * @return Disabled for no options; otherwise both absolute directory and safe ID.
 * @throws std::invalid_argument For unknown/duplicate/incomplete options.
 */
std::optional<SimulationAuditOptions> parse_simulation_audit_options(const std::vector<std::string>& arguments);
/** @brief Copies of already parsed/validated values, not an independent configuration source. */
struct SimulationAuditMetadata {
    std::string source_config;
    std::uint32_t width=0,height=0;
    std::array<double,4> intrinsics{};
    double camera_hz=0,imu_hz=0,launch_speed_mps=0,cap_g=0;
    /** @brief Ny, Nz, sigma_a, sigma_a_pitch, sigma_a_yaw, sigma_q, initial_covariance,
     * maximum_gap_seconds, camera_lever_arm_x, estimated_target_distance, enable_filter (0/1).
     */
    std::array<double,11> parameters{};
    double maximum_command_acceleration=100;
};
enum class AuditLifecycle {ready,fault,stopped};
enum class AuditRejection : std::uint32_t {wrong_generation=1,invalid_time=2};
struct SimulationAuditDiagnostics {
    bool healthy=false;
    std::uint64_t accepted=0,written=0,dropped=0,write_failures=0;
    std::string error;
};
/** @brief Session-specific, bounded background diagnostic sink with no truth input.
 * Uses one existing SPSC ring per producer role. Role misuse is rejected rather
 * than racing queue storage. Callbacks only copy fixed-size values; no file IO.
 * Raw PNG generation is unknown at this port and is explicitly serialized null.
 */
class SimulationAudit {
public:
    /** @param options Explicit opt-in output/identity. @param session_id Real DSIM session.
     * @param metadata Copies of the original loaded AppConfig/handshake.
     * @param capacity Fixed queue capacity per role, allocated only at setup.
     */
    SimulationAudit(const SimulationAuditOptions& options,std::uint64_t session_id,
                    const SimulationAuditMetadata& metadata,std::size_t capacity=2048);
    ~SimulationAudit();
    SimulationAudit(const SimulationAudit&)=delete;
    SimulationAudit& operator=(const SimulationAudit&)=delete;
    /** @brief Enqueue a lifecycle-host observation.
     * @param state Ready/fault/stopped condition. @param time_ns Actual simulation time.
     * @return False when evidence cannot be retained; no disk IO occurs here.
     */
    bool record_lifecycle(AuditLifecycle state,std::int64_t time_ns)noexcept;
    /** @brief Observe an unchanged original detector result.
     * @param frame Real image sequence. @param time Original acquisition time.
     * @param generation Optional processed-watermark generation, not retrospective frame truth.
     * @param found Original valid flag. @param reason Original observation reason.
     * @return Whether the record was retained.
     */
    bool record_vision(std::uint64_t frame,const domain::SensorTime& time,
                       std::optional<std::uint64_t> generation,bool found,std::uint32_t reason)noexcept;
    /** @brief Copy existing PNG intermediate values, without recalculating guidance.
     * @param input Original source identifiers/time. @param trace Original calculation snapshot.
     * @return Whether retained; the raw port has no generation and records it as null.
     */
    bool record_png(const domain::GuidanceInput& input,const guidance::PngCalculationTrace& trace)noexcept;
    /** @brief 记录实际被提交的弹道请求，不制造未检出帧的控制事件。
     * @param command 原始弹道请求及来源。 @return 事件进入有界审计队列时为 true。
     */
    bool record_request(const runtime::GuidanceCommand& command)noexcept;
    /** @brief Observe the original transport enqueue result; sequence is not known yet.
     * @param emission 本次真实输出映射与来源。 @param wire Exact offered DSIM payload.
     * @param accepted Enqueue return value, never a physical acknowledgement.
     * @return Whether retained; duplicate successful valid frame sources fail the audit.
     */
    bool record_submit(const domain::CommandEmissionSnapshot& emission,const dart_sim_transport::Overload& wire,bool accepted)noexcept;
    /** @brief Record a client-side rejected attempt, never a transmitted wire message.
     * @param command Attempted source command. @param reason Client guard disposition.
     * @return Whether retained; this cannot be interpreted as a physical acknowledgement.
     */
    bool record_rejection(const runtime::GuidanceCommand& command,AuditRejection reason)noexcept;
    /** @brief Lock-free health query; false must stop the audited runtime. */
    bool healthy()const noexcept;
    /** @brief Fence producers, drain accepted records, flush and join; idempotent. */
    void stop()noexcept;
    /** @return Writer counters/errors; healthy alone does not certify a trial or recording. */
    SimulationAuditDiagnostics diagnostics()const;
private:
    struct Impl;std::unique_ptr<Impl> impl_;
};
} // namespace mosas::adapters
