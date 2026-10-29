#ifndef MOSAS_DOMAIN_COMMAND_OUTPUT_HPP
#define MOSAS_DOMAIN_COMMAND_OUTPUT_HPP
#include <mosas/domain/contracts.hpp>
#include <optional>

namespace mosas::domain {
/** @brief 一次输出的请求来源和最新完整状态；绝不持有历史 body 控制量。 */
struct CommandOutputContext {
    std::uint64_t source_frame_id = 0, generation = 0, active_generation = 0;
    SensorTime source_time{};
    bool request_valid = false, launched = false, target_found = false;
    GuidanceInvalidReason invalid_reason = GuidanceInvalidReason::algorithm_rejected;
    std::int64_t emit_time_ns = -1, maximum_attitude_age_ns = 0;
    ClockDomain clock_domain = ClockDomain::unknown;
    bool expired = false, output_enabled = false;
    std::optional<FlightStateSnapshot> latest_state;
};
/** @brief 已映射的镖体系执行命令；只有此类型允许进入设备编码器。 */
struct MappedGuidanceCommand {
    std::uint64_t source_frame_id = 0, generation = 0;
    SensorTime source_time{};
    bool valid = false, launched = false, target_found = false;
    GuidanceInvalidReason invalid_reason = GuidanceInvalidReason::algorithm_rejected;
    DartOverloadG overload_g{};
};
/** @brief 同一映射尝试各阶段的真实诊断；弹道量与 body 量不可混用。 */
struct CommandMappingTrace {
    bool valid = false, overload_limited = false, acceleration_limited = false;
    AttitudeState attitude{};
    std::int64_t emit_time_ns = -1;
    BallisticOverloadG raw_navigation_overload{}, after_overload_limit{};
    AccelerationBallisticMps2 ballistic_acceleration{};
    DartOverloadG body_overload{};
    AccelerationDartMps2 body_acceleration{};
    Quantity<GravityG> command_overload{};
    Quantity<Radian> command_phase{};
};
/** @brief 命令及其同一姿态、同一时刻的映射诊断。 */
struct CommandMappingResult {
    MappedGuidanceCommand command{};
    CommandMappingTrace trace{};
};
/** @brief 本地输出处置，不代表下位机或仿真对端的物理执行确认。 */
enum class CommandEmissionDisposition {
    not_attempted, mapping_rejected, log_accepted, serial_written, transport_accepted, io_rejected
};
/** @brief 带单次运行内递增编号的实际输出尝试快照。 */
struct CommandEmissionSnapshot {
    std::uint64_t attempt_id = 0;
    CommandMappingResult mapping{};
    CommandEmissionDisposition disposition = CommandEmissionDisposition::not_attempted;
};
}
#endif
