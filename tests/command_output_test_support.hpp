#ifndef MOSAS_TEST_COMMAND_OUTPUT_SUPPORT_HPP
#define MOSAS_TEST_COMMAND_OUTPUT_SUPPORT_HPP
#include <mosas/guidance/command_output_mapper.hpp>
#include <mosas/runtime/runtime_supervisor.hpp>
#include <mosas/runtime/clock.hpp>
#include "body_vector_test_support.hpp"
namespace mosas::test {
/** @return 明确时间轴的测试传感器时间，不冒用真实设备标定结果。 */
inline domain::SensorTime command_test_time(std::int64_t ns, domain::ClockDomain domain = domain::ClockDomain::monotonic) {
    if (domain == domain::ClockDomain::simulation) return domain::make_simulation_time(ns);
    return {ns, ns, 0, domain::ClockDomain::monotonic, domain::TimestampSource::hardware_sync,
            domain::TimestampQuality::hardware_synchronized};
}
/** @return 已发射、同一时刻且同一代际的完整测试状态。 */
inline domain::FlightStateSnapshot command_test_state(std::int64_t ns, std::uint64_t generation = 0,
    double roll = 0, domain::ClockDomain domain = domain::ClockDomain::monotonic) {
    domain::FlightStateSnapshot state;
    state.time = command_test_time(ns, domain);
    state.attitude = vector_test_attitude(0, roll);
    state.attitude.time = state.time;
    state.launch.phase = domain::LaunchPhase::launched;
    state.launch.generation = generation;
    state.speed_mps.value = 50;
    return state;
}
/** @return 带真实映射器的测试依赖；其余端口由用例提供。 */
inline runtime::RuntimeSupervisorDependencies mapped_dependencies() {
    runtime::RuntimeSupervisorDependencies deps;
    deps.command_mapper = std::make_unique<guidance::CommandOutputMapper>();
    return deps;
}
/** @brief 为独立输出端测试绑定完整的真实映射依赖。
 * @param sink 输出端口。 @param state 最新状态读取器。 @param clock 当前时钟读取器。
 * @param config 物理限制。 @param domain 运行时间轴。 @return 绑定成功时为 true。
 */
inline bool bind_test_output(runtime::CommandSink& sink,
    std::function<std::optional<domain::FlightStateSnapshot>()> state,
    std::function<std::int64_t()> clock, guidance::CommandMappingConfig config = {},
    domain::ClockDomain domain = domain::ClockDomain::monotonic) {
    runtime::CommandOutputBinding binding;
    binding.mapper = std::make_unique<guidance::CommandOutputMapper>(config);
    binding.latest_state = std::move(state); binding.now_ns = std::move(clock);
    binding.output_enabled = [] { return true; };
    binding.clock_domain = domain; binding.maximum_attitude_age_ns = 50'000'000;
    return sink.bind_output(std::move(binding));
}
}
#endif
