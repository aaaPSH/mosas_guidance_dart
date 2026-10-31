#ifndef MOSAS_RUNTIME_COMMAND_OUTPUT_HPP
#define MOSAS_RUNTIME_COMMAND_OUTPUT_HPP
#include <mosas/ports/command_output_mapper.hpp>
#include <atomic>
#include <functional>
#include <memory>
#include <mutex>

namespace mosas::runtime {
/** @brief 输出端必需依赖；仅在启动前绑定，状态源生命周期覆盖输出线程。 */
struct CommandOutputBinding {
    std::unique_ptr<ports::ICommandOutputMapper> mapper;
    std::function<std::optional<domain::FlightStateSnapshot>()> latest_state;
    std::function<std::int64_t()> now_ns;
    std::function<bool()> output_enabled;
    domain::ClockDomain clock_domain = domain::ClockDomain::unknown;
    std::int64_t maximum_attitude_age_ns = 0;
};
/** @brief 所有输出适配器共享的上下文采集与诊断；不执行 I/O 或保存重发请求。 */
class CommandOutputProcessor {
public:
    /** @param binding 完整映射依赖；运行中不得重新绑定。 @return 依赖完整合法时为 true。 */
    bool bind(CommandOutputBinding binding);
    /** @return 已绑定完整依赖时为 true。 */
    bool configured() const noexcept;
    /** @brief 新运行开始时清空映射器触发状态、快照及输出尝试编号，保留绑定。
     * @note 调用方须保证旧输出已结束且新输出尚未开始；允许重复使用同一个 generation。
     */
    void reset() noexcept;
    /** @brief 切代时清空映射器触发状态及快照；不重用同一次运行的尝试编号。
     * @note 必须在输出端发送屏障内调用，排除 prepare()/complete() 在途尝试。
     */
    void invalidate() noexcept;
    /** @param request 最新弹道请求。 @param active_generation 发送屏障内当前代际。
     * @param expired 原提交有效期是否已到期。 @return 本次映射尝试，不代表输出已成功。
     */
    domain::CommandEmissionSnapshot prepare(const domain::GuidanceCommand& request,
        std::uint64_t active_generation, bool expired) noexcept;
    /** @param snapshot 已完成的映射尝试。 @param disposition 实际 I/O 或排队处置。
     * @brief 发布不可变快照，不改变任何请求的有效期。
     */
    void complete(domain::CommandEmissionSnapshot snapshot, domain::CommandEmissionDisposition disposition) noexcept;
    /** @return 最近完成的输出尝试；未输出时为空，不推进时钟或重算控制。 */
    std::optional<domain::CommandEmissionSnapshot> latest_emission() const noexcept;
private:
    CommandOutputBinding binding_;
    std::atomic<std::uint64_t> next_attempt_{0};
    mutable std::mutex mutex_;
    std::optional<domain::CommandEmissionSnapshot> latest_;
};
}
#endif
