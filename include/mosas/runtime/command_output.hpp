#ifndef MOSAS_RUNTIME_COMMAND_OUTPUT_HPP
#define MOSAS_RUNTIME_COMMAND_OUTPUT_HPP
#include <mosas/ports/command_output_mapper.hpp>
#include <atomic>
#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <vector>

namespace mosas::runtime {
/** @brief 单个来源请求的首次实际输出凭据；共享给观测 worker，不保存后续重发结果。 */
class CommandEmissionReceipt final {
public:
    /** @return 当前消费者独立持有的 future；结果为首次匹配输出，跳过或重置时为空。 */
    std::shared_future<std::optional<domain::CommandEmissionSnapshot>> result() const noexcept {
        return result_;
    }
    /** @return 观测 worker 等待的单调时钟截止时刻，不影响控制请求有效期。 */
    std::chrono::steady_clock::time_point deadline() const noexcept { return deadline_; }
private:
    friend class CommandOutputProcessor;
    CommandEmissionReceipt(const domain::GuidanceCommand& request,
                           std::chrono::steady_clock::duration maximum_wait)
        : request_(request), result_(completion_.get_future().share()),
          deadline_(std::chrono::steady_clock::now() + maximum_wait) {}
    domain::GuidanceCommand request_;
    std::promise<std::optional<domain::CommandEmissionSnapshot>> completion_;
    std::shared_future<std::optional<domain::CommandEmissionSnapshot>> result_;
    std::chrono::steady_clock::time_point deadline_;
};
/** @brief 输出端必需依赖；仅在启动前绑定，状态源生命周期覆盖输出线程。 */
struct CommandOutputBinding {
    std::unique_ptr<ports::ICommandOutputMapper> mapper;
    std::function<std::optional<domain::FlightStateSnapshot>()> latest_state;
    std::function<std::int64_t()> now_ns;
    std::function<bool()> output_enabled;
    domain::ClockDomain clock_domain = domain::ClockDomain::unknown;
    std::int64_t maximum_attitude_age_ns = 0;
    std::int64_t maximum_source_age_ns = 0; ///< 正常 PNG 复用 max_camera_age_ns，固定模式不使用。
};
/** @brief 所有输出适配器共享的上下文采集与诊断；不执行 I/O 或保存重发请求。 */
class CommandOutputProcessor {
public:
    /** @param binding 完整映射依赖；运行中不得重新绑定。 @return 依赖完整合法时为 true。 */
    bool bind(CommandOutputBinding binding);
    /** @return 已绑定完整依赖时为 true。 */
    bool configured() const noexcept;
    /** @brief 新运行开始时清空触发状态、快照及尝试编号，结束未完成凭据，保留绑定。
     * @note 调用方须保证旧输出已结束且新输出尚未开始；允许重复使用同一个 generation。
     */
    void reset() noexcept;
    /** @brief 切代时清空触发状态及快照，将未完成凭据置为空；不重用本次运行的尝试编号。
     * @note 必须在输出端发送屏障内调用，排除 prepare()/complete() 在途尝试。
     */
    void invalidate() noexcept;
    /** @param request 最新弹道请求。 @param active_generation 发送屏障内当前代际。
     * @param expired 原提交有效期是否已到期。
     * @param overload_step_g 输出过载量化步长；0 保留连续值，串口传入协议步长。
     * @return 本次映射尝试，最终命令与诊断采用相同精度，不代表输出已成功。
     */
    domain::CommandEmissionSnapshot prepare(const domain::GuidanceCommand& request,
        std::uint64_t active_generation, bool expired, double overload_step_g = 0.0) noexcept;
    /** @brief 在提交前登记同帧输出凭据，不等待发送，也不重算控制。
     * @param request 即将提交的来源请求。
     * @param maximum_wait 观测延后的时间上限；同步输出使用零等待。
     * @return 由处理帧持有的凭据；处理器只保留弱引用，丢帧不积压历史。
     */
    std::shared_ptr<CommandEmissionReceipt> track_emission(
        const domain::GuidanceCommand& request,
        std::chrono::steady_clock::duration maximum_wait);
    /** @param snapshot 已完成的映射尝试。 @param disposition 实际 I/O 或排队处置。
     * @brief 发布快照并完成同帧凭据，跳过的旧来源置为空；不改变请求有效期。
     */
    void complete(domain::CommandEmissionSnapshot snapshot, domain::CommandEmissionDisposition disposition) noexcept;
    /** @return 最近完成的输出尝试；未输出时为空，不推进时钟或重算控制。 */
    std::optional<domain::CommandEmissionSnapshot> latest_emission() const noexcept;
private:
    CommandOutputBinding binding_;
    std::atomic<std::uint64_t> next_attempt_{0};
    mutable std::mutex mutex_;
    std::optional<domain::CommandEmissionSnapshot> latest_;
    std::vector<std::weak_ptr<CommandEmissionReceipt>> pending_emissions_;
};
}
#endif
