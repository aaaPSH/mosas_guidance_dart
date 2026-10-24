#pragma once
#include <mosas/app/simulation_config.hpp>
#include <mosas/adapters/simulation_sources.hpp>
#include <mosas/adapters/simulation_pre_acquisition.hpp>
#include <mosas/adapters/simulation_audit.hpp>
#include <mutex>
#include <set>

namespace mosas::app {
/**
 * @brief 由宿主线程管理仿真会话，为每个新会话创建独立的算法和运行时；断开的会话不能恢复。
 * @note 除 timing() 外，公开方法及析构均由同一生命周期线程调用。
 *       timing() 允许其他线程并发查询，但调用方必须保证运行器对象本身仍然存活。
 */
class SimulationRunner {
public:
    using Log=std::function<void(const std::string&)>;
    /**
     * @brief 保存会话配置；连接和运行时创建延迟到 step() 执行。
     * @param config 仿真包装配置及原应用配置。
     * @param log 在生命周期线程调用的状态报告函数，可为空。
     * @param audit_options 显式启用的可选审计参数；默认关闭。
     */
    SimulationRunner(SimulationConfig config,Log log,
        std::optional<adapters::SimulationAuditOptions> audit_options = {});
    ~SimulationRunner();
    /** @brief 推进连接和会话生命周期；一次握手等待最多占用宿主线程 25 ms。 */
    void step();
    /** @brief 停止并回收运行时，清除处理水位回调，再等待传输线程退出；可能阻塞。 */
    void stop();
    bool running()const noexcept;
    std::uint64_t session_id()const noexcept;
    const std::string& last_error()const noexcept{return error_;}
    /**
     * @brief 获取线程安全的流水线耗时快照，与监督器的创建和销毁同步。
     * @return 当前运行时的统计；尚未创建或已销毁运行时时返回空统计。
     * @note 可与 step()/stop() 并发，但不是无等待接口；调用期间运行器对象必须保持存活。
     */
    runtime::RuntimeTiming timing() const;
    /** @brief 在生命周期线程获取各输出端诊断。 @return 运行时已销毁时返回空诊断。 */
    runtime::RuntimeOutputDiagnostics output_diagnostics() const;
private:
    void finish(std::string error);
    void report(const std::string& message);
    SimulationConfig config_;Log log_;
    std::optional<adapters::SimulationAuditOptions> audit_options_;
    std::shared_ptr<adapters::SimulationAudit> audit_;
    std::shared_ptr<adapters::SimulationSession> session_;
    // 仅同步监督器所有权与跨线程耗时查询，不将整个生命周期流程锁在临界区内。
    mutable std::mutex supervisor_mutex_;
    std::unique_ptr<runtime::RuntimeSupervisor> supervisor_;
    std::unique_ptr<adapters::SimulationPreAcquisitionPolicy> pre_acquisition_;
    adapters::SimulationCameraSource* camera_=nullptr;
    std::set<std::uint64_t> used_;
    std::chrono::steady_clock::time_point retry_{};
    std::string error_;bool request_reported_=false,launch_reported_=false,release_reported_=false;
};
} // namespace mosas::app
