#pragma once
#include <mosas/adapters/simulation_session.hpp>

namespace mosas::adapters {
class SimulationAudit;
/** @brief Nonblocking latest-only two-axis socket output, independent of serial encoding. */
class SimulationCommandSink final:public runtime::CommandSink {
public:
    /** @param session Original transport session. @param audit Optional simulation-only diagnostics. */
    explicit SimulationCommandSink(std::shared_ptr<SimulationSession> session,
        std::shared_ptr<SimulationAudit> audit={}):session_(std::move(session)),audit_(std::move(audit)){}
    bool start()override;
    /** @brief 校验弹道请求代际/来源，使用最新姿态映射后排队；不代表物理执行确认。 */
    bool try_submit(const runtime::GuidanceCommand& command)noexcept override;
    /** @brief Discard unsent old-generation commands; socket session IDs are unrelated to generation. */
    void set_active_generation(std::uint64_t generation)noexcept override;
    void cancel()noexcept override;
    void stop()noexcept override;
    Diagnostics diagnostics()const noexcept override;
private:
    std::shared_ptr<SimulationSession> session_;
    std::shared_ptr<SimulationAudit> audit_;
    mutable std::mutex mutex_;
    std::uint64_t generation_=0,submitted_=0,dropped_=0,failed_=0;
};
} // namespace mosas::adapters
