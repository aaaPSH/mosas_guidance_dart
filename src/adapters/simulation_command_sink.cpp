#include <mosas/adapters/simulation_command_sink.hpp>
#include <mosas/adapters/simulation_audit.hpp>
#include <cmath>

namespace mosas::adapters {
bool SimulationCommandSink::start(){return session_->ready()&&(!audit_||audit_->healthy());}
bool SimulationCommandSink::try_submit(const runtime::GuidanceCommand& command)noexcept{
    try{
        std::lock_guard<std::mutex> lock(mutex_);
        const auto& t=command.source_time;
        const bool wrong_generation=command.generation!=generation_;
        if(wrong_generation || t.monotonic_ns<0 || (command.valid &&
            (t.source!=domain::TimestampSource::simulation || t.source_domain!=domain::ClockDomain::simulation ||
             t.quality!=domain::TimestampQuality::simulation || t.source_time_ns!=t.monotonic_ns))){
            if(audit_)audit_->record_rejection(command,wrong_generation?AuditRejection::wrong_generation:AuditRejection::invalid_time);
            ++dropped_;return false;
        }
        if(audit_&&!audit_->healthy()){++failed_;session_->cancel();return false;}
        dart_sim_transport::Overload value;value.valid=command.valid?1:0;value.reason=static_cast<std::uint32_t>(command.invalid_reason);
        value.source_frame_id=command.source_frame_id;value.left_g=-command.overload_g.z;value.up_g=command.overload_g.y;
        if(!std::isfinite(value.left_g)||!std::isfinite(value.up_g))value.valid=0;
        const bool submitted=session_->submit(value,t.monotonic_ns);
        if(audit_&&!audit_->record_submit(command,value,submitted)){
            ++failed_;session_->cancel();return false;
        }
        if(!submitted){++failed_;return false;}++submitted_;return true;
    }catch(...){return false;}
}
void SimulationCommandSink::set_active_generation(std::uint64_t generation)noexcept{std::lock_guard<std::mutex> lock(mutex_);generation_=generation;session_->discard_commands();}
void SimulationCommandSink::cancel()noexcept{session_->cancel();}
void SimulationCommandSink::stop()noexcept{session_->stop();}
runtime::CommandSink::Diagnostics SimulationCommandSink::diagnostics()const noexcept{std::lock_guard<std::mutex> lock(mutex_);return {true,submitted_,dropped_,failed_};}
} // namespace mosas::adapters
