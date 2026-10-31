#include <mosas/adapters/simulation_command_sink.hpp>
#include <mosas/adapters/simulation_audit.hpp>
#include <cmath>

namespace mosas::adapters {
bool SimulationCommandSink::start(){
    std::lock_guard<std::mutex> lock(mutex_);
    if(!command_output_.configured()||!session_->ready()||(audit_&&!audit_->healthy()))return false;
    command_output_.reset();return true;
}
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
        if(audit_&&(!audit_->healthy()||!audit_->record_request(command))){++failed_;session_->cancel();return false;}
        auto attempt=command_output_.prepare(command,generation_,false);
        const auto& mapped=attempt.mapping.command;
        dart_sim_transport::Overload value;
        value.valid=mapped.valid?1:0;value.reason=static_cast<std::uint32_t>(mapped.invalid_reason);
        value.source_frame_id=mapped.source_frame_id;value.left_g=-mapped.overload_g.z;value.up_g=mapped.overload_g.y;
        const bool submitted=session_->submit(value,t.monotonic_ns);
        const bool recorded=!audit_||audit_->record_submit(attempt,value,submitted);
        command_output_.complete(std::move(attempt),submitted?domain::CommandEmissionDisposition::transport_accepted:
                                                           domain::CommandEmissionDisposition::io_rejected);
        if(!recorded){++failed_;session_->cancel();return false;}
        if(!submitted){++failed_;return false;}++submitted_;return true;
    }catch(...){return false;}
}
void SimulationCommandSink::set_active_generation(std::uint64_t generation)noexcept{
    std::lock_guard<std::mutex> lock(mutex_);generation_=generation;command_output_.invalidate();session_->discard_commands();
}
void SimulationCommandSink::cancel()noexcept{session_->cancel();}
void SimulationCommandSink::stop()noexcept{session_->stop();}
runtime::CommandSink::Diagnostics SimulationCommandSink::diagnostics()const noexcept{
    std::lock_guard<std::mutex> lock(mutex_);return {true,submitted_,dropped_,failed_};
}
}
