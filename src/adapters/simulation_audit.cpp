#include <mosas/adapters/simulation_audit.hpp>
#include <mosas/runtime/spsc_ring.hpp>
#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cmath>
#include <condition_variable>
#include <fcntl.h>
#include <iomanip>
#include <locale>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <sys/stat.h>
#include <thread>
#include <unistd.h>

namespace mosas::adapters {
namespace {
bool safe_id(const std::string& value){
    auto alpha=[](char c){return (c>='0'&&c<='9')||(c>='a'&&c<='z')||(c>='A'&&c<='Z');};
    return !value.empty()&&value.size()<=80&&alpha(value[0])&&
        std::all_of(value.begin(),value.end(),[&](char c){return alpha(c)||c=='_'||c=='.'||c=='-';});
}
void validate_options(const SimulationAuditOptions& options){
    const auto path=options.directory.string();
    if(!options.directory.is_absolute()||path.find_first_of("\r\n")!=std::string::npos||
        path.find('\0')!=std::string::npos||!safe_id(options.trial_id))
        throw std::invalid_argument("invalid simulation audit directory/trial ID");
}
std::string quote(const std::string& value){
    std::ostringstream out;out<<'"';
    for(unsigned char c:value){
        if(c=='"'||c=='\\')out<<'\\'<<c;
        else if(c<32)out<<"\\u"<<std::hex<<std::setw(4)<<std::setfill('0')<<static_cast<unsigned>(c)<<std::dec;
        else out<<c;
    }out<<'"';return out.str();
}
void real(std::ostream& out,double value){
    if(!std::isfinite(value))throw std::runtime_error("nonfinite simulation audit value");
    if(value==0&&std::signbit(value))out<<"-0.0";else out<<value;
}
template<std::size_t N>void array(std::ostream& out,const std::array<double,N>& values){
    out<<'[';for(std::size_t i=0;i<N;++i){if(i)out<<',';real(out,values[i]);}out<<']';
}
void write_all(int fd,const std::string& value){
    for(std::size_t offset=0;offset<value.size();){
        auto n=::write(fd,value.data()+offset,value.size()-offset);
        if(n<0&&errno==EINTR)continue;
        if(n<=0)throw std::runtime_error("simulation audit write failed");
        offset+=static_cast<std::size_t>(n);
    }
}
int create_file(const std::filesystem::path& path){
    int fd=::open(path.c_str(),O_CREAT|O_EXCL|O_WRONLY|O_CLOEXEC,0600);
    if(fd<0)throw std::runtime_error("cannot exclusively create simulation audit file");
    return fd;
}
enum class Kind {ready,fault,stopped,vision,png,request,submit,rejected};
struct Event {
    Kind kind=Kind::ready;
    std::uint64_t frame=0,generation=0,attempt=0;
    std::int64_t time_ns=0,attitude_ns=-1,emit_ns=-1;
    bool generation_known=false,valid=false;std::uint32_t reason=0,flags=0;
    std::array<double,16> values{};
};
constexpr std::size_t roles=5;
std::string serialize(const Event& e,const std::string& trial,std::uint64_t session){
    constexpr const char* names[]={"algorithm_ready","fault","stopped","vision","png","request","submit","rejected"};
    std::ostringstream out;out.imbue(std::locale::classic());out<<std::setprecision(17);
    out<<"{\"schema_version\":2,\"stream\":\"mosas_simulation_audit\",\"trial_id\":"<<quote(trial)
       <<",\"session_id\":"<<session<<",\"kind\":"<<quote(names[static_cast<unsigned>(e.kind)])
       <<",\"source_frame_id\":"<<e.frame<<",\"source_time_ns\":"<<e.time_ns<<",\"generation\":";
    if(e.generation_known)out<<e.generation;else out<<"null";
    if(e.generation_known)out<<",\"generation_source\":"<<quote(e.kind==Kind::vision?"processed_watermark":"command");
    out<<",\"valid\":"<<(e.valid?"true":"false")<<",\"reason\":";
    if(e.kind==Kind::png||e.kind==Kind::ready||e.kind==Kind::fault||e.kind==Kind::stopped)out<<"null";
    else out<<e.reason;
    out<<",\"flags\":"<<e.flags;
    if(e.kind==Kind::png){
        out<<",\"raw_navigation_overload_g\":";array(out,std::array<double,2>{e.values[0],e.values[1]});
        out<<",\"elevation_rate_rad_s\":";real(out,e.values[2]);
        out<<",\"lateral_rate_rad_s\":";real(out,e.values[3]);
        out<<",\"parabolic_rate_rad_s\":";real(out,e.values[4]);
        out<<",\"compensated_rate_rad_s\":";real(out,e.values[5]);
    }else if(e.kind==Kind::request){
        out<<",\"ballistic_request_g\":";
        array(out,std::array<double,3>{e.values[0],e.values[1],e.values[2]});
    }else if(e.kind==Kind::submit){
        out<<",\"output_attempt_id\":"<<e.attempt<<",\"attitude_time_ns\":"<<e.attitude_ns
           <<",\"emit_time_ns\":"<<e.emit_ns<<",\"submit_accepted\":"<<((e.flags&1)?"true":"false")
           <<",\"overload_limited\":"<<((e.flags&2)?"true":"false")
           <<",\"acceleration_limited\":"<<((e.flags&4)?"true":"false")<<",\"left_g\":";real(out,e.values[0]);
        out<<",\"up_g\":";real(out,e.values[1]);
        out<<",\"after_overload_limit_g\":";array(out,std::array<double,2>{e.values[2],e.values[3]});
        out<<",\"ballistic_acceleration_mps2\":";array(out,std::array<double,2>{e.values[4],e.values[5]});
        out<<",\"body_overload_g\":";array(out,std::array<double,2>{e.values[6],e.values[7]});
        out<<",\"raw_navigation_overload_g\":";array(out,std::array<double,2>{e.values[8],e.values[9]});
    }
    out<<"}\n";return out.str();
}
}
std::optional<SimulationAuditOptions> parse_simulation_audit_options(const std::vector<std::string>& args){
    if(args.empty())return std::nullopt;
    SimulationAuditOptions options;bool directory=false,trial=false;
    if(args.size()%2)throw std::invalid_argument("incomplete simulation audit option");
    for(std::size_t i=0;i<args.size();i+=2){
        if(args[i]=="--audit-directory"&&!directory){options.directory=args[i+1];directory=true;}
        else if(args[i]=="--trial-id"&&!trial){options.trial_id=args[i+1];trial=true;}
        else throw std::invalid_argument("unknown/duplicate simulation audit option");
    }
    if(!directory||!trial)throw std::invalid_argument("audit directory and trial ID must be supplied together");
    validate_options(options);return options;
}
struct SimulationAudit::Impl {
    std::array<std::unique_ptr<runtime::SpscRing<Event>>,roles> queues;
    std::array<std::atomic_flag,roles> producer_busy{};
    std::array<std::uint64_t,roles> last_frame{};
    SimulationAuditOptions options;std::uint64_t session;int fd=-1;std::thread worker;
    std::atomic<bool> accepting{true},closing{false},healthy{true};
    std::atomic<unsigned> producers{0};std::atomic<int> producer_error{0};
    std::atomic<std::uint64_t> accepted{0},written{0},dropped{0},failures{0};
    mutable std::mutex error_mutex;std::string error;
    std::mutex stop_mutex,wait_mutex;std::condition_variable ready;
    Impl(const SimulationAuditOptions& opts,std::uint64_t id,const SimulationAuditMetadata& metadata,std::size_t capacity)
        :options(opts),session(id){
        validate_options(options);
        if(!id||!capacity||capacity>65536)throw std::invalid_argument("invalid audit session/capacity");
        if(!std::filesystem::path(metadata.source_config).is_absolute()||!metadata.width||!metadata.height||
            !(metadata.camera_hz>0)||!(metadata.imu_hz>0)||!(metadata.launch_speed_mps>0)||
            !std::isfinite(metadata.cap_g)||metadata.cap_g<0||
            std::isnan(metadata.maximum_command_acceleration)||metadata.maximum_command_acceleration<0)
            throw std::invalid_argument("invalid simulation audit metadata");
        std::ostringstream meta;meta.imbue(std::locale::classic());meta<<std::setprecision(17);
        meta<<"{\"schema_version\":2,\"trial_id\":"<<quote(options.trial_id)<<",\"session_id\":"<<id
            <<",\"source_config\":"<<quote(metadata.source_config)<<",\"width\":"<<metadata.width
            <<",\"height\":"<<metadata.height<<",\"intrinsics\":";array(meta,metadata.intrinsics);
        meta<<",\"camera_hz\":";real(meta,metadata.camera_hz);meta<<",\"imu_hz\":";real(meta,metadata.imu_hz);
        meta<<",\"launch_speed_mps\":";real(meta,metadata.launch_speed_mps);meta<<",\"cap_g\":";real(meta,metadata.cap_g);
        meta<<",\"parameter_names\":[\"Ny\",\"Nz\",\"sigma_a\",\"sigma_l\",\"initial_rate_covariance\",\"maximum_gap_seconds\"]";
        meta<<",\"parameters\":";array(meta,metadata.parameters);
        meta<<",\"maximum_command_acceleration_mps2\":";
        if(std::isfinite(metadata.maximum_command_acceleration)) real(meta,metadata.maximum_command_acceleration);
        else meta<<"null"; // +inf 表示禁用，不写入非法 JSON 数字。
        meta<<"}\n";
        for(std::size_t role=0;role<roles;++role){queues[role]=std::make_unique<runtime::SpscRing<Event>>(capacity);producer_busy[role].clear();}
        std::filesystem::create_directories(options.directory);
        const auto directory=options.directory/("session-"+std::to_string(id));
        if(!std::filesystem::create_directory(directory))throw std::runtime_error("audit session directory already exists");
        if(::chmod(directory.c_str(),0700)!=0)throw std::runtime_error("cannot protect audit directory");
        int metadata_fd=create_file(directory/"metadata.json");
        try{write_all(metadata_fd,meta.str());if(::fsync(metadata_fd)!=0)throw std::runtime_error("audit metadata fsync failed");}
        catch(...){::close(metadata_fd);throw;}
        if(::close(metadata_fd)!=0)throw std::runtime_error("audit metadata close failed");
        fd=create_file(directory/"events.jsonl");
        try{worker=std::thread([this]{run();});}catch(...){::close(fd);fd=-1;throw;}
    }
    ~Impl(){stop();}
    bool submit(std::size_t role,const Event& event)noexcept{
        producers.fetch_add(1,std::memory_order_acq_rel);bool result=false;
        if(accepting.load(std::memory_order_acquire)&&healthy.load(std::memory_order_acquire)){
            if(producer_busy[role].test_and_set(std::memory_order_acquire)){
                ++dropped;producer_error=1;healthy=false;
            }else{
                const bool needs_sequence=role==1||role==2||(role==4&&event.kind==Kind::submit);
                const auto sequence=role==4?event.attempt:event.frame;
                if(needs_sequence&&(!sequence||sequence<=last_frame[role])){
                    ++dropped;producer_error=3;healthy=false;
                }else if(!queues[role]->try_push(event)){
                    ++dropped;producer_error=2;healthy=false;
                }else{
                    if(needs_sequence)last_frame[role]=sequence;
                    ++accepted;result=true;ready.notify_one();
                }
                producer_busy[role].clear(std::memory_order_release);
            }
        }
        producers.fetch_sub(1,std::memory_order_release);return result;
    }
    void fail(const std::string& value){healthy=false;++failures;std::lock_guard<std::mutex> lock(error_mutex);if(error.empty())error=value;}
    void run()noexcept{
        for(;;){
            bool any=false;
            for(auto& queue:queues){Event event;if(queue->try_pop(&event)){
                any=true;try{write_all(fd,serialize(event,options.trial_id,session));++written;}
                catch(const std::exception& e){fail(e.what());}
            }}
            if(any)continue;
            if(closing.load(std::memory_order_acquire))break;
            std::unique_lock<std::mutex> lock(wait_mutex);ready.wait_for(lock,std::chrono::milliseconds(1));
        }
        try{
            std::ostringstream footer;footer<<"{\"schema_version\":2,\"stream\":\"mosas_simulation_audit\",\"kind\":\"sink_complete\",\"accepted\":"
                <<accepted.load()<<",\"written\":"<<written.load()<<",\"dropped_records\":"<<dropped.load()
                <<",\"write_failures\":"<<failures.load()<<",\"healthy\":"<<(healthy.load()?"true":"false")<<"}\n";
            write_all(fd,footer.str());if(::fsync(fd)!=0)throw std::runtime_error("audit events fsync failed");
        }catch(const std::exception& e){fail(e.what());}
    }
    void stop()noexcept{
        std::lock_guard<std::mutex> lock(stop_mutex);accepting=false;
        while(producers.load(std::memory_order_acquire))std::this_thread::yield();
        closing.store(true,std::memory_order_release);ready.notify_one();if(worker.joinable())worker.join();
        if(fd>=0){if(::close(fd)!=0){healthy=false;++failures;producer_error=4;}fd=-1;}
    }
};
SimulationAudit::SimulationAudit(const SimulationAuditOptions& options,std::uint64_t session,
    const SimulationAuditMetadata& metadata,std::size_t capacity):impl_(std::make_unique<Impl>(options,session,metadata,capacity)){}
SimulationAudit::~SimulationAudit()=default;
bool SimulationAudit::record_lifecycle(AuditLifecycle state,std::int64_t time)noexcept{
    Event event;event.kind=state==AuditLifecycle::ready?Kind::ready:(state==AuditLifecycle::fault?Kind::fault:Kind::stopped);
    event.time_ns=time;event.valid=state==AuditLifecycle::ready;return impl_->submit(0,event);
}
bool SimulationAudit::record_vision(std::uint64_t frame,const domain::SensorTime& time,
    std::optional<std::uint64_t> generation,bool found,std::uint32_t reason)noexcept{
    Event event;event.kind=Kind::vision;event.frame=frame;event.time_ns=time.monotonic_ns;
    event.generation_known=generation.has_value();event.generation=generation.value_or(0);
    event.valid=found;event.reason=reason;return impl_->submit(1,event);
}
bool SimulationAudit::record_png(const domain::GuidanceInput& input,const guidance::PngCalculationTrace& trace)noexcept{
    Event event;event.kind=Kind::png;event.frame=input.source_frame_id;event.time_ns=input.time.monotonic_ns;event.valid=trace.valid;
    event.values={trace.requested_overload.y,trace.requested_overload.z,trace.measured_elevation_rate_rad_s,
        trace.measured_lateral_rate_rad_s,trace.parabolic_elevation_rate_rad_s,trace.compensated_elevation_rate_rad_s};
    return impl_->submit(2,event);
}
bool SimulationAudit::record_request(const runtime::GuidanceCommand& command)noexcept{
    Event event;event.kind=Kind::request;event.frame=command.source_frame_id;
    event.time_ns=command.source_time.monotonic_ns;event.generation_known=true;event.generation=command.generation;
    event.valid=command.valid;event.reason=static_cast<std::uint32_t>(command.invalid_reason);
    event.values={command.overload_g.x,command.overload_g.y,command.overload_g.z};return impl_->submit(3,event);
}
bool SimulationAudit::record_submit(const domain::CommandEmissionSnapshot& emission,const dart_sim_transport::Overload& wire,bool accepted)noexcept{
    const auto& command=emission.mapping.command;const auto& trace=emission.mapping.trace;
    Event event;event.kind=Kind::submit;event.frame=wire.source_frame_id;event.time_ns=command.source_time.monotonic_ns;
    event.generation_known=true;event.generation=command.generation;event.valid=wire.valid==1;event.reason=wire.reason;
    event.attempt=emission.attempt_id;event.attitude_ns=trace.attitude.time.monotonic_ns;event.emit_ns=trace.emit_time_ns;
    event.flags=(accepted?1u:0u)|(trace.overload_limited?2u:0u)|(trace.acceleration_limited?4u:0u);
    event.values={wire.left_g,wire.up_g,trace.after_overload_limit.y,trace.after_overload_limit.z,
        trace.ballistic_acceleration.y,trace.ballistic_acceleration.z,trace.body_overload.y,trace.body_overload.z,
        trace.raw_navigation_overload.y,trace.raw_navigation_overload.z};return impl_->submit(4,event);
}
bool SimulationAudit::record_rejection(const runtime::GuidanceCommand& command,AuditRejection reason)noexcept{
    Event event;event.kind=Kind::rejected;event.frame=command.source_frame_id;event.time_ns=command.source_time.monotonic_ns;
    event.generation_known=true;event.generation=command.generation;event.reason=static_cast<std::uint32_t>(reason);
    return impl_->submit(4,event);
}
bool SimulationAudit::healthy()const noexcept{return impl_->healthy.load(std::memory_order_acquire);}
void SimulationAudit::stop()noexcept{impl_->stop();}
SimulationAuditDiagnostics SimulationAudit::diagnostics()const{
    SimulationAuditDiagnostics value;value.healthy=healthy();value.accepted=impl_->accepted.load();value.written=impl_->written.load();
    value.dropped=impl_->dropped.load();value.write_failures=impl_->failures.load();
    std::lock_guard<std::mutex> lock(impl_->error_mutex);value.error=impl_->error;
    if(value.error.empty())switch(impl_->producer_error.load()){
        case 1:value.error="concurrent producer role misuse";break;case 2:value.error="audit queue overflow";break;
        case 3:value.error="nonunique or missing valid source frame";break;case 4:value.error="audit close failed";break;default:break;
    }return value;
}
} // namespace mosas::adapters
