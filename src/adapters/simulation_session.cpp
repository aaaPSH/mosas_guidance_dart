#include <mosas/adapters/simulation_session.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace mosas::adapters {
using namespace dart_sim_transport;
using namespace std::chrono_literals;
domain::Rotation3<domain::DartFrameTag,domain::CameraFrameTag> simulation_camera_rotation(const std::array<double,9>& r) {
    for(double x:r) if(!std::isfinite(x)) throw std::invalid_argument("nonfinite camera rotation");
    for(int i=0;i<3;++i) for(int j=0;j<3;++j) {
        double dot=0;for(int k=0;k<3;++k)dot+=r[3*k+i]*r[3*k+j];
        if(std::abs(dot-(i==j?1:0))>1e-6)throw std::invalid_argument("nonorthogonal camera rotation");
    }
    double det=r[0]*(r[4]*r[8]-r[5]*r[7])-r[1]*(r[3]*r[8]-r[5]*r[6])+r[2]*(r[3]*r[7]-r[4]*r[6]);
    if(std::abs(det-1)>1e-6)throw std::invalid_argument("reflected camera rotation");
    constexpr int permutation[]={0,2,1},sign[]={1,1,-1};std::array<double,9> result;
    for(int i=0;i<3;++i)for(int j=0;j<3;++j)result[3*i+j]=sign[i]*sign[j]*r[3*permutation[i]+permutation[j]];
    return domain::Rotation3<domain::DartFrameTag,domain::CameraFrameTag>(result);
}
SimulationSession::SimulationSession(const std::string& d):endpoint_(d),worker_(&SimulationSession::pump,this){}
SimulationSession::~SimulationSession(){stop();clear_watermark();}
bool SimulationSession::wait_ready(std::chrono::milliseconds timeout){std::unique_lock<std::mutex> lock(mutex_);cv_.wait_for(lock,timeout,[&]{return ready_||failed_||cancelled_;});return ready_&&!failed_&&!cancelled_;}
bool SimulationSession::ready()const{std::lock_guard<std::mutex> lock(mutex_);return ready_&&!failed_&&!cancelled_;}
bool SimulationSession::failed()const{std::lock_guard<std::mutex> lock(mutex_);return failed_;}
bool SimulationSession::cancelled()const{std::lock_guard<std::mutex> lock(mutex_);return cancelled_;}
std::string SimulationSession::error()const{std::lock_guard<std::mutex> lock(mutex_);return error_;}
ServerSnapshot SimulationSession::metadata()const{std::lock_guard<std::mutex> lock(mutex_);return metadata_;}
SimulationSession::Counters SimulationSession::counters()const{std::lock_guard<std::mutex> lock(mutex_);return counters_;}
std::int64_t SimulationSession::now_ns()const noexcept{std::lock_guard<std::mutex> lock(mutex_);return now_;}
void SimulationSession::cancel()noexcept{std::lock_guard<std::mutex> lock(mutex_);cancelled_=true;imu_.clear();image_.reset();cv_.notify_all();}
void SimulationSession::fail(std::string error)noexcept{std::lock_guard<std::mutex> lock(mutex_);if(!failed_){error_=std::move(error);failed_=true;}imu_.clear();image_.reset();cv_.notify_all();}
void SimulationSession::stop()noexcept{cancel();if(worker_.joinable())worker_.join();}
bool SimulationSession::pop_imu(runtime::ImuAngularVelocitySample* sample)noexcept{std::lock_guard<std::mutex> lock(mutex_);if(!sample||failed_||cancelled_||imu_.empty())return false;*sample=std::move(imu_.front());imu_.pop_front();++delivered_;return true;}
bool SimulationSession::wait_imu()noexcept{std::unique_lock<std::mutex> lock(mutex_);cv_.wait(lock,[&]{return failed_||cancelled_||!imu_.empty();});return !failed_&&!cancelled_;}
runtime::ImuSourceDiagnostics SimulationSession::imu_diagnostics()const noexcept{std::lock_guard<std::mutex> lock(mutex_);runtime::ImuSourceDiagnostics d;d.available=true;d.latest_sample_timestamp_ns=last_imu_;d.delivered_sample_count=delivered_;d.queued_chunk_count=imu_.size();d.chunk_queue_capacity=256;return d;}
std::optional<Message> SimulationSession::take_image(){std::lock_guard<std::mutex> lock(mutex_);if(failed_||cancelled_)return {};auto result=std::move(image_);image_.reset();return result;}
void SimulationSession::wait_change(std::chrono::milliseconds timeout){std::unique_lock<std::mutex> lock(mutex_);if(!failed_&&!cancelled_)cv_.wait_for(lock,timeout);}
void SimulationSession::bind_watermark(Watermark w){std::lock_guard<std::mutex> lock(watermark_mutex_);watermark_=std::move(w);}
void SimulationSession::clear_watermark(){bind_watermark({});}
std::optional<runtime::ImuStateSnapshot> SimulationSession::processed_state(){std::lock_guard<std::mutex> lock(watermark_mutex_);return watermark_?watermark_():std::nullopt;}
bool SimulationSession::submit(Overload o,std::int64_t ns){if(!ready())return false;return endpoint_.submit(o,ns);}
void SimulationSession::discard_commands(){endpoint_.discard_pending_commands();}
void SimulationSession::accept(Message message){
    std::lock_guard<std::mutex> lock(mutex_);if(failed_||cancelled_)return;
    const auto t=message.header.sim_time_ns;
    if(message.header.session_id!=metadata_.session_id)throw std::runtime_error("simulation session changed");
    switch(message.header.type){
    case Type::imu:{
        if(t<=last_imu_)throw std::runtime_error("IMU time is not increasing");
        if(last_imu_>=0 && static_cast<double>(t-last_imu_)>1.5e9/metadata_.camera.imu_hz)throw std::runtime_error("missing IMU sample/time gap");
        last_imu_=t;++counters_.imus;const auto& imu=std::get<Imu>(message.payload);
        if(imu.validity_flags!=3 && valid_imu_seen_)throw std::runtime_error("invalid IMU sample after stream initialization");
        if(imu.validity_flags==3){
            valid_imu_seen_=true;
            if(imu_.size()==256)throw std::runtime_error("simulation IMU queue overflow");
            runtime::ImuAngularVelocitySample sample;sample.timestamp_ns=t;sample.sensor_time=domain::make_simulation_time(t);
            sample.acceleration={imu.specific_force_S[0],imu.specific_force_S[2],-imu.specific_force_S[1]};
            sample.angular_velocity={imu.angular_velocity_S[0],imu.angular_velocity_S[2],-imu.angular_velocity_S[1]};imu_.push_back(sample);
        }break;
    }
    case Type::image:{
        if(t<=last_image_)throw std::runtime_error("camera time is not increasing");
        const auto& image=std::get<Image>(message.payload);const auto& c=metadata_.camera;
        if(image.width!=c.width||image.height!=c.height||image.stride!=c.stride||image.pixel_format!=c.pixel_format)throw std::runtime_error("camera metadata changed");
        last_image_=t;++counters_.images;image_=std::move(message);break;
    }
    case Type::clock:if(t<last_clock_)throw std::runtime_error("clock time rollback");last_clock_=t;break;
    case Type::session:{
        auto s=std::get<Session>(message.payload);
        if(s.phase_start_ns<metadata_.session.phase_start_ns)throw std::runtime_error("session phase time rollback");
        metadata_.session=s;break;
    }
    case Type::camera_info:break; // Already validated and immutable by the transport handshake.
    case Type::command_status:{auto s=std::get<CommandStatus>(message.payload);
        if(s.status==CommandResult::applied||s.status==CommandResult::limited){++counters_.applied;if(s.applied_left_g!=0||s.applied_up_g!=0)++counters_.nonzero_applied;}break;}
    default:throw std::runtime_error("unexpected simulation message");
    }
    now_=std::max(now_,t);metadata_.sim_time_ns=now_;
    if((metadata_.session.sensor_ready_flags&1) && image_enabled_since_<0)image_enabled_since_=now_;
    cv_.notify_all();
}
void SimulationSession::pump()noexcept{
    try{
        while(!cancelled()&&!endpoint_.wait_connected(25ms)){if(endpoint_.ended())throw std::runtime_error(endpoint_.diagnostic());}
        if(cancelled()){endpoint_.stop();return;}
        auto snapshot=endpoint_.snapshot();simulation_camera_rotation(snapshot.camera.R_S_from_C);
        {std::lock_guard<std::mutex> lock(mutex_);metadata_=snapshot;now_=snapshot.sim_time_ns;ready_=true;if(snapshot.session.sensor_ready_flags&1)image_enabled_since_=now_;cv_.notify_all();}
        while(!cancelled()){
            if(!endpoint_.connected())throw std::runtime_error("simulation disconnected: "+endpoint_.diagnostic());
            Message message;
            // Drain control in bounded batches; image and fault checks must not starve.
            for(int i=0;i<32 && endpoint_.receive(Channel::control,message,0ms);++i)accept(std::move(message));
            if(endpoint_.receive(Channel::image,message,0ms))accept(std::move(message));
            {std::lock_guard<std::mutex> lock(mutex_);auto reference=last_image_>=0?last_image_:image_enabled_since_;
                if((metadata_.session.sensor_ready_flags&1) && reference>=0 && now_-reference>250'000'000)throw std::runtime_error("simulation image stream missing for 250 ms");}
            if(endpoint_.receive(Channel::control,message,2ms))accept(std::move(message));
        }
    }catch(const std::exception& e){fail(e.what());}catch(...){fail("unknown simulation I/O failure");}
    endpoint_.stop();
}
} // namespace mosas::adapters
