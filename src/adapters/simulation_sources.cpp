#include <mosas/adapters/simulation_sources.hpp>
#include <opencv2/imgproc.hpp>
#include <mosas/vision/opencv_image_frame.hpp>
#include <stdexcept>

namespace mosas::adapters {
using namespace std::chrono_literals;
namespace {
runtime::SourceResult status(const SimulationSession& s){if(s.failed())return {runtime::SourceStatus::fatal,s.error()};if(s.cancelled())return {runtime::SourceStatus::cancelled,{}};return {s.ready()?runtime::SourceStatus::ok:runtime::SourceStatus::fatal,s.ready()?"":"simulation session not ready"};}
}
runtime::SourceResult SimulationImuSource::start(){return status(*session_);}
bool SimulationImuSource::try_pop(runtime::ImuAngularVelocitySample* sample)noexcept{return session_->pop_imu(sample);}
bool SimulationImuSource::wait_for_data()noexcept{return session_->wait_imu();}
void SimulationImuSource::stop()noexcept{session_->stop();}
void SimulationImuSource::cancel()noexcept{session_->cancel();}
runtime::ImuSourceDiagnostics SimulationImuSource::diagnostics()const noexcept{return session_->imu_diagnostics();}
SimulationCameraSource::SimulationCameraSource(std::shared_ptr<SimulationSession> session,std::int64_t budget):session_(std::move(session)),budget_(budget){if(budget<=0)throw std::invalid_argument("nonpositive simulation sync budget");}
runtime::SourceResult SimulationCameraSource::configure(const runtime::CameraCaptureConfig& config){if(config.mode!=runtime::CameraCaptureMode::free_run)return {runtime::SourceStatus::fatal,"simulation camera is free-run"};return status(*session_);}
runtime::SourceResult SimulationCameraSource::capture(runtime::CameraFrame* frame){
    if(!frame)return {runtime::SourceStatus::fatal,"null camera frame"};
    auto deadline=std::chrono::steady_clock::now()+100ms;
    do{
        auto current=status(*session_);if(current.status!=runtime::SourceStatus::ok){pending_.reset();return current;}
        if(auto newest=session_->take_image())pending_=std::move(newest);
        if(pending_){
            const auto t=pending_->header.sim_time_ns;
            if(session_->now_ns()-t>budget_){pending_.reset();++sync_timeouts_;}
            else{
                auto watermark=session_->processed_state();
                if(watermark && watermark->attitude.valid && watermark->time.source==domain::TimestampSource::simulation && watermark->time.monotonic_ns>=t){
                    auto image=std::make_shared<dart_sim_transport::Image>(std::get<dart_sim_transport::Image>(std::move(pending_->payload)));
                    frame->timestamp_ns=t;frame->received_timestamp_ns=session_->now_ns();frame->source_frame_id=pending_->header.sequence;
                    frame->timestamp_source=runtime::CameraTimestampSource::simulation;frame->sensor_time=domain::make_simulation_time(t);
                    frame->image={image,image->pixels.data(),image->pixels.size(),static_cast<int>(image->width),static_cast<int>(image->height),image->stride,domain::PixelFormat::rgb8};
                    pending_.reset();return {runtime::SourceStatus::ok,{}};
                }
            }
        }
        session_->wait_change(1ms); // Short polling lets the processed watermark advance independently.
    }while(std::chrono::steady_clock::now()<deadline);
    return {runtime::SourceStatus::timeout,{}};
}
runtime::SourceResult SimulationCameraSource::prepare(runtime::CameraFrame* frame){
    if(!frame || !frame->image.is_valid() || frame->image.format!=domain::PixelFormat::rgb8)return {runtime::SourceStatus::fatal,"invalid RGB simulation frame"};
    cv::Mat rgb(frame->image.height,frame->image.width,CV_8UC3,frame->image.data,frame->image.stride_bytes),bgr;
    cv::cvtColor(rgb,bgr,cv::COLOR_RGB2BGR);
    if(!vision::assign_bgr8_mat(&frame->image,std::move(bgr)))return {runtime::SourceStatus::fatal,"BGR conversion failed"};
    return {runtime::SourceStatus::ok,{}};
}
void SimulationCameraSource::cancel()noexcept{session_->cancel();}
void SimulationCameraSource::stop()noexcept{session_->stop();pending_.reset();}
} // namespace mosas::adapters
