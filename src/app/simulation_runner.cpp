#include <mosas/app/simulation_runner.hpp>
#include <mosas/app/default_guidance_algorithms.hpp>
#include <mosas/app/output_factory.hpp>
#include <mosas/app/simulation_contract.hpp>
#include <mosas/adapters/simulation_command_sink.hpp>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <sstream>

namespace mosas::app {
using namespace std::chrono_literals;
using dart_sim_transport::Phase;
namespace {
bool equal(double a, double b) {
    return std::isfinite(a) && std::isfinite(b) &&
           std::abs(a-b) <= 1e-6*std::max({1.0,std::abs(a),std::abs(b)});
}
/** @brief Simulation requires every configured recording sink; hardware runtime policy is unchanged. */
void require_healthy_outputs(const runtime::RuntimeOutputDiagnostics& outputs) {
    for(const auto& sink : {std::make_pair("video",&outputs.video),
                            std::make_pair("telemetry",&outputs.telemetry),
                            std::make_pair("imu",&outputs.imu_recording),
                            std::make_pair("logging",&outputs.logging)})
        if(sink.second->enabled && !sink.second->healthy)
            throw std::runtime_error(std::string(sink.first)+" output failed: "+sink.second->last_error);
}
}
SimulationRunner::SimulationRunner(SimulationConfig config, Log log,
    std::optional<adapters::SimulationAuditOptions> audit_options)
    : config_(std::move(config)), log_(std::move(log)), audit_options_(std::move(audit_options)) {}
SimulationRunner::~SimulationRunner() { stop(); }
void SimulationRunner::report(const std::string& message) { if(log_)log_(message); }
bool SimulationRunner::running() const noexcept { return supervisor_ && supervisor_->running(); }
std::uint64_t SimulationRunner::session_id() const noexcept {
    return session_ ? session_->metadata().session_id : 0;
}
runtime::RuntimeTiming SimulationRunner::timing() const {
    return supervisor_ ? supervisor_->timing() : runtime::RuntimeTiming{};
}
runtime::RuntimeOutputDiagnostics SimulationRunner::output_diagnostics() const {
    return supervisor_ ? supervisor_->output_diagnostics() : runtime::RuntimeOutputDiagnostics{};
}
void SimulationRunner::finish(std::string error) {
    if(supervisor_) {
        supervisor_->stop();
        if(error.empty() && supervisor_->faulted())error=supervisor_->last_error();
        const auto outputs=supervisor_->output_diagnostics();
        for(const auto& sink : {std::make_pair("video",&outputs.video),
                                std::make_pair("telemetry",&outputs.telemetry),
                                std::make_pair("imu",&outputs.imu_recording),
                                std::make_pair("logging",&outputs.logging)}) {
            std::ostringstream text;
            text<<"output "<<sink.first<<" accepted="<<sink.second->accepted_count
                <<" dropped="<<sink.second->dropped_count<<" failures="<<sink.second->failure_count;
            if(!sink.second->healthy)text<<" error="<<sink.second->last_error;
            report(text.str());
        }
        if(pre_acquisition_) {
            const auto policy=pre_acquisition_->diagnostics();
            std::ostringstream text;
            text<<"pre_acquisition preset_submitted="<<policy.preset_submitted
                <<" first_preset_ns="<<policy.first_preset_ns<<" last_preset_ns="<<policy.last_preset_ns
                <<" first_detection_ns="<<policy.first_detection_ns
                <<" positive_y_limit_g="<<config_.app.algorithms.png_guidance.maximum_command_overload;
            report(text.str());
        }
        const auto counters=session_->counters();
        const auto performance=supervisor_->statistics();
        std::ostringstream summary;
        summary<<"session="<<session_id()<<" received_images="<<counters.images
               <<" received_imu="<<counters.imus<<" applied="<<counters.applied
               <<" nonzero_applied="<<counters.nonzero_applied
               <<" sync_dropped="<<(camera_?camera_->sync_timeouts():0)
               <<" configured_camera_hz="<<session_->metadata().camera.camera_hz
               <<" capture_fps_host="<<performance.capture_fps
               <<" processing_fps_host="<<performance.processing_fps<<" output_fps_host="<<performance.output_fps;
        report(summary.str());
        session_->clear_watermark();
        supervisor_.reset();
    }
    camera_=nullptr;
    pre_acquisition_.reset();
    if(audit_) {
        audit_->record_lifecycle(error.empty()?adapters::AuditLifecycle::stopped:adapters::AuditLifecycle::fault,
                                session_?session_->now_ns():0);
        audit_->stop();const auto diagnostics=audit_->diagnostics();
        if(!diagnostics.healthy&&error.empty())error="simulation audit failed: "+diagnostics.error;
        std::ostringstream text;text<<"audit accepted="<<diagnostics.accepted<<" written="<<diagnostics.written
            <<" dropped="<<diagnostics.dropped<<" failures="<<diagnostics.write_failures
            <<" healthy="<<diagnostics.healthy;
        report(text.str());audit_.reset();
    }
    if(!error.empty()) {error_=std::move(error);report("simulation stopped: "+error_);}
    if(session_) {session_->clear_watermark();session_->stop();session_.reset();}
    retry_=std::chrono::steady_clock::now()+250ms;
}
void SimulationRunner::stop() { finish({}); }
void SimulationRunner::step() {
    try {
        if(!session_) {
            if(std::chrono::steady_clock::now()<retry_)return;
            session_=std::make_shared<adapters::SimulationSession>(config_.socket_directory);
        }
        if(!supervisor_) {
            if(!session_->wait_ready(25ms)) {
                if(session_->failed() || session_->cancelled())finish(session_->error());
                return;
            }
            const auto meta=session_->metadata();
            if(used_.count(meta.session_id)) {finish({});return;}
            if(meta.session.phase!=Phase::ready) {
                used_.insert(meta.session_id);
                throw std::runtime_error("cannot join a launch without warmup history; reload a new dart");
            }
            if(meta.session.sensor_ready_flags!=3)return;
            used_.insert(meta.session_id);
            validate_simulation_metadata(config_.app,meta);
            auto runtime_config=config_.app.guidance;
            const auto& c=meta.camera;
            runtime_config.camera_intrinsics={c.fx,c.fy,c.cx,c.cy};
            runtime_config.dart_from_camera=adapters::simulation_camera_rotation(c.R_S_from_C);
            const auto& roi=config_.app.algorithms.vision_config.initial_roi;
            if(roi.x<0 || roi.y<0 || roi.width>static_cast<int>(c.width)-roi.x ||
                roi.height>static_cast<int>(c.height)-roi.y)
                throw std::runtime_error("simulation ROI exceeds handshake image dimensions");
            if(config_.app.output.mode==output::OutputMode::match &&
                !output::match_window_fits_memory(config_.app.camera.width,config_.app.camera.height,
                    config_.app.camera.fps,config_.app.output.match_window_duration_ms))
                throw std::runtime_error("match recording window exceeds image memory limit");
            auto output_config=config_.app;
            output_config.guidance=runtime_config;
            auto outputs=make_runtime_output_ports(output_config,std::cout);
            if(audit_options_) {
                if(config_.app.algorithms.algorithm!=GuidanceAlgorithm::png)
                    throw std::runtime_error("version-one training audit requires PNG");
                adapters::SimulationAuditMetadata observed;
                observed.source_config=config_.app_config_path.string();
                observed.width=c.width;observed.height=c.height;observed.intrinsics={c.fx,c.fy,c.cx,c.cy};
                observed.camera_hz=c.camera_hz;observed.imu_hz=c.imu_hz;
                observed.launch_speed_mps=config_.app.algorithms.attitude.launch_speed_mps;
                const auto& png=config_.app.algorithms.png_guidance;const auto& los=config_.app.algorithms.los_rate_filter;
                observed.cap_g=png.maximum_command_overload;
                observed.parameters={png.navigation_constant_y,png.navigation_constant_z,los.sigma_a_pitch,los.sigma_a_yaw,los.sigma_q};
                audit_=std::make_shared<adapters::SimulationAudit>(*audit_options_,meta.session_id,observed);
            }
            guidance::PngTraceObserver observer;
            if(audit_)observer=[audit=audit_](const domain::GuidanceInput& input,const PngCalculationTrace& trace){
                if(!audit->record_png(input,trace))throw std::runtime_error("simulation PNG audit failed");
            };
            auto algorithms=make_default_guidance_algorithms(config_.app.algorithms,std::move(observer));
            algorithms.monotonic_clock=std::make_unique<adapters::SimulationClock>(session_);
            auto camera=std::make_unique<adapters::SimulationCameraSource>(session_,config_.sync_budget_ns);
            camera_=camera.get();
            std::unique_ptr<runtime::CommandSink> command_sink=
                std::make_unique<adapters::SimulationCommandSink>(session_,audit_);
            if(config_.app.algorithms.algorithm==GuidanceAlgorithm::png) {
                pre_acquisition_=std::make_unique<adapters::SimulationPreAcquisitionPolicy>(
                    config_.app.algorithms.png_guidance.maximum_command_overload,
                    [session=session_]{return session->processed_state();},audit_);
                auto ports=pre_acquisition_->wrap(std::move(algorithms.vision_detector),std::move(command_sink));
                algorithms.vision_detector=std::move(ports.vision_detector);
                command_sink=std::move(ports.command_sink);
            }
            supervisor_=std::make_unique<runtime::RuntimeSupervisor>(
                std::make_unique<adapters::SimulationImuSource>(session_),std::move(camera),
                std::move(command_sink),runtime_config,
                std::move(algorithms),std::move(outputs));
            session_->bind_watermark([this]{return supervisor_->latest_state();});
            if(!supervisor_->start())throw std::runtime_error(supervisor_->last_error());
            require_healthy_outputs(supervisor_->output_diagnostics());
            if(audit_&&!audit_->record_lifecycle(adapters::AuditLifecycle::ready,meta.sim_time_ns))
                throw std::runtime_error("simulation ready audit failed");
            request_reported_=launch_reported_=release_reported_=false;
            error_.clear();
            std::ostringstream ready;
            ready<<"MOSAS simulation ready session="<<meta.session_id<<" source="<<config_.app_config_path
                 <<" camera="<<c.width<<'x'<<c.height<<'@'<<c.camera_hz
                 <<" algorithm="<<(config_.app.algorithms.algorithm==GuidanceAlgorithm::png?"png":"body_vector_pn")
                 <<" pre_acquisition_y_limit_g="<<(pre_acquisition_?config_.app.algorithms.png_guidance.maximum_command_overload:0);
            report(ready.str());
        }
        if(audit_&&!audit_->healthy())throw std::runtime_error("simulation audit failed: "+audit_->diagnostics().error);
        if(session_->failed() || session_->cancelled() || supervisor_->faulted()) {
            auto error=session_->failed()?session_->error():supervisor_->last_error();
            finish(error.empty()?"connection/runtime cancelled":error);return;
        }
        require_healthy_outputs(supervisor_->output_diagnostics());
        const auto meta=session_->metadata();
        validate_simulation_metadata(config_.app,meta);
        if(meta.session.phase!=Phase::ready) {
            if(!equal(meta.session.target_speed_mps,config_.app.algorithms.attitude.launch_speed_mps))
                throw std::runtime_error("launch_speed_mps does not match original app_config; reload with matching speed");
            if(!request_reported_) {
                report(std::string(meta.session.phase==Phase::warmup?"request_ns=":"request metadata first observed phase_start_ns=")+
                       std::to_string(meta.session.phase_start_ns));request_reported_=true;
            }
        }
        const auto state=supervisor_->latest_state();
        if(state && state->launch.phase==domain::LaunchPhase::launched && !launch_reported_) {
            report("MOSAS launch_detected_ns="+std::to_string(state->launch.transition_time.monotonic_ns));launch_reported_=true;
        }
        if(meta.session.release_ns>=0 && !release_reported_) {
            report("release_ns="+std::to_string(meta.session.release_ns));release_reported_=true;
            if(!launch_reported_)report("warning: original launch detector has not confirmed launch; check ramp/threshold (not forced)");
        }
    } catch(const std::exception& e) {finish(e.what());}
}
}  // namespace mosas::app
