#include <mosas/runtime/async_info_logger.hpp>
#include <mosas/runtime/clock.hpp>
#include <mosas/runtime/frame_rate_meter.hpp>
#include <mosas/runtime/runtime_supervisor.hpp>
#include <mosas/runtime/timing_distribution.hpp>

#include "frame_pipeline.hpp"
#include "imu_pipeline.hpp"
#include "launch_generation_coordinator.hpp"
#include "output_dispatcher.hpp"
#include "runtime_value_helpers.hpp"

#include <atomic>
#include <exception>
#include <iostream>
#include <mutex>
#include <thread>
#include <utility>

namespace mosas::runtime {

struct RuntimeSupervisor::Impl {
    enum class LifecycleState {
        idle,
        starting,
        start_cancelled,
        running,
        stopping,
        stopped,
    };

    Impl(std::unique_ptr<ImuSource> imu_source,
         std::unique_ptr<CameraSource> camera_source,
         std::unique_ptr<CommandSink> command_sink,
         const RuntimeSupervisorConfig& config,
         RuntimeSupervisorDependencies dependencies,
         std::unique_ptr<FrameSink> frame_sink, std::ostream& output);

    bool start();
    void cancel() noexcept;
    void stop() noexcept;
    bool running() const noexcept;
    bool faulted() const noexcept;
    std::string last_error() const;
    std::optional<ImuStateSnapshot> latest_state() const;
    RuntimeStatistics statistics() const;
    RuntimeTiming timing() const;

private:
    bool validate_dependencies() const;
    bool startup_cancelled() const noexcept;
    void stop_locked() noexcept;
    void request_cancellation(bool cancel_command_sink) noexcept;
    void cancel_components() noexcept;
    void cancel_sources() noexcept;
    enum class TimingStage {
        output_queue_wait,
        output_queue_delay,
        output,
    };

    struct TimingAccumulator {
        TimingDistribution distribution;
    };

    void record_timing(TimingStage stage,
                       std::chrono::steady_clock::duration duration) noexcept;
    void log(const std::string& message);
    void set_error(const std::string& message);
    void set_fault(const std::string& message);

    std::unique_ptr<CommandSink> command_sink_;
    std::unique_ptr<ports::IMonotonicClock> monotonic_clock_;
    std::unique_ptr<LaunchGenerationCoordinator> generation_coordinator_;
    std::unique_ptr<OutputDispatcher> output_dispatcher_;
    std::unique_ptr<ImuPipeline> imu_pipeline_;
    std::unique_ptr<FramePipeline> frame_pipeline_;
    RuntimeSupervisorConfig config_;
    std::ostream* output_stream_ = nullptr;
    AsyncInfoLogger info_logger_;

    std::atomic<bool> stop_requested_{false};
    std::atomic<bool> running_{false};
    std::atomic<bool> faulted_{false};
    std::atomic<bool> cancel_requested_{false};
    std::atomic<bool> command_cancel_requested_{false};
    std::atomic<LifecycleState> lifecycle_state_{LifecycleState::idle};
    std::atomic<std::uint32_t> active_cancel_calls_{0};
    mutable std::mutex lifecycle_mutex_;

    FrameRateMeter output_fps_meter_;
    mutable std::mutex timing_mutex_;
    TimingAccumulator output_queue_wait_timing_;
    TimingAccumulator output_queue_delay_timing_;
    TimingAccumulator output_timing_;
    mutable std::mutex error_mutex_;
    std::string last_error_;
    mutable std::mutex log_mutex_;
};

RuntimeSupervisor::Impl::Impl(std::unique_ptr<ImuSource> imu_source,
                              std::unique_ptr<CameraSource> camera_source,
                              std::unique_ptr<CommandSink> command_sink,
                              const RuntimeSupervisorConfig& config,
                              RuntimeSupervisorDependencies dependencies,
                              std::unique_ptr<FrameSink> frame_sink,
                              std::ostream& output)
    : command_sink_(std::move(command_sink)),
      monotonic_clock_(std::move(dependencies.monotonic_clock)),
      config_(config), output_stream_(&output),
      info_logger_(output, config_.info_log_queue_capacity) {
    generation_coordinator_ = std::make_unique<LaunchGenerationCoordinator>();
    OutputDispatcherCallbacks callbacks;
    callbacks.stop_requested = [this] { return stop_requested_.load(); };
    callbacks.active_generation = [this] {
        return generation_coordinator_->current();
    };
    callbacks.processing_fps = [this] {
        return frame_pipeline_->processing_fps();
    };
    callbacks.record_timing =
        [this](OutputDispatchTimingStage stage,
               std::chrono::steady_clock::duration duration) {
            switch (stage) {
                case OutputDispatchTimingStage::queue_wait:
                    record_timing(TimingStage::output_queue_wait, duration);
                    break;
                case OutputDispatchTimingStage::queue_delay:
                    record_timing(TimingStage::output_queue_delay, duration);
                    break;
                case OutputDispatchTimingStage::publish:
                    record_timing(TimingStage::output, duration);
                    break;
            }
        };
    callbacks.record_published_frame = [this] { output_fps_meter_.record(); };
    callbacks.request_rearm = [this] {
        generation_coordinator_->advance(*command_sink_);
    };
    callbacks.report_fault = [this](std::string message) {
        set_fault(message);
    };
    output_dispatcher_ = std::make_unique<OutputDispatcher>(
        std::move(frame_sink), config_.output_queue_capacity,
        std::move(callbacks));

    ImuPipelineCallbacks imu_callbacks;
    imu_callbacks.stop_requested = [this] { return stop_requested_.load(); };
    imu_callbacks.try_record_sample =
        [this](const domain::ImuKinematicSample& sample) {
            return output_dispatcher_->try_record_imu_sample(sample);
        };
    imu_callbacks.report_fault = [this](std::string message) {
        set_fault(message);
    };
    imu_pipeline_ = std::make_unique<ImuPipeline>(
        std::move(imu_source), std::move(dependencies.flight_state_estimator),
        config_.history_capacity, *generation_coordinator_,
        std::move(imu_callbacks));

    FramePipelineCallbacks frame_callbacks;
    frame_callbacks.stop_requested = [this] { return stop_requested_.load(); };
    frame_callbacks.faulted = [this] { return faulted_.load(); };
    frame_callbacks.report_fault = [this](std::string message) {
        set_fault(message);
    };
    frame_callbacks.log = [this](const std::string& message) { log(message); };
    frame_pipeline_ = std::make_unique<FramePipeline>(
        std::move(camera_source), std::move(dependencies.vision_detector),
        std::move(dependencies.los_estimator),
        std::move(dependencies.los_rate_filter),
        std::move(dependencies.guidance_law), config_, *imu_pipeline_,
        *command_sink_, *generation_coordinator_, *output_dispatcher_,
        std::move(frame_callbacks));
}

bool RuntimeSupervisor::Impl::validate_dependencies() const {
    return imu_pipeline_ != nullptr && imu_pipeline_->valid() &&
           frame_pipeline_ != nullptr && frame_pipeline_->valid() &&
           command_sink_ != nullptr && monotonic_clock_ != nullptr;
}

bool RuntimeSupervisor::Impl::start() {
    std::unique_lock<std::mutex> lifecycle_lock(lifecycle_mutex_);
    const LifecycleState current = lifecycle_state_.load();
    if (current == LifecycleState::starting ||
        current == LifecycleState::start_cancelled ||
        current == LifecycleState::running ||
        current == LifecycleState::stopping) {
        set_error("runtime supervisor is already active");
        return false;
    }
    if (!validate_dependencies()) {
        set_error("runtime dependencies are incomplete");
        return false;
    }

    cancel_requested_.store(false);
    command_cancel_requested_.store(false);
    stop_requested_.store(false);
    running_.store(false);
    faulted_.store(false);
    {
        std::lock_guard<std::mutex> lock(error_mutex_);
        last_error_.clear();
    }
    lifecycle_state_.store(LifecycleState::starting);

    if (!info_logger_.start()) {
        set_error("failed to start asynchronous runtime logger");
        stop_locked();
        return false;
    }
    if (startup_cancelled()) {
        stop_locked();
        return false;
    }

    const SourceResult configure_result = frame_pipeline_->configure_source();
    if (configure_result.status != SourceStatus::ok) {
        set_fault(detail::format_source_error("camera source", "configure",
                                              configure_result));
        stop_locked();
        return false;
    }
    if (startup_cancelled()) {
        stop_locked();
        return false;
    }

    const SourceResult imu_start_result = imu_pipeline_->start_source();
    if (imu_start_result.status != SourceStatus::ok) {
        set_fault(detail::format_source_error("imu source", "start",
                                              imu_start_result));
        stop_locked();
        return false;
    }
    if (startup_cancelled()) {
        stop_locked();
        return false;
    }

    generation_coordinator_->reset(*command_sink_);
    if (!command_sink_->start()) {
        set_fault("command sink start failed");
        stop_locked();
        return false;
    }
    if (startup_cancelled()) {
        stop_locked();
        return false;
    }

    output_dispatcher_->reset();
    frame_pipeline_->reset();
    imu_pipeline_->reset();
    if (startup_cancelled()) {
        stop_locked();
        return false;
    }

    running_.store(true);
    try {
        imu_pipeline_->start_worker();
        frame_pipeline_->start_workers();
        output_dispatcher_->start();
    } catch (const std::exception& exception) {
        set_fault(exception.what());
        stop_locked();
        return false;
    } catch (...) {
        set_fault("failed to start runtime supervisor workers");
        stop_locked();
        return false;
    }

    LifecycleState expected = LifecycleState::starting;
    if (!lifecycle_state_.compare_exchange_strong(expected,
                                                  LifecycleState::running)) {
        stop_locked();
        return false;
    }
    return true;
}

void RuntimeSupervisor::Impl::cancel() noexcept { request_cancellation(true); }

void RuntimeSupervisor::Impl::stop() noexcept {
    request_cancellation(false);
    std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
    stop_locked();
}

bool RuntimeSupervisor::Impl::startup_cancelled() const noexcept {
    return lifecycle_state_.load() != LifecycleState::starting;
}

void RuntimeSupervisor::Impl::stop_locked() noexcept {
    if (lifecycle_state_.load() == LifecycleState::stopped) {
        return;
    }
    lifecycle_state_.store(LifecycleState::stopping);
    cancel_requested_.store(true);
    stop_requested_.store(true);
    cancel_components();
    while (active_cancel_calls_.load() != 0) {
        std::this_thread::yield();
    }

    frame_pipeline_->join_capture_worker();
    frame_pipeline_->close_queue();
    frame_pipeline_->join_processing_worker();
    // capture/prepare 均可能访问相机源，必须等 FramePipeline 两个工作线程退出。
    frame_pipeline_->stop_source();
    imu_pipeline_->join_worker();
    imu_pipeline_->stop_source();
    output_dispatcher_->close_queue();
    output_dispatcher_->join();
    const std::string output_error = output_dispatcher_->stop_sink();
    if (!output_error.empty() && !faulted_.load()) {
        set_fault("frame output failed during stop: " + output_error);
    }
    if (command_sink_ != nullptr) {
        command_sink_->stop();
    }
    info_logger_.stop();
    running_.store(false);
    lifecycle_state_.store(LifecycleState::stopped);
}

void RuntimeSupervisor::Impl::request_cancellation(
    bool cancel_command_sink) noexcept {
    active_cancel_calls_.fetch_add(1);
    bool active_session = false;
    bool components_are_running = false;
    while (true) {
        LifecycleState current = lifecycle_state_.load();
        if (current == LifecycleState::starting) {
            if (lifecycle_state_.compare_exchange_weak(
                    current, LifecycleState::start_cancelled)) {
                active_session = true;
                break;
            }
            continue;
        }
        if (current == LifecycleState::start_cancelled) {
            active_session = true;
        } else if (current == LifecycleState::running) {
            active_session = true;
            components_are_running = true;
        }
        break;
    }

    if (active_session) {
        const bool first_request = !cancel_requested_.exchange(true);
        stop_requested_.store(true);
        if (first_request && components_are_running) {
            cancel_components();
        }
        if (cancel_command_sink && components_are_running &&
            command_sink_ != nullptr &&
            !command_cancel_requested_.exchange(true)) {
            command_sink_->cancel();
        }
    }
    active_cancel_calls_.fetch_sub(1);
}

void RuntimeSupervisor::Impl::cancel_components() noexcept {
    output_dispatcher_->cancel();
    cancel_sources();
}

bool RuntimeSupervisor::Impl::running() const noexcept {
    return running_.load();
}

bool RuntimeSupervisor::Impl::faulted() const noexcept {
    return faulted_.load();
}

std::string RuntimeSupervisor::Impl::last_error() const {
    std::lock_guard<std::mutex> lock(error_mutex_);
    return last_error_;
}

std::optional<ImuStateSnapshot> RuntimeSupervisor::Impl::latest_state() const {
    return imu_pipeline_->latest_state();
}

RuntimeStatistics RuntimeSupervisor::Impl::statistics() const {
    RuntimeStatistics result = frame_pipeline_->statistics();
    result.output_fps = output_fps_meter_.snapshot();
    return result;
}

RuntimeTiming RuntimeSupervisor::Impl::timing() const {
    RuntimeTiming result = frame_pipeline_->timing();
    std::lock_guard<std::mutex> lock(timing_mutex_);
    const auto to_stage = [](const TimingAccumulator& accumulator) {
        const auto summary = accumulator.distribution.summary();
        constexpr double kNanosecondsPerMillisecond = 1e6;
        return RuntimeTimingStage{
            summary.samples,
            summary.minimum_ns / kNanosecondsPerMillisecond,
            summary.average_ns / kNanosecondsPerMillisecond,
            summary.p50_ns / kNanosecondsPerMillisecond,
            summary.p95_ns / kNanosecondsPerMillisecond,
            summary.p99_ns / kNanosecondsPerMillisecond,
            summary.maximum_ns / kNanosecondsPerMillisecond};
    };
    result.output_queue_wait = to_stage(output_queue_wait_timing_);
    result.output_queue_delay = to_stage(output_queue_delay_timing_);
    result.output = to_stage(output_timing_);
    result.output_queue_dropped = output_dispatcher_->dropped_count();
    return result;
}

void RuntimeSupervisor::Impl::record_timing(
    TimingStage stage, std::chrono::steady_clock::duration duration) noexcept {
    const auto duration_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
    if (duration_ns < 0) {
        return;
    }

    std::lock_guard<std::mutex> lock(timing_mutex_);
    TimingAccumulator* accumulator = nullptr;
    switch (stage) {
        case TimingStage::output_queue_wait:
            accumulator = &output_queue_wait_timing_;
            break;
        case TimingStage::output_queue_delay:
            accumulator = &output_queue_delay_timing_;
            break;
        case TimingStage::output:
            accumulator = &output_timing_;
            break;
    }
    accumulator->distribution.record_ns(
        static_cast<std::uint64_t>(duration_ns));
}

void RuntimeSupervisor::Impl::log(const std::string& message) {
    const std::string formatted = "[运行时] " + message;
    const TimestampNs now_ns = monotonic_clock_ != nullptr
                                   ? monotonic_clock_->now_ns()
                                   : monotonic_timestamp_ns();
    if (info_logger_.try_submit_text(now_ns, formatted)) {
        return;
    }
    // 日志线程已经启动时，队列满只丢弃，不能把同步输出带回关键路径。
    if (info_logger_.running()) {
        return;
    }
    std::lock_guard<std::mutex> lock(log_mutex_);
    if (output_stream_ != nullptr) {
        *output_stream_ << formatted << '\n';
    }
}

void RuntimeSupervisor::Impl::set_error(const std::string& message) {
    {
        std::lock_guard<std::mutex> lock(error_mutex_);
        last_error_ = message;
    }
    log("错误: " + message);
}

void RuntimeSupervisor::Impl::set_fault(const std::string& message) {
    if (!faulted_.exchange(true)) {
        set_error(message);
        const auto state = latest_state();
        if (command_sink_ != nullptr && state.has_value() &&
            state->launch.phase == domain::LaunchPhase::launched) {
            generation_coordinator_->with_generation(
                [&](std::uint64_t active_generation) {
                    if (active_generation == state->launch.generation) {
                        // 运行时故障时尽力清零，避免下位机继续保持上一条非零指令。
                        const TimestampNs now_ns = monotonic_clock_->now_ns();
                        command_sink_->try_submit(detail::make_neutral_command(
                            now_ns, state->launch.generation));
                    }
                });
        }
    }
    request_cancellation(false);
    frame_pipeline_->close_queue();
    output_dispatcher_->close_queue();
}

void RuntimeSupervisor::Impl::cancel_sources() noexcept {
    if (imu_pipeline_ != nullptr) {
        imu_pipeline_->cancel_source();
    }
    if (frame_pipeline_ != nullptr) {
        frame_pipeline_->cancel_source();
    }
}

RuntimeSupervisor::RuntimeSupervisor(
    std::unique_ptr<ImuSource> imu_source,
    std::unique_ptr<CameraSource> camera_source,
    std::unique_ptr<CommandSink> command_sink,
    const RuntimeSupervisorConfig& config,
    RuntimeSupervisorDependencies dependencies,
    std::unique_ptr<FrameSink> frame_sink)
    : RuntimeSupervisor(std::move(imu_source), std::move(camera_source),
                        std::move(command_sink), config,
                        std::move(dependencies), std::move(frame_sink),
                        std::cerr) {}

RuntimeSupervisor::RuntimeSupervisor(
    std::unique_ptr<ImuSource> imu_source,
    std::unique_ptr<CameraSource> camera_source,
    std::unique_ptr<CommandSink> command_sink,
    const RuntimeSupervisorConfig& config,
    RuntimeSupervisorDependencies dependencies,
    std::unique_ptr<FrameSink> frame_sink, std::ostream& output)
    : impl_(std::make_unique<Impl>(
          std::move(imu_source), std::move(camera_source),
          std::move(command_sink), config, std::move(dependencies),
          std::move(frame_sink), output)) {}

RuntimeSupervisor::~RuntimeSupervisor() { stop(); }

bool RuntimeSupervisor::start() { return impl_->start(); }
void RuntimeSupervisor::cancel() noexcept { impl_->cancel(); }
void RuntimeSupervisor::stop() noexcept { impl_->stop(); }
bool RuntimeSupervisor::running() const noexcept { return impl_->running(); }
bool RuntimeSupervisor::faulted() const noexcept { return impl_->faulted(); }
std::string RuntimeSupervisor::last_error() const {
    return impl_->last_error();
}
std::optional<ImuStateSnapshot> RuntimeSupervisor::latest_state() const {
    return impl_->latest_state();
}
RuntimeStatistics RuntimeSupervisor::statistics() const {
    return impl_->statistics();
}
RuntimeTiming RuntimeSupervisor::timing() const { return impl_->timing(); }

}  // namespace mosas::runtime
