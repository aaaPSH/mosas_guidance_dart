#ifndef MOSAS_RUNTIME_OUTPUT_PORTS_HPP
#define MOSAS_RUNTIME_OUTPUT_PORTS_HPP

#include <mosas/domain/contracts.hpp>
#include <mosas/runtime/log_event.hpp>
#include <mosas/runtime/output_diagnostics.hpp>
#include <mosas/runtime/processed_frame.hpp>

#include <memory>
#include <string>

namespace mosas::runtime {

// These frame sinks are called only by their dedicated output workers. Their
// record/publish methods may block on encoding or I/O; callers on the control
// path interact only with the worker's bounded non-blocking queue.
class IVideoRecorder {
public:
    virtual ~IVideoRecorder() = default;

    virtual bool start() = 0;
    virtual void reset() noexcept = 0;
    virtual bool record(const ProcessedFrame& frame) = 0;
    virtual bool take_rearm_request() noexcept { return false; }
    virtual void cancel() noexcept = 0;
    virtual void stop() noexcept = 0;
    virtual std::string last_error() const = 0;
};

class ITelemetryRecorder {
public:
    virtual ~ITelemetryRecorder() = default;

    virtual bool start() = 0;
    virtual void reset() noexcept = 0;
    virtual bool record(const ProcessedFrame& frame) = 0;
    virtual bool take_rearm_request() noexcept { return false; }
    virtual void cancel() noexcept = 0;
    virtual void stop() noexcept = 0;
    virtual std::string last_error() const = 0;
};

class IVisualizationSink {
public:
    virtual ~IVisualizationSink() = default;

    virtual bool start() = 0;
    virtual void reset() noexcept = 0;
    virtual bool publish(const ProcessedFrame& frame) = 0;
    virtual void cancel() noexcept = 0;
    virtual void stop() noexcept = 0;
    virtual std::string last_error() const = 0;
};

class IImuRecorder {
public:
    virtual ~IImuRecorder() = default;

    virtual bool start() = 0;
    virtual bool try_record(
        const domain::ImuKinematicSample& sample) noexcept = 0;
    virtual SinkDiagnostics diagnostics() const noexcept = 0;
    virtual void cancel() noexcept = 0;
    virtual void stop() noexcept = 0;
};

class IAsyncLogger {
public:
    virtual ~IAsyncLogger() = default;

    virtual bool start() = 0;
    virtual bool try_submit(const RuntimeLogEvent& event) noexcept = 0;
    virtual SinkDiagnostics diagnostics() const noexcept = 0;
    virtual void stop_and_flush() noexcept = 0;
};

struct RuntimeOutputPorts {
    std::unique_ptr<IVideoRecorder> video;
    std::unique_ptr<ITelemetryRecorder> telemetry;
    std::unique_ptr<IVisualizationSink> visualization;
    std::unique_ptr<IImuRecorder> imu_recorder;
    std::unique_ptr<IAsyncLogger> logger;
};

}  // namespace mosas::runtime

#endif  // MOSAS_RUNTIME_OUTPUT_PORTS_HPP
