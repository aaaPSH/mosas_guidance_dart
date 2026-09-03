#ifndef MOSAS_RUNTIME_INTERFACES_HPP
#define MOSAS_RUNTIME_INTERFACES_HPP

#include <mosas/runtime/runtime_types.hpp>
#include <mosas/vision/vision_types.hpp>

namespace mosas::runtime {

class ImuSource {
public:
    virtual ~ImuSource() = default;

    virtual bool read(ImuSample* sample) = 0;
    virtual void cancel() noexcept = 0;
};

enum class CameraCaptureMode {
    free_run,
    hardware_trigger,
};

struct CameraCaptureConfig {
    CameraCaptureMode mode = CameraCaptureMode::free_run;
};

class CameraSource {
public:
    virtual ~CameraSource() = default;

    virtual bool configure(const CameraCaptureConfig& config) = 0;
    virtual bool capture(CameraFrame* frame) = 0;
    virtual void cancel() noexcept = 0;
};

class CommandSink {
public:
    virtual ~CommandSink() = default;

    virtual bool send(const GuidanceCommand& command) = 0;
};

class FrameSink {
public:
    virtual ~FrameSink() = default;

    virtual bool publish(const CameraFrame& frame,
                         const VisionResult& vision_result,
                         const ImuStateSnapshot& imu_state,
                         const PngGuidanceOutput& guidance) = 0;

    // 携带视线角和视线角速度，供可视化接收；旧接口保持兼容。
    virtual bool publish(const CameraFrame& frame,
                         const VisionResult& vision_result,
                         const ImuStateSnapshot& imu_state,
                         const PngGuidanceOutput& guidance,
                         const VisionOverlayData& overlay) {
        (void)overlay;
        return publish(frame, vision_result, imu_state, guidance);
    }
};

}  // namespace mosas::runtime

#endif  // MOSAS_RUNTIME_INTERFACES_HPP
