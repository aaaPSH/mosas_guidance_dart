#include <mosas/runtime/runtime_interfaces.hpp>
#include <mosas/runtime/runtime_types.hpp>

#include <cassert>

namespace {

class EmptyImuSource final : public mosas::runtime::ImuSource {
public:
    bool read(mosas::runtime::ImuSample* sample) override {
        (void)sample;
        return false;
    }

    void cancel() noexcept override {}
};

class EmptyCameraSource final : public mosas::runtime::CameraSource {
public:
    bool configure(const mosas::runtime::CameraCaptureConfig&) override {
        return true;
    }

    bool capture(mosas::runtime::CameraFrame* frame) override {
        (void)frame;
        return false;
    }

    void cancel() noexcept override {}
};

class EmptyCommandSink final : public mosas::runtime::CommandSink {
public:
    bool send(const mosas::runtime::GuidanceCommand&) override {
        return true;
    }
};

void test_runtime_types_are_constructible() {
    const mosas::runtime::ImuSample imu{
        100, {0.0, 9.80665, 0.0}, {0.0, 0.0, 0.0}};
    const mosas::runtime::CameraFrame frame{200, cv::Mat()};
    assert(imu.timestamp_ns < frame.timestamp_ns);
    assert(mosas::runtime::FlightPhase::pre_launch !=
           mosas::runtime::FlightPhase::free_flight);
}

}  // namespace

int main() {
    test_runtime_types_are_constructible();
    EmptyImuSource imu_source;
    EmptyCameraSource camera_source;
    EmptyCommandSink command_sink;
    (void)imu_source;
    (void)camera_source;
    (void)command_sink;
    return 0;
}
