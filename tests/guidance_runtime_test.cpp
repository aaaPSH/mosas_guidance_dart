#include <mosas/runtime/flight_phase_detector.hpp>
#include <mosas/runtime/imu_state_history.hpp>
#include <mosas/runtime/runtime_interfaces.hpp>
#include <mosas/runtime/runtime_types.hpp>

#include <cassert>
#include <cmath>

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

void test_non_horizontal_static_window_initializes_from_three_axis_baseline() {
    mosas::runtime::FlightPhaseDetectorConfig config;
    config.min_static_samples = 3;
    config.min_static_duration_ns = 2;
    config.max_static_axis_variation = 0.2;
    mosas::runtime::FlightPhaseDetector detector(config);

    assert(detector.update({0, {3.0, 8.0, -4.0}, {0.0, 0.0, 0.0}}) ==
           mosas::runtime::FlightPhase::pre_launch);
    assert(detector.update({1, {3.0, 8.0, -4.0}, {0.0, 0.0, 0.0}}) ==
           mosas::runtime::FlightPhase::pre_launch);
    assert(detector.update({2, {3.0, 8.0, -4.0}, {0.0, 0.0, 0.0}}) ==
           mosas::runtime::FlightPhase::pre_launch);
    const Vector3 baseline = detector.baseline_acceleration();
    assert(std::abs(baseline.x - 3.0) < 1e-12);
    assert(std::abs(baseline.y - 8.0) < 1e-12);
    assert(std::abs(baseline.z + 4.0) < 1e-12);
    assert(detector.initialized());
}

void test_short_elastic_impulse_transitions_to_ejection_then_free_flight() {
    mosas::runtime::FlightPhaseDetectorConfig config;
    config.min_static_samples = 2;
    config.min_static_duration_ns = 1;
    config.max_static_axis_variation = 0.2;
    config.launch_direction = 1;
    config.ejection_start_threshold = 20.0;
    config.ejection_confirm_duration_ns = 2;
    config.free_flight_release_threshold = 3.0;
    config.free_flight_confirm_duration_ns = 2;
    mosas::runtime::FlightPhaseDetector detector(config);

    detector.update({0, {0.0, 9.8, 0.0}, {0.0, 0.0, 0.0}});
    detector.update({1, {0.0, 9.8, 0.0}, {0.0, 0.0, 0.0}});
    assert(detector.update({2, {21.0, 9.8, 0.0}, {0.0, 0.0, 0.0}}) ==
           mosas::runtime::FlightPhase::pre_launch);
    assert(detector.update({4, {21.0, 9.8, 0.0}, {0.0, 0.0, 0.0}}) ==
           mosas::runtime::FlightPhase::ejection);
    assert(detector.update({5, {1.0, 9.8, 0.0}, {0.0, 0.0, 0.0}}) ==
           mosas::runtime::FlightPhase::ejection);
    assert(detector.update({7, {1.0, 9.8, 0.0}, {0.0, 0.0, 0.0}}) ==
           mosas::runtime::FlightPhase::free_flight);
    assert(detector.update({8, {0.0, 9.8, 0.0}, {0.0, 0.0, 0.0}}) ==
           mosas::runtime::FlightPhase::free_flight);
}

void test_light_acceleration_does_not_trigger_launch() {
    mosas::runtime::FlightPhaseDetectorConfig config;
    config.min_static_samples = 2;
    config.min_static_duration_ns = 1;
    config.max_static_axis_variation = 0.2;
    config.ejection_start_threshold = 20.0;
    config.ejection_confirm_duration_ns = 3;
    mosas::runtime::FlightPhaseDetector detector(config);

    detector.update({0, {0.0, 9.8, 0.0}, {0.0, 0.0, 0.0}});
    detector.update({1, {0.0, 9.8, 0.0}, {0.0, 0.0, 0.0}});
    detector.update({2, {10.0, 9.8, 0.0}, {0.0, 0.0, 0.0}});
    detector.update({3, {0.0, 9.8, 0.0}, {0.0, 0.0, 0.0}});
    assert(detector.phase() == mosas::runtime::FlightPhase::pre_launch);
}

void test_history_returns_latest_snapshot_not_after_frame_timestamp() {
    mosas::runtime::ImuStateHistory history(2);
    history.push({10, mosas::runtime::FlightPhase::pre_launch,
                  {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0},
                  {0.0, 9.8, 0.0}, {0.0, 0.0, 0.0}});
    history.push({20, mosas::runtime::FlightPhase::ejection,
                  {0.1, 0.0, 0.0}, {2.0, 0.0, 0.0},
                  {20.0, 9.8, 0.0}, {0.0, 0.0, 0.0}});

    const auto result = history.find_at_or_before(19);
    assert(result.has_value());
    assert(result->timestamp_ns == 10);
    assert(!history.find_at_or_before(9).has_value());
}

void test_history_evicts_oldest_snapshot_at_capacity() {
    mosas::runtime::ImuStateHistory history(2);
    for (mosas::runtime::TimestampNs timestamp = 1; timestamp <= 3;
         ++timestamp) {
        history.push({timestamp, mosas::runtime::FlightPhase::pre_launch,
                      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
                      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}});
    }
    assert(history.size() == 2);
    assert(!history.find_at_or_before(1).has_value());
    assert(history.find_at_or_before(3)->timestamp_ns == 3);
}

}  // namespace

int main() {
    test_runtime_types_are_constructible();
    test_non_horizontal_static_window_initializes_from_three_axis_baseline();
    test_short_elastic_impulse_transitions_to_ejection_then_free_flight();
    test_light_acceleration_does_not_trigger_launch();
    test_history_returns_latest_snapshot_not_after_frame_timestamp();
    test_history_evicts_oldest_snapshot_at_capacity();
    EmptyImuSource imu_source;
    EmptyCameraSource camera_source;
    EmptyCommandSink command_sink;
    (void)imu_source;
    (void)camera_source;
    (void)command_sink;
    return 0;
}
