#include <mosas/runtime/runtime_supervisor.hpp>

#include "fakes/fake_algorithm_ports.hpp"
#include "fakes/fake_camera_source.hpp"
#include "fakes/fake_clock.hpp"
#include "fakes/fake_command_sink.hpp"
#include "fakes/fake_imu_source.hpp"
#include "test_support.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <future>
#include <memory>
#include <mutex>
#include <sstream>
#include <thread>
#include <utility>
#include <vector>

namespace {

mosas::runtime::CameraFrame make_frame(std::int64_t timestamp_ns,
                                       std::uint8_t pixel_value) {
    auto pixels = std::make_shared<std::vector<std::uint8_t>>(12, pixel_value);
    mosas::runtime::CameraFrame frame{};
    frame.timestamp_ns = timestamp_ns;
    frame.image.owner = pixels;
    frame.image.data = pixels->data();
    frame.image.data_size_bytes = pixels->size();
    frame.image.width = 2;
    frame.image.height = 2;
    frame.image.stride_bytes = 6;
    frame.image.format = mosas::domain::PixelFormat::bgr8;
    return frame;
}

class CapturingFrameSink final : public mosas::runtime::FrameSink {
public:
    struct PublishedFrame {
        mosas::runtime::CameraFrame frame;
        mosas::domain::VisionDetection vision;
        mosas::domain::FlightStateSnapshot state;
        mosas::domain::GuidanceOutput guidance;
        mosas::domain::GuidanceOverlay overlay;
    };

    bool publish(const mosas::runtime::CameraFrame& frame,
                 const mosas::domain::VisionDetection& vision,
                 const mosas::domain::FlightStateSnapshot& state,
                 const mosas::domain::GuidanceOutput& guidance,
                 const mosas::domain::GuidanceOverlay& overlay) override {
        std::lock_guard<std::mutex> lock(mutex_);
        frames_.push_back({frame, vision, state, guidance, overlay});
        return true;
    }

    void reset() noexcept override {
        std::lock_guard<std::mutex> lock(mutex_);
        frames_.clear();
    }

    std::vector<PublishedFrame> frames() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return frames_;
    }

private:
    mutable std::mutex mutex_;
    std::vector<PublishedFrame> frames_;
};

}  // namespace

int main() {
    using namespace mosas;

    constexpr runtime::TimestampNs kImuFirstNs = 1'000'000'000;
    constexpr runtime::TimestampNs kFrameFirstNs = 1'010'000'000;
    constexpr runtime::TimestampNs kFrameSecondNs = 1'015'000'000;
    constexpr runtime::TimestampNs kImuLastNs = 1'020'000'000;

    std::vector<runtime::ImuAngularVelocitySample> imu_samples{
        {kImuFirstNs, {0.0, 0.0, 9.80665}, {0.0, 0.0, 0.0}, 0.0, 0.0},
        {kImuLastNs, {1.0, 2.0, 9.80665}, {0.01, 0.02, 0.03}, 0.25, -0.5},
    };
    auto imu_source =
        std::make_unique<test::FakeImuSource>(std::move(imu_samples));
    auto* imu_source_control = imu_source.get();
    std::vector<runtime::CameraFrame> camera_frames{
        make_frame(kFrameFirstNs, 0x11), make_frame(kFrameSecondNs, 0x22)};
    auto camera_source =
        std::make_unique<test::FakeCameraSource>(std::move(camera_frames));
    auto* camera_source_control = camera_source.get();
    auto command_sink = std::make_unique<test::FakeCommandSink>();
    auto* command_sink_control = command_sink.get();
    auto frame_sink = std::make_unique<CapturingFrameSink>();
    auto* frame_sink_control = frame_sink.get();

    auto vision = std::make_unique<test::FakeVisionDetector>();
    auto* vision_control = vision.get();
    auto flight_state = std::make_unique<test::FakeFlightStateEstimator>();
    auto* flight_state_control = flight_state.get();
    auto los = std::make_unique<test::FakeLosEstimator>();
    auto* los_control = los.get();
    auto rate_filter = std::make_unique<test::FakeLosRateFilter>();
    auto* rate_filter_control = rate_filter.get();
    auto guidance = std::make_unique<test::FakeGuidanceLaw>();
    auto* guidance_control = guidance.get();

    runtime::RuntimeSupervisorDependencies dependencies;
    dependencies.vision_detector = std::move(vision);
    dependencies.flight_state_estimator = std::move(flight_state);
    dependencies.los_estimator = std::move(los);
    dependencies.los_rate_filter = std::move(rate_filter);
    dependencies.guidance_law = std::move(guidance);
    dependencies.monotonic_clock = std::make_unique<test::FakeClock>(77);

    runtime::RuntimeSupervisorConfig config;
    config.launch_speed_mps = 13.0;
    config.history_capacity = 8;
    config.capture_queue_capacity = 2;
    config.output_queue_capacity = 4;
    config.camera_intrinsics = {100.0, 100.0, 1.0, 1.0};

    std::ostringstream log_output;
    {
        runtime::RuntimeSupervisor missing_dependencies(
            std::unique_ptr<runtime::ImuSource>{},
            std::unique_ptr<runtime::CameraSource>{},
            std::unique_ptr<runtime::CommandSink>{}, config,
            runtime::RuntimeSupervisorDependencies{},
            std::unique_ptr<runtime::FrameSink>{}, log_output);
        CHECK(!missing_dependencies.start());
        CHECK(missing_dependencies.last_error() ==
              "runtime dependencies are incomplete");
    }

    runtime::RuntimeSupervisor runtime(
        std::move(imu_source), std::move(camera_source),
        std::move(command_sink), config, std::move(dependencies),
        std::move(frame_sink), log_output);
    CHECK(runtime.start());

    domain::FlightStateSnapshot flight_result{};
    flight_result.attitude.valid = true;
    flight_result.attitude.navigation_from_dart = {};
    flight_result.attitude.corrected_angular_velocity = {0.0, 0.0, 0.0};
    flight_result.launch.phase = domain::LaunchPhase::launched;
    flight_result.launch.transition_time.monotonic_ns = 900'000'000;
    flight_result.launch.transition_time.source_domain =
        domain::ClockDomain::monotonic;
    flight_result.launch.reason = domain::LaunchReason::threshold_confirmed;
    flight_state_control->set_result(flight_result);

    domain::VisionDetection detection{};
    detection.observation.valid = true;
    detection.observation.reason = domain::ObservationReason::found;
    detection.observation.center = {10.0, 20.0};
    detection.diagnostics.bounding_box = {8, 18, 4, 4};
    detection.diagnostics.area_pixels = 12;
    vision_control->push(detection);
    vision_control->push(detection);

    los_control->set_result({true, {0.1}, {-0.2}});
    rate_filter_control->set_result({true, {0.3}, {-0.4}});
    domain::GuidanceOutput guidance_result{};
    guidance_result.valid = true;
    guidance_result.invalid_reason = domain::GuidanceInvalidReason::none;
    guidance_result.command.valid = true;
    guidance_result.command.invalid_reason =
        domain::GuidanceInvalidReason::none;
    guidance_result.command.overload_g = {0.0, 0.0, 0.0};
    guidance_result.diagnostics.body_overload = {0.0, 0.0, 0.0};
    guidance_control->set_result(guidance_result);

    imu_source_control->release();
    const auto state_deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(1);
    bool imu_history_ready = false;
    while (std::chrono::steady_clock::now() < state_deadline) {
        const auto state = runtime.latest_state();
        if (state.has_value() && state->time.monotonic_ns == kImuLastNs) {
            imu_history_ready = true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    CHECK(imu_history_ready);
    CHECK(flight_state_control->call_count() == 2);
    CHECK(flight_state_control->last_timestamp_ns() == kImuLastNs);
    const auto& last_imu_sample = flight_state_control->last_sample();
    CHECK(last_imu_sample.time.monotonic_ns == kImuLastNs);
    CHECK_NEAR(last_imu_sample.acceleration.x, 1.0, 1e-12);
    CHECK_NEAR(last_imu_sample.acceleration.y, 2.0, 1e-12);
    CHECK_NEAR(last_imu_sample.angular_velocity.z, 0.03, 1e-12);
    CHECK_NEAR(last_imu_sample.servo_control_p_normalized, 0.25, 1e-12);
    CHECK_NEAR(last_imu_sample.servo_control_y_normalized, -0.5, 1e-12);
    camera_source_control->release();

    const auto command_deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(1);
    bool valid_command_seen = false;
    bool invalid_zero_command_seen = false;
    while (std::chrono::steady_clock::now() < command_deadline) {
        const auto commands = command_sink_control->commands();
        for (const auto& command : commands) {
            if (command.valid) {
                valid_command_seen = true;
                CHECK(command.generation == 0);
                CHECK(command.source_frame_id == 1);
                CHECK(command.source_time.monotonic_ns == kFrameSecondNs);
                CHECK_NEAR(command.overload_g.x, 0.0, 1e-12);
                CHECK_NEAR(command.overload_g.y, 0.0, 1e-12);
                CHECK_NEAR(command.overload_g.z, 0.0, 1e-12);
            } else if (command.invalid_reason ==
                       domain::GuidanceInvalidReason::stale_input) {
                invalid_zero_command_seen = true;
                CHECK(command.generation == 0);
                CHECK(command.source_time.monotonic_ns == kFrameFirstNs);
                CHECK_NEAR(command.overload_g.x, 0.0, 1e-12);
                CHECK_NEAR(command.overload_g.y, 0.0, 1e-12);
                CHECK_NEAR(command.overload_g.z, 0.0, 1e-12);
            }
        }
        if (valid_command_seen) {
            break;
        }
        CHECK(!runtime.faulted());
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    CHECK(valid_command_seen);
    CHECK(invalid_zero_command_seen);

    const auto published = frame_sink_control->frames();
    CHECK(!published.empty());
    const auto valid_frame =
        std::find_if(published.begin(), published.end(),
                     [](const CapturingFrameSink::PublishedFrame& frame) {
                         return frame.guidance.valid;
                     });
    CHECK(valid_frame != published.end());
    CHECK(valid_frame->frame.source_frame_id == 1);
    CHECK(valid_frame->frame.timestamp_ns == kFrameSecondNs);
    CHECK(valid_frame->state.time.monotonic_ns == kFrameSecondNs);
    CHECK(valid_frame->vision.observation.valid);
    CHECK(valid_frame->guidance.command.valid);
    CHECK(valid_frame->state.attitude.valid);
    CHECK(valid_frame->state.launch.phase == domain::LaunchPhase::launched);
    CHECK(valid_frame->frame.image.is_valid());
    CHECK(guidance_control->call_count() == 1);
    CHECK_NEAR(guidance_control->last_input().speed_mps.value, 13.0, 1e-12);
    CHECK_NEAR(guidance_control->last_input().line_of_sight.q_y.value, 0.1,
               1e-12);
    CHECK_NEAR(valid_frame->guidance.command.overload_g.x, 0.0, 1e-12);
    CHECK_NEAR(valid_frame->guidance.command.overload_g.y, 0.0, 1e-12);
    CHECK_NEAR(valid_frame->guidance.command.overload_g.z, 0.0, 1e-12);
    CHECK(rate_filter_control->call_count() == 2);
    CHECK_NEAR(rate_filter_control->last_dt_seconds(), 0.005, 1e-12);
    CHECK_NEAR(rate_filter_control->last_body_angular_velocity().x, 0.0, 1e-12);
    CHECK(vision_control->call_count() == 2);
    CHECK(los_control->call_count() == 2);
    CHECK(los_control->last_detection().observation.source_frame_id == 1);
    CHECK(los_control->last_detection().observation.time.monotonic_ns ==
          kFrameSecondNs);
    CHECK_NEAR(los_control->last_detection().observation.center.u, 10.0, 1e-12);
    CHECK_NEAR(los_control->last_intrinsics().fx, 100.0, 1e-12);
    CHECK_NEAR(los_control->last_camera_to_dart().matrix()[0], 1.0, 1e-12);
    CHECK(guidance_control->last_input().source_frame_id == 1);
    CHECK(guidance_control->last_input().time.monotonic_ns == kFrameSecondNs);
    CHECK_NEAR(guidance_control->last_input().angular_velocity.q_y.value, 0.3,
               1e-12);
    CHECK(guidance_control->last_input().attitude.valid);
    CHECK(vision_control->last_image_width() == 2);
    CHECK(vision_control->last_image_height() == 2);
    CHECK(vision_control->last_first_pixel() == 0x22);
    const std::thread::id frame_algorithm_thread =
        vision_control->last_thread_id();
    CHECK(frame_algorithm_thread != std::thread::id{});
    CHECK(los_control->last_thread_id() == frame_algorithm_thread);
    CHECK(rate_filter_control->last_thread_id() == frame_algorithm_thread);
    CHECK(guidance_control->last_thread_id() == frame_algorithm_thread);
    CHECK(flight_state_control->last_thread_id() != std::thread::id{});
    CHECK(flight_state_control->last_thread_id() != frame_algorithm_thread);

    const auto statistics = runtime.statistics();
    CHECK(statistics.capture_fps >= 0.0);
    CHECK(statistics.processing_fps >= 0.0);
    CHECK(statistics.output_fps >= 0.0);
    const auto timing = runtime.timing();
    CHECK(timing.capture.samples == 2);
    CHECK(timing.capture_queue_wait.samples == 2);
    CHECK(timing.capture_queue_delay.samples == 2);
    CHECK(timing.prepare.samples == 2);
    CHECK(timing.vision.samples == 2);
    CHECK(timing.line_of_sight.samples == 2);
    CHECK(timing.guidance.samples == 1);
    CHECK(timing.command.samples == 1);
    CHECK(timing.processing_total.samples == 2);
    CHECK(timing.capture_queue_dropped == 0);
    CHECK(timing.output_queue_dropped == 0);

    runtime.stop();
    CHECK(!runtime.faulted());
    CHECK(!runtime.running());
    runtime.stop();
    CHECK(command_sink_control->stop_count() == 1);

    CHECK(runtime.start());
    CHECK(!runtime.latest_state().has_value());
    flight_state_control->set_result(flight_result);
    vision_control->push(detection);
    vision_control->push(detection);
    los_control->set_result({true, {0.1}, {-0.2}});
    rate_filter_control->set_result({true, {0.3}, {-0.4}});
    guidance_control->set_result(guidance_result);
    imu_source_control->release();

    const auto restart_state_deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(1);
    bool restarted_state_ready = false;
    while (std::chrono::steady_clock::now() < restart_state_deadline) {
        const auto state = runtime.latest_state();
        if (state.has_value() && state->time.monotonic_ns == kImuLastNs) {
            restarted_state_ready = true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    CHECK(restarted_state_ready);
    camera_source_control->release();

    const auto restart_command_deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(1);
    bool restarted_command_seen = false;
    while (std::chrono::steady_clock::now() < restart_command_deadline) {
        for (const auto& command : command_sink_control->commands()) {
            if (command.valid) {
                CHECK(command.generation == 0);
                CHECK(command.source_frame_id == 1);
                CHECK(command.source_time.monotonic_ns == kFrameSecondNs);
                restarted_command_seen = true;
            }
        }
        if (restarted_command_seen) {
            break;
        }
        CHECK(!runtime.faulted());
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    CHECK(restarted_command_seen);

    const auto restart_output_deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(1);
    bool restarted_frame_seen = false;
    while (std::chrono::steady_clock::now() < restart_output_deadline) {
        for (const auto& frame : frame_sink_control->frames()) {
            if (frame.guidance.valid) {
                CHECK(frame.frame.source_frame_id == 1);
                CHECK(frame.frame.timestamp_ns == kFrameSecondNs);
                restarted_frame_seen = true;
            }
        }
        if (restarted_frame_seen) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    CHECK(restarted_frame_seen);

    CHECK(camera_source_control->wait_until_capture_waiting(
        std::chrono::seconds(1)));
    camera_source_control->defer_capture_return_after_cancel();
    auto cancellation =
        std::async(std::launch::async, [&runtime] { runtime.cancel(); });
    const bool cancellation_returned_without_join =
        cancellation.wait_for(std::chrono::seconds(1)) ==
        std::future_status::ready;
    camera_source_control->release_cancelled_capture();
    cancellation.get();
    CHECK(cancellation_returned_without_join);
    CHECK(runtime.running());
    CHECK(command_sink_control->cancel_count() == 1);
    runtime.stop();
    CHECK(!runtime.running());
    CHECK(!runtime.faulted());
    CHECK(command_sink_control->stop_count() == 2);
    return 0;
}
