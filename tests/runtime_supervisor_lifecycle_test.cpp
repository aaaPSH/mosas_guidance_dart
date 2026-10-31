#include <mosas/runtime/runtime_supervisor.hpp>

#include "test_support.hpp"
#include "command_output_test_support.hpp"

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <exception>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace {

using namespace std::chrono_literals;

class EventLog {
public:
    void push(std::string event) {
        std::lock_guard<std::mutex> lock(mutex_);
        events_.push_back(std::move(event));
    }

    std::size_t index_of(const std::string& event) const {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto found = std::find(events_.begin(), events_.end(), event);
        return static_cast<std::size_t>(std::distance(events_.begin(), found));
    }

private:
    mutable std::mutex mutex_;
    std::vector<std::string> events_;
};

mosas::runtime::CameraFrame
make_frame(mosas::runtime::TimestampNs timestamp_ns) {
    auto pixels = std::make_shared<std::vector<std::uint8_t>>(3, 0x44);
    mosas::runtime::CameraFrame frame{};
    frame.timestamp_ns = timestamp_ns;
    frame.image.owner = pixels;
    frame.image.data = pixels->data();
    frame.image.data_size_bytes = pixels->size();
    frame.image.width = 1;
    frame.image.height = 1;
    frame.image.stride_bytes = 3;
    frame.image.format = mosas::domain::PixelFormat::bgr8;
    return frame;
}

class ProbeCameraSource final : public mosas::runtime::CameraSource {
public:
    explicit ProbeCameraSource(EventLog& events, bool sequence_time = false)
        : events_(events), sequence_time_(sequence_time) {}

    bool uses_sequence_time() const noexcept override { return sequence_time_; }
    void bind_acquisition_sync(std::shared_ptr<mosas::domain::AcquisitionSync> sync) override {
        acquisition_sync_ = std::move(sync);
    }
    /** @return Whether configure-time readiness survived runtime startup. */
    bool acquisition_ready() const noexcept { return acquisition_sync_->state().ready; }
    /** @return Whether a previous session's launch was still latched when configure began. */
    bool launched_when_configured() const noexcept { return launched_when_configured_; }

    void set_configure_result(mosas::runtime::SourceResult result) {
        std::lock_guard<std::mutex> lock(mutex_);
        configure_result_ = std::move(result);
    }

    mosas::runtime::SourceResult
    configure(const mosas::runtime::CameraCaptureConfig&) override {
        std::lock_guard<std::mutex> lock(mutex_);
        ++configure_count_;
        cancelled_ = false;
        failure_requested_ = false;
        frame_ready_ = false;
        events_.push("camera.configure");
        if (sequence_time_) {
            const auto state = acquisition_sync_->state();
            launched_when_configured_ = state.launched;
            // Model an SDK source finishing synchronization before configure returns.
            acquisition_sync_->ready(1, state.epoch);
        }
        return configure_result_;
    }

    mosas::runtime::SourceResult
    capture(mosas::runtime::CameraFrame* frame) override {
        std::unique_lock<std::mutex> lock(mutex_);
        capture_active_ = true;
        events_.push("camera.capture.enter");
        condition_.notify_all();
        condition_.wait(lock, [this] {
            return cancelled_ || failure_requested_ || frame_ready_;
        });
        if (failure_requested_) {
            failure_requested_ = false;
            capture_active_ = false;
            events_.push("camera.capture.failure");
            condition_.notify_all();
            return {mosas::runtime::SourceStatus::fatal,
                    "injected camera failure"};
        }
        if (cancelled_) {
            capture_active_ = false;
            events_.push("camera.capture.cancelled");
            condition_.notify_all();
            return {mosas::runtime::SourceStatus::cancelled, {}};
        }
        frame_ready_ = false;
        capture_active_ = false;
        *frame = make_frame(1'010'000'000);
        events_.push("camera.capture.frame");
        condition_.notify_all();
        return {mosas::runtime::SourceStatus::ok, {}};
    }

    void cancel() noexcept override {
        std::lock_guard<std::mutex> lock(mutex_);
        ++cancel_count_;
        cancelled_ = true;
        events_.push("camera.cancel");
        condition_.notify_all();
    }

    void stop() noexcept override {
        std::lock_guard<std::mutex> lock(mutex_);
        ++stop_count_;
        stopped_while_capturing_ = capture_active_;
        events_.push("camera.stop");
    }

    void release_frame() noexcept {
        std::lock_guard<std::mutex> lock(mutex_);
        frame_ready_ = true;
        condition_.notify_all();
    }

    void fail_capture() noexcept {
        std::lock_guard<std::mutex> lock(mutex_);
        failure_requested_ = true;
        condition_.notify_all();
    }

    bool wait_until_capturing(std::chrono::milliseconds timeout) {
        std::unique_lock<std::mutex> lock(mutex_);
        return condition_.wait_for(lock, timeout,
                                   [this] { return capture_active_; });
    }

    std::size_t configure_count() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return configure_count_;
    }
    std::size_t cancel_count() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return cancel_count_;
    }
    std::size_t stop_count() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return stop_count_;
    }
    bool stopped_while_capturing() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return stopped_while_capturing_;
    }

private:
    EventLog& events_;
    const bool sequence_time_;
    std::shared_ptr<mosas::domain::AcquisitionSync> acquisition_sync_;
    bool launched_when_configured_ = false;
    mutable std::mutex mutex_;
    std::condition_variable condition_;
    mosas::runtime::SourceResult configure_result_{
        mosas::runtime::SourceStatus::ok, {}};
    std::size_t configure_count_ = 0;
    std::size_t cancel_count_ = 0;
    std::size_t stop_count_ = 0;
    bool capture_active_ = false;
    bool cancelled_ = false;
    bool failure_requested_ = false;
    bool frame_ready_ = false;
    bool stopped_while_capturing_ = false;
};

class ProbeImuSource final : public mosas::runtime::ImuSource {
public:
    ProbeImuSource(
        EventLog& events,
        std::vector<mosas::runtime::ImuAngularVelocitySample> samples)
        : events_(events), samples_(std::move(samples)) {}

    void set_start_result(mosas::runtime::SourceResult result) {
        std::lock_guard<std::mutex> lock(mutex_);
        start_result_ = std::move(result);
    }

    mosas::runtime::SourceResult start() override {
        std::lock_guard<std::mutex> lock(mutex_);
        ++start_count_;
        started_ = start_result_.status == mosas::runtime::SourceStatus::ok;
        cancelled_ = false;
        released_count_ = 0;
        next_sample_ = 0;
        events_.push("imu.start");
        return start_result_;
    }

    bool try_pop(
        mosas::runtime::ImuAngularVelocitySample* sample) noexcept override {
        if (sample == nullptr) {
            return false;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        if (!started_ || cancelled_ || next_sample_ >= released_count_) {
            return false;
        }
        *sample = samples_[next_sample_++];
        return true;
    }

    bool wait_for_data() noexcept override {
        std::unique_lock<std::mutex> lock(mutex_);
        waiting_ = true;
        events_.push("imu.wait.enter");
        condition_.notify_all();
        condition_.wait(lock, [this] {
            return cancelled_ || next_sample_ < released_count_;
        });
        waiting_ = false;
        events_.push("imu.wait.exit");
        condition_.notify_all();
        return started_ && !cancelled_;
    }

    void cancel() noexcept override {
        std::lock_guard<std::mutex> lock(mutex_);
        ++cancel_count_;
        cancelled_ = true;
        events_.push("imu.cancel");
        condition_.notify_all();
    }

    void stop() noexcept override {
        std::lock_guard<std::mutex> lock(mutex_);
        ++stop_count_;
        stopped_while_waiting_ = waiting_;
        started_ = false;
        events_.push("imu.stop");
    }

    void release_samples(std::size_t count) noexcept {
        std::lock_guard<std::mutex> lock(mutex_);
        released_count_ = std::min(count, samples_.size());
        condition_.notify_all();
    }

    bool wait_until_waiting(std::chrono::milliseconds timeout) {
        std::unique_lock<std::mutex> lock(mutex_);
        return condition_.wait_for(lock, timeout, [this] { return waiting_; });
    }

    std::size_t start_count() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return start_count_;
    }
    std::size_t cancel_count() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return cancel_count_;
    }
    std::size_t stop_count() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return stop_count_;
    }
    bool stopped_while_waiting() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return stopped_while_waiting_;
    }

private:
    EventLog& events_;
    mutable std::mutex mutex_;
    std::condition_variable condition_;
    std::vector<mosas::runtime::ImuAngularVelocitySample> samples_;
    mosas::runtime::SourceResult start_result_{mosas::runtime::SourceStatus::ok,
                                               {}};
    std::size_t next_sample_ = 0;
    std::size_t released_count_ = 0;
    std::size_t start_count_ = 0;
    std::size_t cancel_count_ = 0;
    std::size_t stop_count_ = 0;
    bool started_ = false;
    bool cancelled_ = false;
    bool waiting_ = false;
    bool stopped_while_waiting_ = false;
};

class ProbeCommandSink final : public mosas::runtime::CommandSink {
public:
    explicit ProbeCommandSink(EventLog& events) : events_(events) {}

    void set_start_succeeds(bool succeeds) noexcept {
        start_succeeds_ = succeeds;
    }

    bool start() override {
        std::lock_guard<std::mutex> lock(mutex_);
        ++start_count_;
        active_generation_ = 0;
        commands_.clear();
        events_.push("command.start");
        return start_succeeds_;
    }

    bool try_submit(
        const mosas::runtime::GuidanceCommand& command) noexcept override {
        std::lock_guard<std::mutex> lock(mutex_);
        if (command.generation != active_generation_) {
            return false;
        }
        commands_.push_back(command);
        return true;
    }

    void set_active_generation(std::uint64_t generation) noexcept override {
        std::lock_guard<std::mutex> lock(mutex_);
        active_generation_ = generation;
    }

    void cancel() noexcept override {
        std::lock_guard<std::mutex> lock(mutex_);
        ++cancel_count_;
        events_.push("command.cancel");
    }

    void stop() noexcept override {
        std::lock_guard<std::mutex> lock(mutex_);
        ++stop_count_;
        events_.push("command.stop");
    }

    std::size_t start_count() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return start_count_;
    }
    std::size_t cancel_count() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return cancel_count_;
    }
    std::size_t stop_count() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return stop_count_;
    }
    std::uint64_t active_generation() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return active_generation_;
    }
    std::vector<mosas::runtime::GuidanceCommand> commands() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return commands_;
    }

private:
    EventLog& events_;
    mutable std::mutex mutex_;
    std::uint64_t active_generation_ = 0;
    std::vector<mosas::runtime::GuidanceCommand> commands_;
    std::size_t start_count_ = 0;
    std::size_t cancel_count_ = 0;
    std::size_t stop_count_ = 0;
    bool start_succeeds_ = true;
};

class ProbeVideoRecorder final : public mosas::runtime::IVideoRecorder {
public:
    explicit ProbeVideoRecorder(EventLog& events) : events_(events) {}

    void set_publish_blocked(bool blocked) noexcept {
        std::lock_guard<std::mutex> lock(mutex_);
        block_publish_ = blocked;
    }
    void set_publish_failure(bool fails) noexcept {
        std::lock_guard<std::mutex> lock(mutex_);
        fail_publish_ = fails;
    }
    void set_reported_error(std::string error) {
        std::lock_guard<std::mutex> lock(mutex_);
        reported_error_ = std::move(error);
    }

    bool start() override { return true; }

    bool record(const mosas::runtime::ProcessedFrame&) override {
        std::unique_lock<std::mutex> lock(mutex_);
        publish_active_ = true;
        ++publish_count_;
        events_.push("sink.publish.enter");
        condition_.notify_all();
        if (block_publish_) {
            condition_.wait(lock,
                            [this] { return cancelled_ || publish_released_; });
        }
        const bool succeeds = !fail_publish_;
        if (!succeeds) {
            last_error_ = "injected sink publish failure";
        }
        if (rearm_on_publish_) {
            rearm_on_publish_ = false;
            rearm_pending_ = true;
        }
        publish_active_ = false;
        events_.push("sink.publish.exit");
        condition_.notify_all();
        return succeeds;
    }

    void cancel() noexcept override {
        std::lock_guard<std::mutex> lock(mutex_);
        ++cancel_count_;
        cancelled_ = true;
        events_.push("sink.cancel");
        condition_.notify_all();
    }

    void reset() noexcept override {
        std::lock_guard<std::mutex> lock(mutex_);
        cancelled_ = false;
        publish_released_ = false;
        rearm_pending_ = false;
        events_.push("sink.reset");
    }

    void stop() noexcept override {
        std::lock_guard<std::mutex> lock(mutex_);
        ++stop_count_;
        stopped_while_publishing_ = publish_active_;
        events_.push("sink.stop");
    }

    bool take_rearm_request() noexcept override {
        std::lock_guard<std::mutex> lock(mutex_);
        const bool pending = rearm_pending_;
        rearm_pending_ = false;
        return pending;
    }

    std::string last_error() const override {
        std::lock_guard<std::mutex> lock(mutex_);
        return last_error_.empty() ? reported_error_ : last_error_;
    }

    void request_rearm_after_publish() noexcept {
        std::lock_guard<std::mutex> lock(mutex_);
        rearm_on_publish_ = true;
    }

    bool wait_for_publish(std::chrono::milliseconds timeout) {
        std::unique_lock<std::mutex> lock(mutex_);
        return condition_.wait_for(lock, timeout,
                                   [this] { return publish_count_ > 0; });
    }

    bool wait_for_publish_to_finish(std::chrono::milliseconds timeout) {
        std::unique_lock<std::mutex> lock(mutex_);
        return condition_.wait_for(lock, timeout,
                                   [this] { return !publish_active_; });
    }

    std::size_t publish_count() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return publish_count_;
    }
    std::size_t cancel_count() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return cancel_count_;
    }
    std::size_t stop_count() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return stop_count_;
    }
    bool stopped_while_publishing() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return stopped_while_publishing_;
    }

private:
    EventLog& events_;
    mutable std::mutex mutex_;
    std::condition_variable condition_;
    std::string last_error_;
    std::string reported_error_;
    std::size_t publish_count_ = 0;
    std::size_t cancel_count_ = 0;
    std::size_t stop_count_ = 0;
    bool publish_active_ = false;
    bool block_publish_ = false;
    bool fail_publish_ = false;
    bool publish_released_ = false;
    bool cancelled_ = false;
    bool stopped_while_publishing_ = false;
    bool rearm_on_publish_ = false;
    bool rearm_pending_ = false;
};

class ProbeVisionDetector final : public mosas::ports::IVisionDetector {
public:
    void set_throws(bool throws) noexcept { throws_ = throws; }

    mosas::domain::VisionDetection
    detect(const mosas::domain::ImageView&, std::uint64_t,
           const mosas::domain::SensorTime&) override {
        if (throws_) {
            throw std::runtime_error("injected vision failure");
        }
        return {};
    }

    void reset() noexcept override {}

private:
    bool throws_ = false;
};

class ProbeFlightStateEstimator final
    : public mosas::ports::IFlightStateEstimator {
public:
    ProbeFlightStateEstimator() {
        result_.attitude.valid = true;
        result_.attitude.navigation_from_dart = {};
        result_.launch.phase = mosas::domain::LaunchPhase::pre_launch;
    }

    mosas::domain::FlightStateSnapshot
    update(const mosas::domain::ImuKinematicSample&) override {
        return result_;
    }

    void reset() noexcept override {}
    void set_launched() noexcept {
        result_.launch.phase = mosas::domain::LaunchPhase::launched;
        result_.launch.transition_time.monotonic_ns = 900'000'000;
        result_.launch.reason =
            mosas::domain::LaunchReason::threshold_confirmed;
    }

private:
    mosas::domain::FlightStateSnapshot result_{};
};

class ProbeLosEstimator final : public mosas::ports::ILosEstimator {
public:
    mosas::domain::DartLineOfSight
    estimate(const mosas::domain::VisionDetection&,
             const mosas::domain::CameraIntrinsics&,
             const mosas::domain::Rotation3<mosas::domain::DartFrameTag,
                                            mosas::domain::CameraFrameTag>&)
        override {
        return {};
    }
};

class ProbeLosRateFilter final : public mosas::ports::ILosRateFilter {
public:
    mosas::domain::LosRateEstimate
    update(const mosas::domain::LosRateFilterInput&) override {
        return {};
    }

    mosas::domain::LineOfSightRateSample sample() const override { return {}; }
    void reset() noexcept override {}
};

class ProbeGuidanceLaw final : public mosas::ports::IGuidanceLaw {
public:
    mosas::domain::GuidanceOutput
    calculate(const mosas::domain::GuidanceInput&) override {
        return {};
    }
};

class ProbeClock final : public mosas::ports::IMonotonicClock {
public:
    std::int64_t now_ns() const noexcept override { return 123; }
    mosas::domain::ClockDomain clock_domain() const noexcept override { return mosas::domain::ClockDomain::monotonic; }
};

std::vector<mosas::runtime::ImuAngularVelocitySample> valid_samples() {
    return {{1'000'000'000, {0.0, 0.0, 9.80665}, {0.0, 0.0, 0.0}, 0.0, 0.0},
            {1'020'000'000, {0.0, 0.0, 9.80665}, {0.0, 0.0, 0.0}, 0.0, 0.0}};
}

struct RuntimeFixture {
    explicit RuntimeFixture(
        std::vector<mosas::runtime::ImuAngularVelocitySample> samples = {},
        bool sequence_camera = false) {
        auto camera_source = std::make_unique<ProbeCameraSource>(events, sequence_camera);
        camera = camera_source.get();
        auto imu_source =
            std::make_unique<ProbeImuSource>(events, std::move(samples));
        imu = imu_source.get();
        auto command_sink = std::make_unique<ProbeCommandSink>(events);
        command = command_sink.get();
        auto video_recorder = std::make_unique<ProbeVideoRecorder>(events);
        sink = video_recorder.get();
        auto vision_detector = std::make_unique<ProbeVisionDetector>();
        vision = vision_detector.get();

        auto dependencies = mosas::test::mapped_dependencies();
        dependencies.vision_detector = std::move(vision_detector);
        auto flight_state_estimator =
            std::make_unique<ProbeFlightStateEstimator>();
        flight_state = flight_state_estimator.get();
        dependencies.flight_state_estimator = std::move(flight_state_estimator);
        dependencies.los_estimator = std::make_unique<ProbeLosEstimator>();
        dependencies.los_rate_filter = std::make_unique<ProbeLosRateFilter>();
        dependencies.guidance_law = std::make_unique<ProbeGuidanceLaw>();
        dependencies.monotonic_clock = std::make_unique<ProbeClock>();

        mosas::runtime::RuntimeSupervisorConfig config;
        config.capture_queue_capacity = 4;
        config.output_queue_capacity = 4;
        mosas::runtime::RuntimeOutputPorts output_ports;
        output_ports.video = std::move(video_recorder);
        runtime = std::make_unique<mosas::runtime::RuntimeSupervisor>(
            std::move(imu_source), std::move(camera_source),
            std::move(command_sink), config, std::move(dependencies),
            std::move(output_ports));
    }

    bool wait_for_fault(std::chrono::milliseconds timeout = 1s) const {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (std::chrono::steady_clock::now() < deadline) {
            if (runtime->faulted()) {
                return true;
            }
            std::this_thread::yield();
        }
        return runtime->faulted();
    }

    bool wait_for_latest_state(mosas::runtime::TimestampNs timestamp,
                               std::chrono::milliseconds timeout = 1s) const {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (std::chrono::steady_clock::now() < deadline) {
            const auto state = runtime->latest_state();
            if (state.has_value() && state->time.monotonic_ns == timestamp) {
                return true;
            }
            std::this_thread::yield();
        }
        return false;
    }

    EventLog events;
    ProbeCameraSource* camera = nullptr;
    ProbeImuSource* imu = nullptr;
    ProbeCommandSink* command = nullptr;
    ProbeVideoRecorder* sink = nullptr;
    ProbeVisionDetector* vision = nullptr;
    ProbeFlightStateEstimator* flight_state = nullptr;
    std::unique_ptr<mosas::runtime::RuntimeSupervisor> runtime;
};

/** @return Zero if startup and restart reset the barrier before callbacks can publish readiness. */
int test_configure_time_acquisition_readiness_survives_start() {
    RuntimeFixture fixture(valid_samples(), true);
    fixture.flight_state->set_launched();
    for (int session = 0; session < 2; ++session) {
        CHECK(fixture.runtime->start());
        CHECK(!fixture.camera->launched_when_configured());
        CHECK(fixture.camera->acquisition_ready());
        fixture.imu->release_samples(2);
        CHECK(fixture.wait_for_latest_state(1'020'000'000));
        CHECK(fixture.runtime->latest_state()->launch.phase == mosas::domain::LaunchPhase::launched);
        fixture.runtime->stop();
        CHECK(!fixture.runtime->faulted());
    }
    return 0;
}

int test_partial_start_rollback_and_idempotent_stop() {
    RuntimeFixture fixture;
    fixture.imu->set_start_result(
        {mosas::runtime::SourceStatus::fatal, "injected IMU start failure"});

    CHECK(!fixture.runtime->start());
    CHECK(fixture.runtime->faulted());
    CHECK(fixture.runtime->last_error() ==
          "imu source start fatal: injected IMU start failure");
    CHECK(fixture.camera->configure_count() == 1);
    CHECK(fixture.imu->start_count() == 1);
    CHECK(fixture.command->start_count() == 0);
    CHECK(fixture.camera->stop_count() == 1);
    CHECK(fixture.imu->stop_count() == 1);
    CHECK(fixture.command->stop_count() == 1);
    CHECK(fixture.sink->stop_count() == 1);
    CHECK(!fixture.camera->stopped_while_capturing());
    CHECK(!fixture.imu->stopped_while_waiting());
    CHECK(!fixture.sink->stopped_while_publishing());
    CHECK(fixture.events.index_of("camera.configure") <
          fixture.events.index_of("imu.start"));
    CHECK(fixture.events.index_of("imu.cancel") <
          fixture.events.index_of("camera.stop"));
    CHECK(fixture.events.index_of("camera.stop") <
          fixture.events.index_of("imu.stop"));
    CHECK(fixture.events.index_of("imu.stop") <
          fixture.events.index_of("sink.stop"));
    CHECK(fixture.events.index_of("sink.stop") <
          fixture.events.index_of("command.stop"));

    fixture.runtime->stop();
    fixture.runtime->stop();
    CHECK(fixture.camera->stop_count() == 1);
    CHECK(fixture.imu->stop_count() == 1);
    CHECK(fixture.command->stop_count() == 1);
    CHECK(fixture.sink->stop_count() == 1);

    RuntimeFixture camera_config_failure;
    camera_config_failure.camera->set_configure_result(
        {mosas::runtime::SourceStatus::fatal,
         "injected camera configure failure"});
    CHECK(!camera_config_failure.runtime->start());
    CHECK(camera_config_failure.runtime->last_error() ==
          "camera source configure fatal: injected camera configure failure");
    CHECK(camera_config_failure.imu->start_count() == 0);
    CHECK(camera_config_failure.command->start_count() == 0);
    CHECK(camera_config_failure.camera->stop_count() == 1);
    CHECK(camera_config_failure.imu->stop_count() == 1);
    CHECK(camera_config_failure.sink->stop_count() == 1);
    return 0;
}

int test_command_sink_start_failure_rolls_back() {
    RuntimeFixture fixture;
    fixture.command->set_start_succeeds(false);

    CHECK(!fixture.runtime->start());
    CHECK(fixture.runtime->faulted());
    CHECK(fixture.runtime->last_error() == "command sink start failed");
    CHECK(fixture.imu->start_count() == 1);
    CHECK(fixture.command->start_count() == 1);
    CHECK(fixture.camera->stop_count() == 1);
    CHECK(fixture.imu->stop_count() == 1);
    CHECK(fixture.command->stop_count() == 1);
    CHECK(fixture.sink->stop_count() == 1);
    CHECK(fixture.events.index_of("command.start") <
          fixture.events.index_of("camera.stop"));
    CHECK(fixture.events.index_of("camera.stop") <
          fixture.events.index_of("imu.stop"));
    CHECK(fixture.events.index_of("imu.stop") <
          fixture.events.index_of("sink.stop"));
    return 0;
}

int test_camera_fault_preserves_first_error() {
    RuntimeFixture fixture;
    fixture.sink->set_reported_error("secondary sink cleanup failure");
    CHECK(fixture.runtime->start());
    CHECK(fixture.camera->wait_until_capturing(1s));
    fixture.camera->fail_capture();
    CHECK(fixture.wait_for_fault());
    CHECK(fixture.runtime->last_error() ==
          "camera source capture fatal: injected camera failure");

    fixture.runtime->stop();
    CHECK(fixture.runtime->last_error() ==
          "camera source capture fatal: injected camera failure");
    CHECK(!fixture.camera->stopped_while_capturing());
    CHECK(!fixture.imu->stopped_while_waiting());
    CHECK(!fixture.sink->stopped_while_publishing());
    CHECK(fixture.camera->cancel_count() > 0);
    CHECK(fixture.imu->cancel_count() > 0);
    CHECK(fixture.events.index_of("camera.capture.failure") <
          fixture.events.index_of("camera.stop"));
    return 0;
}

int test_invalid_imu_sample_fault_cancels_blocked_sources() {
    RuntimeFixture fixture(
        {{-1, {0.0, 0.0, 9.80665}, {0.0, 0.0, 0.0}, 0.0, 0.0}});
    CHECK(fixture.runtime->start());
    CHECK(fixture.imu->wait_until_waiting(1s));
    fixture.imu->release_samples(1);
    CHECK(fixture.wait_for_fault());
    CHECK(fixture.runtime->last_error() ==
          "imu source returned an invalid kinematic sample");
    fixture.runtime->stop();
    CHECK(fixture.camera->cancel_count() > 0);
    CHECK(!fixture.camera->stopped_while_capturing());
    CHECK(!fixture.imu->stopped_while_waiting());
    return 0;
}

int test_algorithm_fault() {
    RuntimeFixture fixture(valid_samples());
    fixture.vision->set_throws(true);
    fixture.flight_state->set_launched();
    CHECK(fixture.runtime->start());
    fixture.imu->release_samples(2);
    CHECK(fixture.wait_for_latest_state(1'020'000'000));
    CHECK(fixture.camera->wait_until_capturing(1s));
    fixture.camera->release_frame();
    CHECK(fixture.wait_for_fault());
    CHECK(fixture.runtime->last_error() == "injected vision failure");
    fixture.runtime->stop();
    const auto commands = fixture.command->commands();
    CHECK(commands.size() == 1);
    CHECK(!commands.front().valid);
    CHECK(commands.front().launched);
    CHECK(!commands.front().target_found);
    CHECK(commands.front().overload_g.y == 0.0);
    CHECK(commands.front().overload_g.z == 0.0);
    CHECK(!fixture.camera->stopped_while_capturing());
    CHECK(!fixture.imu->stopped_while_waiting());
    return 0;
}

int test_missing_target_does_not_refresh_command() {
    RuntimeFixture fixture(valid_samples());
    fixture.flight_state->set_launched();
    CHECK(fixture.runtime->start());
    fixture.imu->release_samples(2);
    CHECK(fixture.wait_for_latest_state(1'020'000'000));
    fixture.camera->release_frame();
    CHECK(fixture.sink->wait_for_publish(1s));
    fixture.runtime->stop();
    CHECK(!fixture.runtime->faulted());
    CHECK(fixture.command->commands().empty());
    CHECK(fixture.runtime->latest_state()->launch.phase == mosas::domain::LaunchPhase::launched);
    return 0;
}

int test_frame_sink_fault_preserves_first_error() {
    RuntimeFixture fixture(valid_samples());
    fixture.sink->set_publish_failure(true);
    CHECK(fixture.runtime->start());
    fixture.imu->release_samples(2);
    CHECK(fixture.wait_for_latest_state(1'020'000'000));
    CHECK(fixture.camera->wait_until_capturing(1s));
    fixture.camera->release_frame();
    CHECK(fixture.sink->wait_for_publish(1s));
    CHECK(fixture.wait_for_fault());
    CHECK(fixture.runtime->last_error() == "frame output failed");
    fixture.runtime->stop();
    CHECK(fixture.runtime->last_error() == "frame output failed");
    CHECK(!fixture.sink->stopped_while_publishing());
    return 0;
}

int test_frame_sink_rearm_advances_command_generation() {
    RuntimeFixture fixture(valid_samples());
    fixture.sink->request_rearm_after_publish();
    CHECK(fixture.runtime->start());
    fixture.imu->release_samples(2);
    CHECK(fixture.wait_for_latest_state(1'020'000'000));
    CHECK(fixture.camera->wait_until_capturing(1s));
    fixture.camera->release_frame();
    CHECK(fixture.sink->wait_for_publish(1s));

    const auto deadline = std::chrono::steady_clock::now() + 1s;
    while (fixture.command->active_generation() == 0 &&
           std::chrono::steady_clock::now() < deadline) {
        std::this_thread::yield();
    }
    CHECK(fixture.command->active_generation() == 1);
    fixture.runtime->stop();
    CHECK(fixture.command->commands().empty());
    CHECK(!fixture.runtime->faulted());
    return 0;
}

int test_cancel_unblocks_sink_and_stop_joins_workers() {
    RuntimeFixture fixture(valid_samples());
    fixture.sink->set_publish_blocked(true);
    CHECK(fixture.runtime->start());
    fixture.imu->release_samples(2);
    CHECK(fixture.wait_for_latest_state(1'020'000'000));
    CHECK(fixture.camera->wait_until_capturing(1s));
    fixture.camera->release_frame();
    CHECK(fixture.sink->wait_for_publish(1s));

    fixture.runtime->cancel();
    fixture.runtime->cancel();
    CHECK(fixture.sink->wait_for_publish_to_finish(1s));
    CHECK(fixture.runtime->running());
    CHECK(fixture.sink->cancel_count() == 1);
    CHECK(fixture.camera->cancel_count() == 1);
    CHECK(fixture.imu->cancel_count() == 1);
    CHECK(fixture.command->cancel_count() == 1);

    fixture.runtime->stop();
    CHECK(!fixture.runtime->running());
    CHECK(!fixture.camera->stopped_while_capturing());
    CHECK(!fixture.imu->stopped_while_waiting());
    CHECK(!fixture.sink->stopped_while_publishing());
    CHECK(fixture.events.index_of("sink.publish.exit") <
          fixture.events.index_of("sink.stop"));
    CHECK(fixture.events.index_of("camera.capture.cancelled") <
          fixture.events.index_of("camera.stop"));
    return 0;
}

}  // namespace

int main() {
    if (test_configure_time_acquisition_readiness_survives_start() != 0) {
        return 1;
    }
    if (test_partial_start_rollback_and_idempotent_stop() != 0) {
        return 1;
    }
    if (test_command_sink_start_failure_rolls_back() != 0) {
        return 1;
    }
    if (test_camera_fault_preserves_first_error() != 0) {
        return 1;
    }
    if (test_invalid_imu_sample_fault_cancels_blocked_sources() != 0) {
        return 1;
    }
    if (test_algorithm_fault() != 0) {
        return 1;
    }
    if (test_missing_target_does_not_refresh_command() != 0) {
        return 1;
    }
    if (test_frame_sink_fault_preserves_first_error() != 0) {
        return 1;
    }
    if (test_frame_sink_rearm_advances_command_generation() != 0) {
        return 1;
    }
    if (test_cancel_unblocks_sink_and_stop_joins_workers() != 0) {
        return 1;
    }
    return 0;
}
