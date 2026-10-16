#ifndef MOSAS_TEST_FAKES_FAKE_ALGORITHM_PORTS_HPP
#define MOSAS_TEST_FAKES_FAKE_ALGORITHM_PORTS_HPP

#include <mosas/ports/attitude_estimator.hpp>
#include <mosas/ports/flight_state_estimator.hpp>
#include <mosas/ports/guidance_law.hpp>
#include <mosas/ports/launch_detector.hpp>
#include <mosas/ports/los_estimator.hpp>
#include <mosas/ports/los_rate_filter.hpp>
#include <mosas/ports/vision_detector.hpp>

#include <deque>
#include <thread>
#include <utility>

namespace mosas::test {

class FakeVisionDetector final : public ports::IVisionDetector {
public:
    domain::VisionDetection detect(
        const domain::ImageView& image, std::uint64_t source_frame_id,
        const domain::SensorTime& time) override {
        ++call_count_;
        last_thread_id_ = std::this_thread::get_id();
        last_image_width_ = image.width;
        last_image_height_ = image.height;
        last_first_pixel_ = image.data == nullptr ? 0 : image.data[0];
        if (!image.is_valid()) {
            domain::VisionDetection invalid{};
            invalid.observation.source_frame_id = source_frame_id;
            invalid.observation.time = time;
            invalid.observation.reason = domain::ObservationReason::invalid_frame;
            return invalid;
        }
        if (results_.empty()) {
            domain::VisionDetection missing{};
            missing.observation.source_frame_id = source_frame_id;
            missing.observation.time = time;
            return missing;
        }
        domain::VisionDetection result = std::move(results_.front());
        results_.pop_front();
        result.observation.source_frame_id = source_frame_id;
        result.observation.time = time;
        return result;
    }

    void reset() noexcept override {
        results_.clear();
        call_count_ = 0;
        last_image_width_ = 0;
        last_image_height_ = 0;
        last_first_pixel_ = 0;
        last_thread_id_ = {};
    }

    void push(domain::VisionDetection result) {
        results_.push_back(std::move(result));
    }

    std::size_t call_count() const noexcept { return call_count_; }
    int last_image_width() const noexcept { return last_image_width_; }
    int last_image_height() const noexcept { return last_image_height_; }
    std::uint8_t last_first_pixel() const noexcept { return last_first_pixel_; }
    std::thread::id last_thread_id() const noexcept { return last_thread_id_; }

private:
    std::deque<domain::VisionDetection> results_;
    std::size_t call_count_ = 0;
    int last_image_width_ = 0;
    int last_image_height_ = 0;
    std::uint8_t last_first_pixel_ = 0;
    std::thread::id last_thread_id_{};
};

class FakeAttitudeEstimator final : public ports::IAttitudeEstimator {
public:
    domain::AttitudeState update(
        const domain::ImuKinematicSample& sample) override {
        ++call_count_;
        last_timestamp_ns_ = sample.time.monotonic_ns;
        result_.time = sample.time;
        return result_;
    }

    void reset() noexcept override {
        call_count_ = 0;
        last_timestamp_ns_ = -1;
        result_ = {};
    }

    void set_result(domain::AttitudeState result) noexcept {
        result_ = result;
    }
    std::size_t call_count() const noexcept { return call_count_; }
    std::int64_t last_timestamp_ns() const noexcept {
        return last_timestamp_ns_;
    }

private:
    domain::AttitudeState result_{};
    std::size_t call_count_ = 0;
    std::int64_t last_timestamp_ns_ = -1;
};

class FakeLaunchDetector final : public ports::ILaunchDetector {
public:
    domain::LaunchState update(
        const domain::ImuKinematicSample& sample) override {
        ++call_count_;
        result_.transition_time = sample.time;
        return result_;
    }

    void reset() noexcept override {
        call_count_ = 0;
        result_ = {};
    }

    void set_result(domain::LaunchState result) noexcept { result_ = result; }
    std::size_t call_count() const noexcept { return call_count_; }

private:
    domain::LaunchState result_{};
    std::size_t call_count_ = 0;
};

class FakeFlightStateEstimator final : public ports::IFlightStateEstimator {
public:
    domain::FlightStateSnapshot update(
        const domain::ImuKinematicSample& sample) override {
        ++call_count_;
        last_thread_id_ = std::this_thread::get_id();
        last_timestamp_ns_ = sample.time.monotonic_ns;
        last_sample_ = sample;
        return result_;
    }

    void reset() noexcept override {
        call_count_ = 0;
        last_timestamp_ns_ = -1;
        last_sample_ = {};
        last_thread_id_ = {};
        result_ = {};
    }

    void set_result(domain::FlightStateSnapshot result) noexcept {
        result_ = result;
    }
    std::size_t call_count() const noexcept { return call_count_; }
    std::int64_t last_timestamp_ns() const noexcept {
        return last_timestamp_ns_;
    }
    const domain::ImuKinematicSample& last_sample() const noexcept {
        return last_sample_;
    }
    std::thread::id last_thread_id() const noexcept { return last_thread_id_; }

private:
    domain::FlightStateSnapshot result_{};
    domain::ImuKinematicSample last_sample_{};
    std::size_t call_count_ = 0;
    std::int64_t last_timestamp_ns_ = -1;
    std::thread::id last_thread_id_{};
};

class FakeLosEstimator final : public ports::ILosEstimator {
public:
    domain::LineOfSight estimate(
        const domain::VisionDetection& detection,
        const domain::CameraIntrinsics& intrinsics,
        const domain::Rotation3<domain::DartFrameTag,
                                domain::CameraFrameTag>& camera_to_dart) override {
        ++call_count_;
        last_thread_id_ = std::this_thread::get_id();
        last_detection_ = detection;
        last_intrinsics_ = intrinsics;
        last_camera_to_dart_ = camera_to_dart;
        return result_;
    }

    void set_result(domain::LineOfSight result) noexcept { result_ = result; }
    std::size_t call_count() const noexcept { return call_count_; }
    const domain::VisionDetection& last_detection() const noexcept {
        return last_detection_;
    }
    const domain::CameraIntrinsics& last_intrinsics() const noexcept {
        return last_intrinsics_;
    }
    const domain::Rotation3<domain::DartFrameTag,
                            domain::CameraFrameTag>&
    last_camera_to_dart() const noexcept {
        return last_camera_to_dart_;
    }
    std::thread::id last_thread_id() const noexcept { return last_thread_id_; }

private:
    domain::LineOfSight result_{};
    domain::VisionDetection last_detection_{};
    domain::CameraIntrinsics last_intrinsics_{};
    domain::Rotation3<domain::DartFrameTag, domain::CameraFrameTag>
        last_camera_to_dart_{};
    std::size_t call_count_ = 0;
    std::thread::id last_thread_id_{};
};

class FakeLosRateFilter final : public ports::ILosRateFilter {
public:
    domain::LosRateEstimate update(
        const domain::LosRateFilterInput& input) override {
        ++call_count_;
        last_thread_id_ = std::this_thread::get_id();
        sample_.line_of_sight = input.line_of_sight;
        sample_.filtered_angular_velocity = result_.angular_velocity;
        last_input_ = input;
        return result_;
    }

    domain::LineOfSightRateSample sample() const override { return sample_; }

    void reset() noexcept override {
        call_count_ = 0;
        sample_ = {};
        last_thread_id_ = {};
        last_input_ = {};
    }

    void set_result(domain::LosRateEstimate result) noexcept {
        result_ = result;
    }
    std::size_t call_count() const noexcept { return call_count_; }
    const domain::AngularVelocityDartRadPerSec&
    last_body_angular_velocity() const noexcept {
        return last_input_.attitude.corrected_angular_velocity;
    }
    double last_dt_seconds() const noexcept { return last_input_.dt_seconds; }
    const domain::LosRateFilterInput& last_input() const noexcept {
        return last_input_;
    }
    std::thread::id last_thread_id() const noexcept { return last_thread_id_; }

private:
    domain::LosRateEstimate result_{};
    domain::LineOfSightRateSample sample_{};
    domain::LosRateFilterInput last_input_{};
    std::size_t call_count_ = 0;
    std::thread::id last_thread_id_{};
};

class FakeGuidanceLaw final : public ports::IGuidanceLaw {
public:
    domain::GuidanceOutput calculate(
        const domain::GuidanceInput& input) override {
        ++call_count_;
        last_thread_id_ = std::this_thread::get_id();
        last_input_ = input;
        return result_;
    }

    void set_result(domain::GuidanceOutput result) noexcept {
        result_ = result;
    }
    std::size_t call_count() const noexcept { return call_count_; }
    const domain::GuidanceInput& last_input() const noexcept {
        return last_input_;
    }
    std::thread::id last_thread_id() const noexcept { return last_thread_id_; }

private:
    domain::GuidanceOutput result_{};
    domain::GuidanceInput last_input_{};
    std::size_t call_count_ = 0;
    std::thread::id last_thread_id_{};
};

}  // namespace mosas::test

#endif  // MOSAS_TEST_FAKES_FAKE_ALGORITHM_PORTS_HPP
