#include <mosas/runtime/flight_phase_detector.hpp>

#include <algorithm>
#include <cmath>

namespace mosas::runtime {
namespace {

bool is_finite(const Vector3& value) {
    return std::isfinite(value.x) && std::isfinite(value.y) &&
           std::isfinite(value.z);
}

bool is_valid_config(const FlightPhaseDetectorConfig& config) {
    return config.min_static_samples > 0 &&
           config.min_static_duration_ns >= 0 &&
           std::isfinite(config.max_static_axis_variation) &&
           config.max_static_axis_variation >= 0.0 &&
           (config.launch_direction == 1 || config.launch_direction == -1) &&
           std::isfinite(config.ejection_start_threshold) &&
           config.ejection_start_threshold >= 0.0 &&
           config.ejection_confirm_duration_ns >= 0 &&
           std::isfinite(config.free_flight_release_threshold) &&
           config.free_flight_release_threshold >= 0.0 &&
           config.free_flight_confirm_duration_ns >= 0;
}

FlightPhaseDetectorConfig default_config() {
    return {};
}

Vector3 divide(const Vector3& value, double divisor) {
    return {value.x / divisor, value.y / divisor, value.z / divisor};
}

}  // namespace

FlightPhaseDetector::FlightPhaseDetector(
    const FlightPhaseDetectorConfig& config)
    : config_(is_valid_config(config) ? config : default_config()) {}

void FlightPhaseDetector::reset_static_window(const ImuSample& sample) {
    static_sample_count_ = 1;
    static_start_timestamp_ns_ = sample.timestamp_ns;
    static_sum_ = sample.acceleration;
    static_min_ = sample.acceleration;
    static_max_ = sample.acceleration;
}

void FlightPhaseDetector::accumulate_static_sample(const ImuSample& sample) {
    ++static_sample_count_;
    static_sum_.x += sample.acceleration.x;
    static_sum_.y += sample.acceleration.y;
    static_sum_.z += sample.acceleration.z;
    static_min_.x = std::min(static_min_.x, sample.acceleration.x);
    static_min_.y = std::min(static_min_.y, sample.acceleration.y);
    static_min_.z = std::min(static_min_.z, sample.acceleration.z);
    static_max_.x = std::max(static_max_.x, sample.acceleration.x);
    static_max_.y = std::max(static_max_.y, sample.acceleration.y);
    static_max_.z = std::max(static_max_.z, sample.acceleration.z);
}

FlightPhase FlightPhaseDetector::update(const ImuSample& sample) {
    if (sample.timestamp_ns < 0 || !is_finite(sample.acceleration) ||
        !is_finite(sample.angular_velocity) ||
        (has_timestamp_ && sample.timestamp_ns <= last_timestamp_ns_)) {
        return phase_;
    }
    has_timestamp_ = true;
    last_timestamp_ns_ = sample.timestamp_ns;

    if (phase_ == FlightPhase::pre_launch && !initialized_) {
        if (static_sample_count_ == 0) {
            reset_static_window(sample);
        } else {
            const Vector3 next_min{
                std::min(static_min_.x, sample.acceleration.x),
                std::min(static_min_.y, sample.acceleration.y),
                std::min(static_min_.z, sample.acceleration.z)};
            const Vector3 next_max{
                std::max(static_max_.x, sample.acceleration.x),
                std::max(static_max_.y, sample.acceleration.y),
                std::max(static_max_.z, sample.acceleration.z)};
            if (next_max.x - next_min.x > config_.max_static_axis_variation ||
                next_max.y - next_min.y > config_.max_static_axis_variation ||
                next_max.z - next_min.z > config_.max_static_axis_variation) {
                reset_static_window(sample);
            } else {
                accumulate_static_sample(sample);
            }
        }

        const TimestampNs static_duration =
            sample.timestamp_ns - static_start_timestamp_ns_;
        if (static_sample_count_ >= config_.min_static_samples &&
            static_duration >= config_.min_static_duration_ns) {
            baseline_acceleration_ = divide(
                static_sum_, static_cast<double>(static_sample_count_));
            initialized_ = true;
        }
        return phase_;
    }

    const double signed_ax_delta =
        static_cast<double>(config_.launch_direction) *
        (sample.acceleration.x - baseline_acceleration_.x);

    if (phase_ == FlightPhase::pre_launch) {
        if (signed_ax_delta >= config_.ejection_start_threshold) {
            if (!has_ejection_candidate_) {
                has_ejection_candidate_ = true;
                ejection_candidate_start_ns_ = sample.timestamp_ns;
            }
            if (sample.timestamp_ns - ejection_candidate_start_ns_ >=
                config_.ejection_confirm_duration_ns) {
                phase_ = FlightPhase::ejection;
                has_ejection_candidate_ = false;
            }
        } else {
            has_ejection_candidate_ = false;
        }
        return phase_;
    }

    if (phase_ == FlightPhase::ejection) {
        if (signed_ax_delta <= config_.free_flight_release_threshold) {
            if (!has_release_candidate_) {
                has_release_candidate_ = true;
                release_candidate_start_ns_ = sample.timestamp_ns;
            }
            if (sample.timestamp_ns - release_candidate_start_ns_ >=
                config_.free_flight_confirm_duration_ns) {
                phase_ = FlightPhase::free_flight;
                has_release_candidate_ = false;
            }
        } else {
            has_release_candidate_ = false;
        }
    }
    return phase_;
}

FlightPhase FlightPhaseDetector::phase() const noexcept {
    return phase_;
}

Vector3 FlightPhaseDetector::baseline_acceleration() const noexcept {
    return baseline_acceleration_;
}

bool FlightPhaseDetector::initialized() const noexcept {
    return initialized_;
}

}  // namespace mosas::runtime
