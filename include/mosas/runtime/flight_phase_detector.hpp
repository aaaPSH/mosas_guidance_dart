#ifndef MOSAS_FLIGHT_PHASE_DETECTOR_HPP
#define MOSAS_FLIGHT_PHASE_DETECTOR_HPP

#include <mosas/runtime/runtime_types.hpp>

namespace mosas::runtime {

struct FlightPhaseDetectorConfig {
    int min_static_samples = 20;
    TimestampNs min_static_duration_ns = 100000000;
    double max_static_axis_variation = 0.5;
    int launch_direction = 1;
    double ejection_start_threshold = 20.0;
    TimestampNs ejection_confirm_duration_ns = 1000000;
    double free_flight_release_threshold = 3.0;
    TimestampNs free_flight_confirm_duration_ns = 1000000;
};

class FlightPhaseDetector {
public:
    explicit FlightPhaseDetector(const FlightPhaseDetectorConfig& config = {});

    FlightPhase update(const ImuSample& sample);

    FlightPhase phase() const noexcept;
    Vector3 baseline_acceleration() const noexcept;
    bool initialized() const noexcept;

private:
    void accumulate_static_sample(const ImuSample& sample);
    void reset_static_window(const ImuSample& sample);

    FlightPhaseDetectorConfig config_;
    FlightPhase phase_ = FlightPhase::pre_launch;
    bool initialized_ = false;
    bool has_timestamp_ = false;
    TimestampNs last_timestamp_ns_ = 0;

    int static_sample_count_ = 0;
    TimestampNs static_start_timestamp_ns_ = 0;
    Vector3 static_sum_{0.0, 0.0, 0.0};
    Vector3 static_min_{0.0, 0.0, 0.0};
    Vector3 static_max_{0.0, 0.0, 0.0};
    Vector3 baseline_acceleration_{0.0, 0.0, 0.0};

    bool has_ejection_candidate_ = false;
    TimestampNs ejection_candidate_start_ns_ = 0;
    bool has_release_candidate_ = false;
    TimestampNs release_candidate_start_ns_ = 0;
};

}  // namespace mosas::runtime

#endif  // MOSAS_FLIGHT_PHASE_DETECTOR_HPP
