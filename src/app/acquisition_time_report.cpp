#include <mosas/app/acquisition_time_report.hpp>
#include <iomanip>
#include <ostream>

namespace mosas::app {
void write_acquisition_time_report(std::ostream& out, const char* source,
                                   const domain::SequenceTimeDiagnostics& t) {
    out << std::setprecision(12) << "[ACQUISITION_TIME] source=" << source
        << " configured_hz=" << t.configured_hz << " baseline_ns=" << t.baseline_ns
        << " baseline_compensation_ns=" << t.baseline_compensation_ns
        << " warmup_ns=" << t.warmup_ns << " stable_window_ns=" << t.stable_window_ns
        << " tolerance_ns=" << t.tolerance_ns << " frequency_window_ns=" << t.frequency_window_ns
        << " observed=" << t.observed << " published=" << t.published
        << " observation_seconds=" << (t.first_receive_ns >= 0 && t.last_receive_ns >= t.first_receive_ns
            ? static_cast<double>(t.last_receive_ns - t.first_receive_ns) / 1e9 : 0)
        << " warmup_discarded=" << t.warmup_discarded << " baselines=" << t.baselines
        << " known_missing=" << t.known_missing << " unknown_discontinuities=" << t.unknown_discontinuities
        << " sequence_errors=" << t.sequence_errors << " residual_exceeded=" << t.residual_exceeded
        << " post_launch_anomalies=" << t.post_launch_anomalies
        << " frequency_windows=" << t.frequency_windows;
    if (t.frequency_windows) {
        out << " estimated_hz=" << t.estimated_hz << " frequency_error_ppm=" << t.frequency_error_ppm
            << " receive_drift_ns_per_second=" << t.drift_ns_per_second;
    } else {
        out << " estimated_hz=N/A reason=no_complete_clean_observation_window";
    }
    if (t.residual_available) {
        out << " receive_residual_first_ns=" << t.first_residual_ns
            << " receive_residual_last_ns=" << t.last_residual_ns
            << " receive_residual_min_ns=" << t.minimum_residual_ns
            << " receive_residual_max_ns=" << t.maximum_residual_ns;
    } else out << " receive_residual=N/A";
    if (t.capture_residual_available) {
        out << " sdk_capture_residual_min_ns=" << t.minimum_capture_residual_ns
            << " sdk_capture_residual_max_ns=" << t.maximum_capture_residual_ns;
    }
    out << " invalid_capture_times=" << t.invalid_capture_times
        << " quality=" << (!t.synchronized ? "not_synchronized" :
            (t.unknown_discontinuities || t.sequence_errors || t.residual_exceeded || t.invalid_capture_times)
                ? "timing_anomalies_observed" : "host_estimated_not_hardware_verified")
        << " note=receive_residual_includes_transport_and_queue_jitter;post_launch_clock_not_corrected\n";
}
}
