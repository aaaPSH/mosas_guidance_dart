#include <mosas/app/acquisition_time_report.hpp>
#include "test_support.hpp"
#include <sstream>

int main() {
    mosas::domain::SequenceTimeDiagnostics timing;
    std::ostringstream insufficient;
    mosas::app::write_acquisition_time_report(insufficient, "imu", timing);
    CHECK(insufficient.str().find("estimated_hz=N/A") != std::string::npos);
    CHECK(insufficient.str().find("quality=not_synchronized") != std::string::npos);
    timing.synchronized = true;
    timing.frequency_windows = 3;
    timing.configured_hz = 1000;
    timing.estimated_hz = 999.5;
    timing.unknown_discontinuities = 1;
    timing.post_launch_anomalies = 1;
    std::ostringstream report;
    mosas::app::write_acquisition_time_report(report, "imu", timing);
    CHECK(report.str().find("estimated_hz=999.5") != std::string::npos);
    CHECK(report.str().find("unknown_discontinuities=1") != std::string::npos);
    CHECK(report.str().find("quality=timing_anomalies_observed") != std::string::npos);
    CHECK(report.str().find("post_launch_clock_not_corrected") != std::string::npos);
    return 0;
}
