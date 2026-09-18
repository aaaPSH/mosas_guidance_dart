#include <mosas/runtime/frame_timing.hpp>

#include <cassert>
#include <cstdint>

namespace {

void test_explicit_realtime_and_monotonic_conversion() {
    const mosas::runtime::CameraClockMapping mapping{10'000, 2'000};
    const auto realtime = mosas::runtime::convert_camera_timeval(
        0, 12, mosas::runtime::CameraTimestampDomain::realtime, mapping,
        0);
    assert(realtime.has_value());
    assert(*realtime == 4'000);

    const auto monotonic = mosas::runtime::convert_camera_timeval(
        0, 12, mosas::runtime::CameraTimestampDomain::monotonic, mapping,
        0);
    assert(monotonic.has_value());
    assert(*monotonic == 12'000);
}

void test_invalid_domain_and_time_are_rejected() {
    const mosas::runtime::CameraClockMapping mapping{10'000, 2'000};
    assert(!mosas::runtime::convert_camera_timeval(
                0, 0, mosas::runtime::CameraTimestampDomain::unknown, mapping,
                0)
                .has_value());
    assert(!mosas::runtime::convert_camera_timeval(
                0, 1'000'000, mosas::runtime::CameraTimestampDomain::realtime,
                mapping, 0)
                .has_value());
    assert(!mosas::runtime::convert_camera_timeval(
                INT64_MAX, 0, mosas::runtime::CameraTimestampDomain::realtime,
                mapping, 0)
                .has_value());
    assert(!mosas::runtime::convert_camera_timeval(
                0, 0, mosas::runtime::CameraTimestampDomain::realtime,
                {INT64_MIN, 2}, 0)
                .has_value());
}

void test_timestamp_validation_rejects_future_stale_and_rollback() {
    assert(mosas::runtime::validate_camera_timestamp(1'000, 1'010, -1,
                                                     100, 100));
    assert(!mosas::runtime::validate_camera_timestamp(1'200, 1'010, -1, 100,
                                                      100));
    assert(!mosas::runtime::validate_camera_timestamp(800, 1'010, -1, 100,
                                                      100));
    assert(!mosas::runtime::validate_camera_timestamp(900, 1'010, 950, 100,
                                                      100));
}

}  // namespace

int main() {
    test_explicit_realtime_and_monotonic_conversion();
    test_invalid_domain_and_time_are_rejected();
    test_timestamp_validation_rejects_future_stale_and_rollback();
    return 0;
}
