#include <mosas/runtime/frame_timing.hpp>

#include "test_support.hpp"

#include <cstdint>
#include <limits>

int main() {
    using namespace mosas::runtime;
    const CameraClockMapping mapping{10'000'000'000LL, 5'000'000'000LL};

    const auto realtime = convert_camera_timeval(
        10, 200, CameraTimestampDomain::realtime, mapping, -2'000'000LL);
    CHECK(realtime.has_value());
    CHECK(*realtime == 4'998'200'000LL);

    const auto monotonic = convert_camera_timeval(
        5, 100, CameraTimestampDomain::monotonic, mapping, 0);
    CHECK(monotonic.has_value());
    CHECK(*monotonic == 5'000'100'000LL);

    CHECK(!convert_camera_timeval(
               5, 0, CameraTimestampDomain::unknown, mapping, 0)
               .has_value());
    CHECK(!convert_camera_timeval(
               5, 1'000'000, CameraTimestampDomain::monotonic, mapping, 0)
               .has_value());
    CHECK(!convert_camera_timeval(
               std::numeric_limits<std::int64_t>::max(), 0,
               CameraTimestampDomain::monotonic, mapping, 0)
               .has_value());
    const CameraClockMapping overflowing_mapping{
        0, std::numeric_limits<std::int64_t>::max()};
    CHECK(!convert_camera_timeval(
               0, 1, CameraTimestampDomain::realtime, overflowing_mapping, 0)
               .has_value());

    CHECK(validate_camera_timestamp(5'000'000'000LL, 5'100'000'000LL,
                                   4'900'000'000LL, 50'000'000LL,
                                   200'000'000LL));
    CHECK(!validate_camera_timestamp(5'000'000'000LL, 5'100'000'000LL,
                                    5'000'000'000LL, 50'000'000LL,
                                    200'000'000LL));
    return 0;
}
