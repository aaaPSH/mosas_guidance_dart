#include <mosas/runtime/frame_timing.hpp>
#include <mosas/domain/sequence_time.hpp>

#include "test_support.hpp"

#include <cstdint>
#include <limits>

namespace {
/** @return Zero if a constant time-domain translation preserves exact warmup, periods and frequency. */
int compensated_windows() {
    using namespace mosas;
    for (auto bias : {0LL, -80'000'000LL, 80'000'000LL}) {
        domain::SequenceTime clock({100, 40'000'000, 20'000'000, 50'000'000, 20'000'000});
        clock.start(*runtime::apply_camera_timestamp_bias(1'000'000'000, bias));
        for (unsigned sequence = 0; sequence <= 20; ++sequence) {
            const auto receive = runtime::apply_camera_timestamp_bias(1'000'000'000 + sequence * 10'000'000LL, bias);
            const auto candidate = runtime::apply_camera_timestamp_bias(998'000'000 + sequence * 10'000'000LL, bias);
            CHECK(receive && candidate);
            const auto stamp = clock.observe(sequence, *candidate, *receive, *receive);
            CHECK(stamp.has_value() == (sequence >= 6));
            if (stamp) CHECK(stamp->timestamp_ns == *candidate);
        }
        const auto timing = clock.diagnostics();
        CHECK(timing.warmup_discarded == 6 && timing.baselines == 1);
        CHECK(timing.first_residual_ns == 2'000'000 && timing.last_residual_ns == 2'000'000);
        CHECK(timing.residual_exceeded == 0 && timing.frequency_windows > 0);
        CHECK_NEAR(timing.estimated_hz, 100, 1e-12);
    }
    return 0;
}
}

int main() {
    using namespace mosas::runtime;
    const CameraClockMapping mapping{10'000'000'000LL, 5'000'000'000LL};
    const auto maximum = std::numeric_limits<TimestampNs>::max();
    const auto minimum = std::numeric_limits<TimestampNs>::min();
    CHECK(apply_camera_timestamp_bias(0, 0) == 0);
    CHECK(apply_camera_timestamp_bias(100, -100) == 0);
    CHECK(apply_camera_timestamp_bias(0, maximum) == maximum);
    CHECK(apply_camera_timestamp_bias(maximum, -maximum) == 0);
    CHECK(!apply_camera_timestamp_bias(maximum, 1));
    CHECK(!apply_camera_timestamp_bias(100, minimum));
    CHECK(!apply_camera_timestamp_bias(100, -101));
    CHECK(!apply_camera_timestamp_bias(-1, 1));

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

    CHECK(!convert_camera_timeval(5, 0, CameraTimestampDomain::monotonic,
                                 mapping, -5'000'000'001LL));
    CHECK(convert_camera_timeval(5, 0, CameraTimestampDomain::monotonic,
                                mapping, -5'000'000'000LL) == 0);

    CHECK(validate_camera_timestamp(5'000'000'000LL, 5'100'000'000LL,
                                   4'900'000'000LL, 50'000'000LL,
                                   200'000'000LL));
    CHECK(!validate_camera_timestamp(5'000'000'000LL, 5'100'000'000LL,
                                    5'000'000'000LL, 50'000'000LL,
                                    200'000'000LL));
    CHECK(compensated_windows() == 0);
    return 0;
}
