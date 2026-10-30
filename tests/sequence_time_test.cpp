#include <mosas/domain/sequence_time.hpp>
#include "test_support.hpp"
#include <cmath>
#include <limits>

int main() {
    using namespace mosas::domain;
    SequenceTimeConfig config{999.5, 0, 0, 50'000'000, 1'000'000'000};
    CHECK(SequenceTime::valid_config(config));
    auto invalid = config; invalid.frequency_hz = std::numeric_limits<double>::quiet_NaN();
    CHECK(!SequenceTime::valid_config(invalid));
    auto sync = std::make_shared<AcquisitionSync>(2);
    SequenceTime clock(config); clock.bind(sync, 2); clock.start(1'000'000'000);
    auto t = clock.observe(10, 1'000'000'000, 1'000'000'000, 1'000'000'000);
    CHECK(t && t->timestamp_ns == 1'000'000'000);
    CHECK(sync->latch_launch(0));
    const auto expected = 1'000'000'000 + static_cast<std::int64_t>(std::round(1e9L * 1'000'000 / 999.5L));
    t = clock.observe(1'000'010, expected + 100'000'000, expected + 100'000'000, expected + 100'000'000);
    CHECK(t && t->timestamp_ns == expected);
    clock.discontinuity();
    t = clock.observe(1'000'011, expected + 200'000'000, expected + 200'000'000, expected + 200'000'000);
    CHECK(t && t->epoch == 0);
    CHECK(clock.diagnostics().baselines == 1);
    CHECK(clock.diagnostics().post_launch_anomalies >= 2);
    CHECK(!clock.observe(1'000'011, expected, expected, expected));

    SequenceTimeConfig warm{1000, 100'000'000, 100'000'000, 10'000'000, 1'000'000'000};
    SequenceTime startup(warm); startup.start(1'000'000'000);
    CHECK(!startup.observe(0, 0, 0, 1'000'000'000));
    CHECK(!startup.observe(1, 1'100'000'000, 1'100'000'000, 1'100'000'000));
    CHECK(!startup.observe(2, 1'101'000'000, 1'101'000'000, 1'101'000'000));
    CHECK(startup.observe(101, 1'200'000'000, 1'200'000'000, 1'200'000'000));
    startup.discontinuity();
    CHECK(!startup.diagnostics().synchronized);

    SequenceTime rate({1000, 0, 0, 100'000'000, 1'000'000'000}); rate.start(0);
    CHECK(rate.observe(0, 0, 0, 0));
    CHECK(rate.observe(1001, 1'000'000'000, 1'000'000'000, 1'000'000'000));
    CHECK(std::abs(rate.diagnostics().estimated_hz - 1001) < 1e-9);
    CHECK(std::abs(rate.diagnostics().frequency_error_ppm - 1000) < 1e-6);
    CHECK(!sync->invalidate());
    return 0;
}
