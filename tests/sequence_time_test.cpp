#include <mosas/domain/sequence_time.hpp>
#include "test_support.hpp"
#include <cmath>
#include <limits>

namespace {
/** @brief 验证窗口中间的最小候选及相对窗口首序号的总偏移舍入。 @return 通过时为 0。 */
int check_window_baseline() {
    using namespace mosas::domain;
    SequenceTime clock({3, 0, 1'000'000'000, 10'000'000, 1'000'000'000});
    clock.start(1'000'000'000);
    CHECK(!clock.observe(50, 1'002'000'000, 1'002'000'000, 1'002'000'000));
    CHECK(!clock.observe(51, 1'334'333'333, 1'334'333'333, 1'334'333'333));
    CHECK(!clock.observe(52, 1'669'666'667, 1'669'666'667, 1'669'666'667));
    auto stamp = clock.observe(53, 2'004'000'000, 2'004'000'000, 2'004'000'000);
    CHECK(stamp && stamp->timestamp_ns == 2'001'000'000);
    CHECK(clock.diagnostics().baseline_ns == 1'001'000'000);
    CHECK(clock.diagnostics().warmup_discarded == 3);
    stamp = clock.observe(54, 2'334'333'333, 2'334'333'333, 2'334'333'333);
    CHECK(stamp && stamp->timestamp_ns == 2'334'333'333);
    // 改以最小候选所在的序号 51 为原点，会因两次舍入在此少 1 ns。
    stamp = clock.observe(55, 2'667'666'667, 2'667'666'667, 2'667'666'667);
    CHECK(stamp && stamp->timestamp_ns == 2'667'666'667);
    CHECK(clock.diagnostics().baselines == 1);
    return 0;
}

/** @brief 验证无效观测、窗口重启、采集重启和同步换代均不沿用旧候选。 @return 通过时为 0。 */
int check_window_resets() {
    using namespace mosas::domain;
    SequenceTime clock({1000, 0, 2'000'000, 5'000'000, 1'000'000'000});
    clock.start(0);
    CHECK(!clock.observe(0, 0, 0, 0));
    CHECK(!clock.observe(1, 0, 1'000'000, 1'000'000));
    auto stamp = clock.observe(3, 3'000'000, 3'000'000, 3'000'000);
    CHECK(stamp && stamp->timestamp_ns == 2'000'000);
    CHECK(clock.diagnostics().baseline_ns == 0);

    clock.start(1'000'000'000);
    CHECK(!clock.observe(10, 1'000'000'000, 1'000'000'000, 1'000'000'000));
    CHECK(!clock.observe(11, -1, 1'001'000'000, 1'001'000'000));
    CHECK(!clock.observe(12, 1'004'000'000, 1'004'000'000, 1'004'000'000));
    stamp = clock.observe(14, 1'007'000'000, 1'007'000'000, 1'007'000'000);
    CHECK(stamp && stamp->timestamp_ns == 1'006'000'000);
    CHECK(clock.diagnostics().baseline_ns == 1'004'000'000);

    clock.start(2'000'000'000);
    CHECK(!clock.observe(20, 2'000'000'000, 2'000'000'000, 2'000'000'000));
    CHECK(!clock.observe(21, 2'020'000'000, 2'020'000'000, 2'020'000'000));
    stamp = clock.observe(23, 2'023'000'000, 2'023'000'000, 2'023'000'000);
    CHECK(stamp && stamp->timestamp_ns == 2'022'000'000);
    CHECK(clock.diagnostics().baseline_ns == 2'020'000'000);
    CHECK(clock.diagnostics().baselines == 1);

    auto sync = std::make_shared<AcquisitionSync>(2);
    clock.bind(sync, 2);
    clock.start(3'000'000'000);
    CHECK(!clock.observe(100, 3'000'000'000, 3'000'000'000, 3'000'000'000));
    CHECK(sync->invalidate());
    CHECK(!clock.observe(0, 3'020'000'000, 3'020'000'000, 3'020'000'000));
    stamp = clock.observe(2, 3'023'000'000, 3'023'000'000, 3'023'000'000);
    CHECK(stamp && stamp->timestamp_ns == 3'022'000'000 && stamp->epoch == 1);
    CHECK(clock.diagnostics().baseline_ns == 3'020'000'000);
    CHECK(sync->state().ready);
    clock.discontinuity();
    CHECK(!sync->state().ready);
    CHECK(!clock.observe(5, 3'040'000'000, 3'040'000'000, 3'040'000'000));
    stamp = clock.observe(7, 3'043'000'000, 3'043'000'000, 3'043'000'000);
    CHECK(stamp && stamp->timestamp_ns == 3'042'000'000 && stamp->epoch == 2);
    CHECK(clock.diagnostics().baseline_ns == 3'040'000'000);
    CHECK(clock.diagnostics().baselines == 2);
    return 0;
}
}

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
    CHECK(check_window_baseline() == 0);
    CHECK(check_window_resets() == 0);
    return 0;
}
