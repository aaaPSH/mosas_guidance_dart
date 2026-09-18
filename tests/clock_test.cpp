#include <mosas/runtime/clock.hpp>

#include <cassert>

namespace {

void test_monotonic_timestamp_is_non_decreasing() {
    const auto first = mosas::runtime::monotonic_timestamp_ns();
    const auto second = mosas::runtime::monotonic_timestamp_ns();
    assert(first >= 0);
    assert(second >= first);
}

void test_realtime_to_monotonic_preserves_offset() {
    const auto converted = mosas::runtime::realtime_to_monotonic(
        10'000, 12'000, 2'000);
    assert(converted.has_value());
    assert(*converted == 0);
}

void test_invalid_time_values_are_rejected() {
    assert(!mosas::runtime::timeval_to_nanoseconds(1, 1'000'000)
                .has_value());
    assert(!mosas::runtime::realtime_to_monotonic(-1, 1, 1).has_value());
}

}  // namespace

int main() {
    test_monotonic_timestamp_is_non_decreasing();
    test_realtime_to_monotonic_preserves_offset();
    test_invalid_time_values_are_rejected();
    return 0;
}
