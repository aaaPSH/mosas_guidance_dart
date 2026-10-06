#include <mosas/runtime/clock.hpp>

#include "test_support.hpp"

int main() {
    const auto timestamp = mosas::runtime::timeval_to_nanoseconds(1, 2);
    CHECK(timestamp.has_value());
    CHECK(*timestamp == 1'000'002'000LL);
    return 0;
}
