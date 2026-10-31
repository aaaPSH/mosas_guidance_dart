#ifndef MOSAS_TEST_SUPPORT_HPP
#define MOSAS_TEST_SUPPORT_HPP

#include <cmath>
#include <iostream>

#define CHECK(condition)                                                      \
    do {                                                                      \
        if (!(condition)) {                                                   \
            std::cerr << __FILE__ << ':' << __LINE__                         \
                      << ": CHECK(" #condition ") failed\n";                \
            return 1;                                                         \
        }                                                                     \
    } while (false)

#define CHECK_NEAR(actual, expected, tolerance)                               \
    do {                                                                      \
        const double check_actual = static_cast<double>(actual);              \
        const double check_expected = static_cast<double>(expected);          \
        const double check_tolerance = static_cast<double>(tolerance);        \
        if (!std::isfinite(check_actual) ||                                   \
            std::fabs(check_actual - check_expected) > check_tolerance) {     \
            std::cerr << __FILE__ << ':' << __LINE__                         \
                      << ": CHECK_NEAR(" #actual ", " #expected            \
                      << ") failed: actual=" << check_actual                \
                      << " expected=" << check_expected                     \
                      << " tolerance=" << check_tolerance << '\n';           \
            return 1;                                                         \
        }                                                                     \
    } while (false)

#endif  // MOSAS_TEST_SUPPORT_HPP
