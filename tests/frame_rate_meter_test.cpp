#include <mosas/runtime/frame_rate_meter.hpp>

#include <stdexcept>
#include <string>

namespace {

void test_check(bool condition, const char* expression, const char* file,
                int line) {
    if (!condition) {
        throw std::runtime_error(std::string(file) + ":" +
                                 std::to_string(line) +
                                 ": check failed: " + expression);
    }
}

#define TEST_CHECK(condition) \
    test_check(static_cast<bool>(condition), #condition, __FILE__, __LINE__)

void test_independent_meters_keep_their_own_samples() {
    mosas::runtime::FrameRateMeter capture_meter;
    mosas::runtime::FrameRateMeter processing_meter;

    capture_meter.record();
    capture_meter.record();
    processing_meter.record();

    TEST_CHECK(capture_meter.snapshot() > 0.0);
    TEST_CHECK(processing_meter.snapshot() > 0.0);
}

void test_snapshot_does_not_clear_samples() {
    mosas::runtime::FrameRateMeter meter;
    meter.record();
    TEST_CHECK(meter.snapshot() > 0.0);
    TEST_CHECK(meter.snapshot() > 0.0);
}

}  // namespace

int main() {
    test_independent_meters_keep_their_own_samples();
    test_snapshot_does_not_clear_samples();
    return 0;
}
