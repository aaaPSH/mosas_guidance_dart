#include <mosas/app/host_adapters.hpp>

#include <chrono>
#include <cmath>
#include <sstream>
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

void test_simulated_imu_produces_monotonic_zero_samples() {
    mosas::app::SimulatedImuSource imu(1000.0);
    mosas::runtime::ImuSample first{};
    mosas::runtime::ImuSample second{};
    TEST_CHECK(imu.read(&first));
    TEST_CHECK(imu.read(&second));
    TEST_CHECK(second.timestamp_ns >= first.timestamp_ns);
    TEST_CHECK(first.acceleration.x == 0.0);
    TEST_CHECK(first.acceleration.y == 0.0);
    TEST_CHECK(first.acceleration.z == 0.0);
    TEST_CHECK(first.angular_velocity.x == 0.0);
    TEST_CHECK(first.angular_velocity.y == 0.0);
    TEST_CHECK(first.angular_velocity.z == 0.0);
    imu.cancel();
    TEST_CHECK(!imu.read(&first));
}

void test_logging_command_sink_writes_log() {
    std::ostringstream output;
    mosas::app::LoggingCommandSink sink(output);
    mosas::runtime::GuidanceCommand command{};
    command.timestamp_ns = 123;
    command.output.valid = true;
    command.output.command_overload = 1.5;
    TEST_CHECK(sink.send(command));
    TEST_CHECK(output.str().find("[日志]") != std::string::npos);
    TEST_CHECK(output.str().find("123") != std::string::npos);
}

void test_camera_source_fails_without_camera() {
    mosas::app::CameraAppConfig config;
    config.device = "/definitely/missing/mosas-camera";
    mosas::app::OpenCvCameraSource camera(config);
    TEST_CHECK(!camera.configure(
        mosas::runtime::CameraCaptureConfig{}));
    mosas::runtime::CameraFrame frame{};
    TEST_CHECK(!camera.capture(&frame));
    camera.cancel();
}

}  // namespace

int main() {
    test_simulated_imu_produces_monotonic_zero_samples();
    test_logging_command_sink_writes_log();
    test_camera_source_fails_without_camera();
    return 0;
}
