#include "serial_test_support.hpp"
#include "test_support.hpp"
#include "output_dispatcher.hpp"
#include "fakes/fake_algorithm_ports.hpp"
#include "fakes/fake_camera_source.hpp"
#include "fakes/fake_clock.hpp"
#include "fakes/fake_imu_source.hpp"

#include <mosas/app/default_guidance_algorithms.hpp>
#include <mosas/output/local_recording_session.hpp>
#include <mosas/output/telemetry_csv_recorder.hpp>
#include <mosas/vision/opencv_image_frame.hpp>
#include <opencv2/imgproc.hpp>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace {
using namespace mosas;
using namespace mosas::test;
using namespace std::chrono_literals;

/** @brief 作用域结束时删除 CSV 测试文件，不在项目中留下产物。 */
struct RecordingDirectory {
    std::filesystem::path path = std::filesystem::temp_directory_path() /
        ("mosas_serial_output_" + std::to_string(SerialClock::now().time_since_epoch().count()));
    ~RecordingDirectory() { std::filesystem::remove_all(path); }
};

/** @return 串口帧中的有符号过载值，单位 g；offset 为 yaw=4 或 pitch=6。 */
double wire_overload(const ControlFrame& frame, std::size_t offset) {
    const int value = frame[offset] | (static_cast<int>(frame[offset + 1]) << 8);
    return (value >= 0x8000 ? value - 0x10000 : value) / 100.0;
}

/** @return CSV 一行的字段，当前记录不含引号或转义。 */
std::vector<std::string> csv_columns(const std::string& row) {
    std::istringstream input(row);
    std::vector<std::string> columns;
    std::string value;
    while (std::getline(input, value, ',')) columns.push_back(value);
    if (row.back() == ',') columns.emplace_back();
    return columns;
}

/** @return 新提交的陈旧曝光请求也必须清零，且不清除 IMU 发射位时返回 0。 */
int stale_exposure() {
    SerialFixture serial;
    CHECK(serial.open());
    LiveOutputState state;
    std::ostringstream errors;
    app::CommandAppConfig config;
    config.send_rate_hz = 1000;
    config.command_timeout_ms = 100;
    app::SerialCommandSink sink(serial.port, config, errors);
    CHECK(bind_live_output(sink, state));
    CHECK(sink.start());
    auto request = fresh_command();
    request.source_time = command_test_time(runtime::monotonic_timestamp_ns() - 3'000'000'000LL);
    const auto receipt = sink.track_emission(request);
    CHECK(sink.try_submit(request));
    CHECK(receipt->result().wait_for(500ms) == std::future_status::ready);
    const auto emitted = receipt->result().get();
    CHECK(emitted);
    CHECK(!emitted->mapping.command.valid);
    CHECK(emitted->mapping.command.invalid_reason == domain::GuidanceInvalidReason::source_too_old);
    CHECK(emitted->mapping.command.launched && !emitted->mapping.command.target_found);
    CHECK(serial.wait_for(encoded(true, false)));
    sink.stop();
    CHECK(errors.str().empty());
    return 0;
}

/** @return 原识别/滤波/PNG/串口链路使用 max_camera_age_ns 拦截曝光后的处理延迟时返回 0。 */
int runtime_exposure_age() {
    SerialFixture serial;
    CHECK(serial.open());
    std::vector<runtime::ImuAngularVelocitySample> samples;
    for (int ms = 0; ms < 230; ++ms) samples.push_back({ms * 1'000'000LL, {0, 9.80665, 0}, {}, 0, 0});
    auto imu = std::make_unique<FakeImuSource>(samples);
    auto* release_imu = imu.get();
    std::vector<runtime::CameraFrame> frames;
    for (int ms : {120, 130}) {
        cv::Mat image = cv::Mat::zeros(40, 40, CV_8UC3);
        cv::rectangle(image, {10, 10, 20, 20}, {0, 255, 0}, cv::FILLED);
        runtime::CameraFrame frame;
        frame.timestamp_ns = ms * 1'000'000LL;
        frame.received_timestamp_ns = frame.timestamp_ns; // 接收校验通过，但发送时图像已旧。
        frame.sensor_time = command_test_time(frame.timestamp_ns);
        frame.timestamp_source = runtime::CameraTimestampSource::sdk_monotonic;
        CHECK(vision::assign_bgr8_mat(&frame.image, std::move(image)));
        frames.push_back(std::move(frame));
    }
    auto camera = std::make_unique<FakeCameraSource>(frames);
    auto* release_camera = camera.get();
    auto dependencies = app::make_default_guidance_algorithms({});
    auto flight = std::make_unique<FakeFlightStateEstimator>();
    auto* set_flight = flight.get();
    dependencies.flight_state_estimator = std::move(flight);
    dependencies.monotonic_clock = std::make_unique<FakeClock>(230'000'000);
    app::CommandAppConfig command;
    command.send_rate_hz = 1000;
    std::ostringstream errors;
    auto sink = std::make_unique<app::SerialCommandSink>(serial.port, command, errors);
    auto* output = sink.get();
    runtime::RuntimeSupervisorConfig config;
    config.capture_queue_capacity = 8;
    config.max_camera_age_ns = 5'000'000;
    config.camera_intrinsics = {100, 100, 20, 20};
    runtime::RuntimeSupervisor supervisor(std::move(imu), std::move(camera), std::move(sink),
        config, std::move(dependencies), {});
    CHECK(supervisor.start());
    auto state = command_test_state(0);
    state.launch.transition_time = command_test_time(0);
    set_flight->set_result(state);
    release_imu->release();
    const auto deadline = SerialClock::now() + 1s;
    while (!supervisor.latest_state() || supervisor.latest_state()->time.monotonic_ns != 229'000'000) {
        CHECK(SerialClock::now() < deadline);
        std::this_thread::yield();
    }
    release_camera->release();
    const auto emitted = wait_for_emission(*output, [](const auto& value) {
        return value.mapping.command.source_time.monotonic_ns == 130'000'000;
    });
    CHECK(emitted && !emitted->mapping.command.valid);
    CHECK(emitted->mapping.command.invalid_reason == domain::GuidanceInvalidReason::source_too_old);
    CHECK(emitted->mapping.command.launched && !emitted->mapping.command.target_found);
    CHECK(emitted->mapping.command.overload_g.y == 0 && emitted->mapping.command.overload_g.z == 0);
    CHECK(emitted->disposition == domain::CommandEmissionDisposition::serial_written);
    CHECK(supervisor.timing().guidance.samples == 1);
    supervisor.stop();
    CHECK(!supervisor.faulted() && errors.str().empty());
    return 0;
}

/** @return 量化后的实际字节、首个发送回执和 CSV 一致，且不越过两类上限时返回 0。 */
int quantized_output() {
    RecordingDirectory directory;
    output::LocalRecordingConfig recording;
    recording.recording_path = (directory.path / "serial.avi").string();
    recording.mode = output::LocalRecordingMode::continuous;
    recording.width = recording.height = 16;
    recording.fps = 100;
    auto session = std::make_shared<output::LocalRecordingSession>(recording);
    runtime::OutputDispatcher dispatcher(
        nullptr, std::make_unique<output::TelemetryCsvRecorder>(session),
        nullptr, nullptr, 8, {});
    dispatcher.reset();
    dispatcher.start();

    struct Case {
        double y, z, maximum_g, maximum_acceleration;
        guidance::GuidanceDebugMode mode = guidance::GuidanceDebugMode::disabled;
    };
    const std::vector<Case> cases{
        {0.0049, -0.0049, 0.3, 9.80665},
        {0.005, -0.005, 0.3, 9.80665},
        {0.075, -0.075, 0.3, 9.80665},
        {0.299, 0.024, 0.3, 9.80665},
        {-0.024, -0.299, 10, 0.3 * 9.80665},
        {0.006, 0.006, 0.009, 9.80665},
        {0.2, 0.1, 0, 9.80665},
        {0.299, 0.024, 0.3, 9.80665, guidance::GuidanceDebugMode::target_detected},
    };
    std::size_t index = 0;
    std::vector<domain::CommandEmissionSnapshot> emissions;
    for (const auto& test : cases) {
        SerialFixture serial;
        CHECK(serial.open());
        LiveOutputState state;
        std::ostringstream errors;
        app::CommandAppConfig config;
        config.send_rate_hz = 1000;
        guidance::CommandMappingConfig mapping;
        mapping.maximum_command_overload = test.maximum_g;
        mapping.maximum_command_acceleration = test.maximum_acceleration;
        mapping.debug.mode = test.mode;
        mapping.debug.pitch_overload_g = test.y;
        mapping.debug.yaw_overload_g = test.z;
        app::SerialCommandSink sink(serial.port, config, errors);
        CHECK(bind_live_output(sink, state, mapping));
        CHECK(sink.start());
        auto request = fresh_command(test.y, test.z);
        request.source_frame_id = ++index;
        runtime::ProcessedFrame frame;
        frame.frame.source_frame_id = request.source_frame_id;
        frame.frame.timestamp_ns = request.source_time.monotonic_ns;
        frame.pending_command_emission = sink.track_emission(request);
        const auto receipt = frame.pending_command_emission;
        CHECK(dispatcher.enqueue(frame));
        CHECK(sink.try_submit(request));
        CHECK(receipt->result().wait_for(500ms) == std::future_status::ready);
        const auto emitted = receipt->result().get();
        CHECK(emitted && emitted->mapping.command.valid);
        CHECK(emitted->disposition == domain::CommandEmissionDisposition::serial_written);
        ControlFrame wire;
        const auto wire_deadline = SerialClock::now() + 500ms;
        do {
            CHECK(SerialClock::now() < wire_deadline);
            CHECK(serial.read_frame(wire, wire_deadline - SerialClock::now()));
        } while (wire[3] == 0);
        const double y = wire_overload(wire, 6), z = wire_overload(wire, 4);
        CHECK_NEAR(emitted->mapping.command.overload_g.y, y, 1e-12);
        CHECK_NEAR(emitted->mapping.command.overload_g.z, z, 1e-12);
        CHECK_NEAR(emitted->mapping.trace.body_overload.y, y, 1e-12);
        CHECK_NEAR(emitted->mapping.trace.body_overload.z, z, 1e-12);
        CHECK(emitted->mapping.trace.unlimited_body_overload);
        CHECK_NEAR(emitted->mapping.trace.unlimited_body_overload->y, test.y, 1e-12);
        CHECK_NEAR(emitted->mapping.trace.unlimited_body_overload->z, test.z, 1e-12);
        CHECK_NEAR(emitted->mapping.trace.command_overload.value, std::hypot(y, z), 1e-12);
        CHECK(std::hypot(y, z) <= test.maximum_g);
        CHECK(std::hypot(y, z) * mapping.gravity <= test.maximum_acceleration);
        emissions.push_back(*emitted);
        sink.stop();
        CHECK(errors.str().empty());
    }
    dispatcher.close_queue();
    dispatcher.join();
    CHECK(dispatcher.stop_sink().empty());
    std::ifstream csv(session->paths_for_generation(0).telemetry_csv);
    std::string row;
    CHECK(std::getline(csv, row));
    for (const auto& emitted : emissions) {
        CHECK(std::getline(csv, row));
        const auto columns = csv_columns(row);
        CHECK(columns.size() == 38);
        CHECK(columns[35].empty());
        CHECK_NEAR(std::stod(columns[36]), emitted.mapping.trace.unlimited_body_overload->y, 1e-6);
        CHECK_NEAR(std::stod(columns[37]), emitted.mapping.trace.unlimited_body_overload->z, 1e-6);
        CHECK_NEAR(std::stod(columns[24]), emitted.mapping.command.overload_g.y, 1e-12);
        CHECK_NEAR(std::stod(columns[25]), emitted.mapping.command.overload_g.z, 1e-12);
        CHECK_NEAR(std::stod(columns[26]), emitted.mapping.trace.command_overload.value, 1e-6);
        CHECK(columns[29] == "1" && columns[30] == "0");
    }
    CHECK(!std::getline(csv, row));
    return 0;
}
} // namespace

int main() {
    const int stale = stale_exposure();
    const int runtime_age = runtime_exposure_age();
    const int quantized = quantized_output();
    CHECK(stale == 0 && runtime_age == 0 && quantized == 0);
    return 0;
}
