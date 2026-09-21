#include <mosas/app/host_adapters.hpp>
#include <serial_package/serial_port.hpp>
#include <serial_package/serial_protocol.hpp>

#include <array>
#include <atomic>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fcntl.h>
#include <pty.h>
#include <sstream>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

#include <poll.h>

namespace {

struct PseudoTerminal {
    int master = -1;
    int slave = -1;
    std::string slave_name;

    PseudoTerminal() {
        char name[128]{};
        assert(::openpty(&master, &slave, name, nullptr, nullptr) == 0);
        slave_name = name;
        const int flags = ::fcntl(master, F_GETFL, 0);
        assert(flags >= 0);
        assert(::fcntl(master, F_SETFL, flags | O_NONBLOCK) == 0);
    }

    ~PseudoTerminal() {
        if (master >= 0) {
            ::close(master);
        }
        if (slave >= 0) {
            ::close(slave);
        }
    }
};

std::array<std::uint8_t, 14> read_control_frame(int master) {
    std::array<std::uint8_t, 14> frame{};
    std::size_t received = 0;
    while (received < frame.size()) {
        pollfd descriptor{master, POLLIN, 0};
        assert(::poll(&descriptor, 1, 500) == 1);
        const ssize_t count =
            ::read(master, frame.data() + received, frame.size() - received);
        assert(count > 0);
        received += static_cast<std::size_t>(count);
    }
    return frame;
}

void test_serial_command_sink_sends_zero_z_y_overloads() {
    PseudoTerminal terminal;
    auto port = std::make_shared<serial_package::SerialPort>();
    serial_package::SerialPortConfig config;
    config.device = terminal.slave_name;
    config.baud_rate = 115200;
    config.read_timeout_ms = 50;
    config.write_timeout_ms = 50;
    assert(port->open(config));

    std::ostringstream log;
    mosas::app::SerialCommandSink sink(port, log);
    mosas::runtime::GuidanceCommand command{
        123,
        {true, {}, {}, {99.0, 1.25, -2.5}, 0.0, 0.0}};
    assert(sink.send(command));

    const auto frame = read_control_frame(terminal.master);
    const std::array<std::uint8_t, 14> expected{
        0xAA, 0x55, 0x00, 0x00, 0x06, 0xFF, 0x7D, 0x00,
        0x00, 0x00, 0x83, 0x9D, 0x0D, 0x0A};
    assert(frame == expected);
    assert(log.str().find(
               "[串口][控制][发送] bytes=AA 55 00 00 06 FF 7D 00 00 00 83 9D 0D 0A") !=
           std::string::npos);
}

void test_serial_imu_source_decodes_rad_s_and_logs_initialization_g() {
    PseudoTerminal terminal;
    auto port = std::make_shared<serial_package::SerialPort>();
    serial_package::SerialPortConfig config;
    config.device = terminal.slave_name;
    config.baud_rate = 115200;
    config.read_timeout_ms = 50;
    config.write_timeout_ms = 50;
    assert(port->open(config));

    std::ostringstream log;
    mosas::app::SerialImuSource source(port, log);
    const std::array<std::uint8_t, serial_package::kImuFrameSize> valid_frame{
        0x55, 0xAA, 0x64, 0x00, 0x38, 0xFF, 0x2C, 0x01,
        0x0A, 0x00, 0xEC, 0xFF, 0x1E, 0x00, 0x34, 0x12,
        0x21, 0x04, 0x0D, 0x0A};
    auto invalid_frame = valid_frame;
    invalid_frame[16] ^= 0x01;
    assert(::write(terminal.master, invalid_frame.data(), invalid_frame.size()) ==
           static_cast<ssize_t>(invalid_frame.size()));
    assert(::write(terminal.master, valid_frame.data(), valid_frame.size()) ==
           static_cast<ssize_t>(valid_frame.size()));

    mosas::runtime::ImuSample sample{};
    const auto result = source.read(&sample);
    assert(result.status == mosas::runtime::SourceStatus::ok);
    assert(sample.timestamp_ns > 0);
    assert(std::abs(sample.acceleration.x - 1.0) < 1e-12);
    assert(std::abs(sample.acceleration.y + 2.0) < 1e-12);
    assert(std::abs(sample.angular_velocity.x - 1.0) < 1e-12);
    assert(std::abs(sample.angular_velocity.y + 2.0) < 1e-12);
    assert(std::abs(sample.angular_velocity.z - 3.0) < 1e-12);
    assert(log.str().find("checksum") != std::string::npos);
    assert(log.str().find("expected=0x0421") != std::string::npos);
    assert(log.str().find("g_bytes=0x3412") != std::string::npos);
    assert(log.str().find(
               "[串口][IMU][接收] bytes=55 AA 64 00 38 FF 2C 01 0A 00 EC FF 1E 00 34 12 21 04 0D 0A") !=
           std::string::npos);

    auto zero_g_frame = valid_frame;
    zero_g_frame[14] = 0;
    zero_g_frame[15] = 0;
    zero_g_frame[16] = 0xDB;
    zero_g_frame[17] = 0x03;
    assert(::write(terminal.master, zero_g_frame.data(), zero_g_frame.size()) ==
           static_cast<ssize_t>(zero_g_frame.size()));
    assert(source.read(&sample).status == mosas::runtime::SourceStatus::ok);
    assert(log.str().find("unexpected") == std::string::npos);

    assert(::write(terminal.master, valid_frame.data(), valid_frame.size()) ==
           static_cast<ssize_t>(valid_frame.size()));
    assert(source.read(&sample).status == mosas::runtime::SourceStatus::ok);
    assert(log.str().find("expected=0x0000") != std::string::npos);
}

void test_serial_command_error_is_available_through_sink_interface() {
    std::ostringstream log;
    std::unique_ptr<mosas::runtime::CommandSink> sink =
        std::make_unique<mosas::app::SerialCommandSink>(nullptr, log);
    const mosas::runtime::GuidanceCommand command{
        123,
        {true, {}, {}, {0.0, 1.0, 2.0}, 0.0, 0.0}};

    assert(!sink->send(command));
    assert(sink->last_error().find("not open") != std::string::npos);
    assert(log.str().find("[串口][控制][错误]") != std::string::npos);
}

void test_serial_imu_source_reports_timeout_cancel_and_closed_port() {
    PseudoTerminal terminal;
    auto port = std::make_shared<serial_package::SerialPort>();
    serial_package::SerialPortConfig config;
    config.device = terminal.slave_name;
    config.baud_rate = 115200;
    config.read_timeout_ms = 20;
    config.write_timeout_ms = 20;
    assert(port->open(config));

    std::ostringstream log;
    mosas::app::SerialImuSource source(port, log);
    mosas::runtime::ImuSample sample{};
    assert(source.read(&sample).status == mosas::runtime::SourceStatus::timeout);

    source.cancel();
    assert(source.read(&sample).status ==
           mosas::runtime::SourceStatus::cancelled);

    auto second_port = std::make_shared<serial_package::SerialPort>();
    assert(second_port->open(config));
    std::ostringstream closed_log;
    mosas::app::SerialImuSource closed_source(second_port, closed_log);
    second_port->close();
    const auto result = closed_source.read(&sample);
    assert(result.status == mosas::runtime::SourceStatus::fatal);
    assert(result.message.find("not open") != std::string::npos);
    assert(closed_log.str().find("[串口][IMU][错误]") !=
           std::string::npos);
}

void test_serial_imu_cancel_wakes_a_long_read() {
    PseudoTerminal terminal;
    auto port = std::make_shared<serial_package::SerialPort>();
    serial_package::SerialPortConfig config;
    config.device = terminal.slave_name;
    config.baud_rate = 115200;
    config.read_timeout_ms = 5000;
    config.write_timeout_ms = 50;
    assert(port->open(config));

    mosas::app::SerialImuSource source(port);
    std::atomic<mosas::runtime::SourceStatus> status{
        mosas::runtime::SourceStatus::fatal};
    std::thread reader([&] {
        mosas::runtime::ImuSample sample{};
        status.store(source.read(&sample).status);
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    const auto cancel_started = std::chrono::steady_clock::now();
    source.cancel();
    reader.join();
    const auto cancel_duration = std::chrono::steady_clock::now() -
                                 cancel_started;

    assert(status.load() == mosas::runtime::SourceStatus::cancelled);
    assert(cancel_duration < std::chrono::milliseconds(500));
}

void test_serial_command_sink_can_be_cancelled_through_interface() {
    std::ostringstream log;
    std::unique_ptr<mosas::runtime::CommandSink> sink =
        std::make_unique<mosas::app::SerialCommandSink>(nullptr, log);
    const mosas::runtime::GuidanceCommand command{
        123,
        {true, {}, {}, {0.0, 1.0, 2.0}, 0.0, 0.0}};

    sink->cancel();
    assert(!sink->send(command));
    assert(sink->last_error().find("cancel") != std::string::npos);
}

void test_serial_imu_reset_flushes_stale_input() {
    PseudoTerminal terminal;
    auto port = std::make_shared<serial_package::SerialPort>();
    serial_package::SerialPortConfig config;
    config.device = terminal.slave_name;
    config.baud_rate = 115200;
    config.read_timeout_ms = 50;
    assert(port->open(config));

    const std::array<std::uint8_t, serial_package::kImuFrameSize> stale_frame{
        0x55, 0xAA, 0x64, 0x00, 0x38, 0xFF, 0x2C, 0x01,
        0x0A, 0x00, 0xEC, 0xFF, 0x1E, 0x00, 0x34, 0x12,
        0x21, 0x04, 0x0D, 0x0A};
    assert(::write(terminal.master, stale_frame.data(), stale_frame.size()) ==
           static_cast<ssize_t>(stale_frame.size()));

    mosas::app::SerialImuSource source(port);
    source.reset();
    mosas::runtime::ImuSample sample{};
    assert(source.read(&sample).status == mosas::runtime::SourceStatus::timeout);
}

void test_serial_imu_reset_failure_is_reported() {
    std::ostringstream log;
    mosas::app::SerialImuSource source(nullptr, log);
    source.reset();
    assert(!source.reset_ok());
    assert(source.last_error().find("reset") != std::string::npos);
    assert(log.str().find("[串口][IMU][错误]") != std::string::npos);
}

void test_serial_command_reset_failure_is_reported() {
    std::ostringstream log;
    mosas::app::SerialCommandSink sink(nullptr, log);
    sink.reset();
    assert(!sink.reset_ok());
    assert(sink.last_error().find("reset") != std::string::npos);
    assert(log.str().find("[串口][控制][错误]") != std::string::npos);
}

void test_serial_port_exposes_operation_error() {
    PseudoTerminal terminal;
    auto port = std::make_shared<serial_package::SerialPort>();
    serial_package::SerialPortConfig config;
    config.device = terminal.slave_name;
    config.baud_rate = 115200;
    config.read_timeout_ms = 17;
    config.write_timeout_ms = 23;
    assert(port->open(config));
    assert(port->read_timeout_ms() == 17);
    assert(port->write_timeout_ms() == 23);
    port->close();

    std::array<std::uint8_t, 8> input{};
    std::string operation_error;
    assert(port->read(input.data(), input.size(), 50, &operation_error) < 0);
    assert(operation_error.find("not open") != std::string::npos);
}

void test_serial_port_connection_generation_changes_on_reopen() {
    PseudoTerminal terminal;
    auto port = std::make_shared<serial_package::SerialPort>();
    serial_package::SerialPortConfig config;
    config.device = terminal.slave_name;
    assert(port->open(config));
    const std::uint64_t first_generation = port->connection_generation();
    port->close();
    assert(port->open(config));
    assert(port->connection_generation() != first_generation);
}

}  // namespace

int main() {
    test_serial_command_sink_sends_zero_z_y_overloads();
    test_serial_imu_source_decodes_rad_s_and_logs_initialization_g();
    test_serial_command_error_is_available_through_sink_interface();
    test_serial_imu_source_reports_timeout_cancel_and_closed_port();
    test_serial_imu_cancel_wakes_a_long_read();
    test_serial_command_sink_can_be_cancelled_through_interface();
    test_serial_imu_reset_flushes_stale_input();
    test_serial_imu_reset_failure_is_reported();
    test_serial_command_reset_failure_is_reported();
    test_serial_port_exposes_operation_error();
    test_serial_port_connection_generation_changes_on_reopen();
    return 0;
}
