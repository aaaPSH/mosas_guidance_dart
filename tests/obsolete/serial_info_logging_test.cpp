#include <mosas/app/host_adapters.hpp>
#include <mosas/app/serial_imu_source.hpp>
#include <serial_package/serial_port.hpp>
#include <serial_package/serial_protocol.hpp>

#include <array>
#include <cassert>
#include <cstdint>
#include <fcntl.h>
#include <pty.h>
#include <sstream>
#include <string>
#include <unistd.h>

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

std::shared_ptr<serial_package::SerialPort> open_port(
    const PseudoTerminal& terminal) {
    auto port = std::make_shared<serial_package::SerialPort>();
    serial_package::SerialPortConfig config;
    config.device = terminal.slave_name;
    config.read_timeout_ms = 50;
    config.write_timeout_ms = 50;
    assert(port->open(config));
    return port;
}

void test_control_send_logs_info_and_bytes() {
    PseudoTerminal terminal;
    auto port = open_port(terminal);
    std::ostringstream log;
    mosas::app::SerialCommandSink sink(port, log);

    const mosas::runtime::GuidanceCommand command{
        123,
        {true, {}, {}, {99.0, 1.25, -2.5}, 0.0, 0.0}};
    assert(sink.send(command));

    const std::string text = log.str();
    assert(text.find("[INFO][串口][控制][发送]") != std::string::npos);
    assert(text.find("AA 55 00 00 06 FF 7D 00 00 00 83 9D 0D 0A") !=
           std::string::npos);
}

void test_imu_receive_logs_info_bytes_and_decoded_values() {
    PseudoTerminal terminal;
    auto port = open_port(terminal);
    std::ostringstream log;
    mosas::app::SerialImuSource source(port, 200.0, 50, log);

    const std::array<std::uint8_t, serial_package::kImuFrameSize> frame{
        0x55, 0xAA, 0x64, 0x00, 0x38, 0xFF, 0x2C, 0x01,
        0x0A, 0x00, 0xEC, 0xFF, 0x1E, 0x00, 0x34, 0x12,
        0x6B, 0x25, 0x0D, 0x0A};
    mosas::runtime::ImuSample sample{};
    assert(source.read(&sample).status ==
           mosas::runtime::SourceStatus::timeout);
    assert(::write(terminal.master, frame.data(), frame.size()) ==
           static_cast<ssize_t>(frame.size()));

    assert(source.read(&sample).status == mosas::runtime::SourceStatus::ok);

    const std::string text = log.str();
    assert(text.find("[INFO][串口][IMU][接收]") != std::string::npos);
    assert(text.find("55 AA 64 00 38 FF 2C 01 0A 00 EC FF 1E 00 34 12 6B 25 0D 0A") !=
           std::string::npos);
    assert(text.find("g_bytes=0x1234") != std::string::npos);
}

}  // namespace

int main() {
    test_control_send_logs_info_and_bytes();
    test_imu_receive_logs_info_bytes_and_decoded_values();
    return 0;
}
