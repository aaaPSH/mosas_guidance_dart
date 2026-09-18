#include <mosas/app/host_adapters.hpp>
#include <mosas/runtime/async_info_logger.hpp>
#include <mosas/runtime/latest_value_mailbox.hpp>
#include <serial_package/serial_protocol.hpp>

#include <pty.h>
#include <poll.h>
#include <unistd.h>

#include <array>
#include <cassert>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <sstream>
#include <thread>

namespace {

void test_latest_value_mailbox_keeps_newest_value() {
    mosas::runtime::LatestValueMailbox<int> mailbox;
    assert(mailbox.try_publish(1));
    assert(mailbox.try_publish(2));
    int value = 0;
    assert(mailbox.try_consume(&value));
    assert(value == 2);
    assert(mailbox.dropped_count() == 1);
}

void test_async_logger_formats_on_worker_thread() {
    std::ostringstream output;
    mosas::runtime::AsyncInfoLogger logger(output);
    assert(logger.start());
    mosas::runtime::RuntimeLogEvent event{};
    event.kind = mosas::runtime::RuntimeLogEventKind::guidance_command;
    event.timestamp_ns = 123;
    event.value0 = 1.5;
    assert(logger.try_submit(event));
    logger.stop();
    assert(output.str().find("timestamp_ns=123") != std::string::npos);
}

void test_serial_command_submission_is_async() {
    int master_fd = -1;
    int slave_fd = -1;
    char slave_name[128]{};
    assert(openpty(&master_fd, &slave_fd, slave_name, nullptr, nullptr) == 0);

    auto port = std::make_shared<serial_package::SerialPort>();
    serial_package::SerialPortConfig config;
    config.device = slave_name;
    config.write_timeout_ms = 20;
    config.read_timeout_ms = 20;
    assert(port->open(config));

    mosas::app::SerialCommandSink sink(port);
    assert(sink.start());
    mosas::runtime::GuidanceCommand command{};
    command.timestamp_ns = 123;
    command.output.valid = true;
    command.output.body_overload = {0.0, 0.25, -0.5};
    const auto started = std::chrono::steady_clock::now();
    assert(sink.try_submit(command));
    assert(std::chrono::steady_clock::now() - started <
           std::chrono::milliseconds(10));

    std::array<std::uint8_t, serial_package::kControlFrameSize> frame{};
    pollfd descriptor{master_fd, POLLIN, 0};
    assert(::poll(&descriptor, 1, 500) == 1);
    assert(::read(master_fd, frame.data(), frame.size()) ==
           static_cast<ssize_t>(frame.size()));

    sink.stop();
    port->close();
    ::close(master_fd);
    ::close(slave_fd);
}

}  // namespace

int main() {
    test_latest_value_mailbox_keeps_newest_value();
    test_async_logger_formats_on_worker_thread();
    test_serial_command_submission_is_async();
    return 0;
}
