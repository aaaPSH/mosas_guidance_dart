#include <mosas/app/serial_imu_source.hpp>
#include <serial_package/serial_protocol.hpp>

#include <pty.h>
#include <unistd.h>

#include <array>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <thread>

namespace {

constexpr double kDegreesToRadians = 0.017453292519943295;

std::array<std::uint8_t, serial_package::kImuFrameSize> make_frame() {
    std::array<std::uint8_t, serial_package::kImuFrameSize> frame{};
    frame[0] = 0x55;
    frame[1] = 0xAA;
    // 加速度保持为零，三轴角速度分别编码为 1.0、-2.0、3.0 deg/s。
    frame[8] = 10;
    frame[10] = 0xEC;
    frame[11] = 0xFF;
    frame[12] = 30;
    frame[18] = 0x0D;
    frame[19] = 0x0A;
    const std::uint16_t crc = serial_package::calculate_imu_frame_crc(frame);
    frame[16] = static_cast<std::uint8_t>(crc & 0xFFu);
    frame[17] = static_cast<std::uint8_t>((crc >> 8u) & 0xFFu);
    return frame;
}

void test_serial_imu_source_decodes_on_consumer_thread() {
    int master_fd = -1;
    int slave_fd = -1;
    char slave_name[128]{};
    assert(openpty(&master_fd, &slave_fd, slave_name, nullptr, nullptr) == 0);

    auto port = std::make_shared<serial_package::SerialPort>();
    serial_package::SerialPortConfig config;
    config.device = slave_name;
    config.baud_rate = 115200;
    config.read_timeout_ms = 20;
    config.write_timeout_ms = 20;
    assert(port->open(config));

    mosas::app::SerialImuSource source(port, 200.0, 20);
    const auto start_result = source.start();
    assert(start_result.status == mosas::runtime::SourceStatus::ok);
    bool priority_reported = false;
    for (int attempt = 0; attempt < 100 && !priority_reported; ++attempt) {
        const auto diagnostics = source.diagnostics();
        priority_reported =
            diagnostics.receive_realtime_priority_applied ||
            diagnostics.receive_priority_error;
        if (!priority_reported) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    assert(priority_reported);

    const auto frame = make_frame();
    assert(::write(master_fd, frame.data(), frame.size()) ==
           static_cast<ssize_t>(frame.size()));

    mosas::runtime::ImuAngularVelocitySample sample{};
    bool received = false;
    for (int attempt = 0; attempt < 300 && !received; ++attempt) {
        received = source.try_pop(&sample);
        if (!received) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    assert(received);
    assert(sample.timestamp_ns >= 0);
    assert(std::abs(sample.angular_velocity.x - kDegreesToRadians) < 1e-9);
    assert(std::abs(sample.angular_velocity.y + 2.0 * kDegreesToRadians) <
           1e-9);
    assert(std::abs(sample.angular_velocity.z - 3.0 * kDegreesToRadians) <
           1e-9);

    source.stop();

    serial_package::ReceiveThreadConfig strict_receive_config;
    strict_receive_config.realtime_priority = 20;
    strict_receive_config.require_realtime_priority = true;
    mosas::app::SerialImuSource strict_source(
        port, 200.0, 20, false, std::cerr, 0, strict_receive_config);
    const auto strict_start_result = strict_source.start();
    const auto strict_diagnostics = strict_source.diagnostics();
    if (strict_diagnostics.receive_priority_error) {
        assert(strict_start_result.status ==
               mosas::runtime::SourceStatus::fatal);
    } else {
        assert(strict_start_result.status == mosas::runtime::SourceStatus::ok);
    }
    strict_source.stop();

    port->close();
    ::close(master_fd);
    ::close(slave_fd);
}

}  // namespace

int main() {
    test_serial_imu_source_decodes_on_consumer_thread();
    return 0;
}
