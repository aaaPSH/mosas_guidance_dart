#include <serial_package/serial_port.hpp>

#include <pty.h>
#include <sched.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <cassert>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <string>
#include <thread>

namespace {

struct Capture {
    std::array<std::uint8_t, 64> bytes{};
    std::atomic<std::size_t> size{0};
    std::atomic<std::int64_t> timestamp_ns{-1};
    std::atomic<bool> received{false};
};

void capture_callback(void* context, const std::uint8_t* data,
                      std::size_t size,
                      std::int64_t first_byte_timestamp_ns) noexcept {
    auto* capture = static_cast<Capture*>(context);
    if (capture == nullptr || data == nullptr || size > capture->bytes.size()) {
        return;
    }
    std::memcpy(capture->bytes.data(), data, size);
    capture->timestamp_ns.store(first_byte_timestamp_ns,
                                std::memory_order_relaxed);
    capture->size.store(size, std::memory_order_relaxed);
    capture->received.store(true, std::memory_order_release);
}

void test_async_receive_delivers_raw_bytes_and_timestamp() {
    int master_fd = -1;
    int slave_fd = -1;
    char slave_name[128]{};
    assert(openpty(&master_fd, &slave_fd, slave_name, nullptr, nullptr) == 0);

    serial_package::SerialPort port;
    serial_package::SerialPortConfig config;
    config.device = slave_name;
    config.baud_rate = 115200;
    config.read_timeout_ms = 20;
    config.write_timeout_ms = 20;
    assert(port.open(config));

    Capture capture;
    serial_package::ReceiveThreadConfig receive_config;
    receive_config.read_chunk_size = 64;
    assert(port.start_receive(capture_callback, &capture, receive_config));
    assert(port.receive_running());

    const std::array<std::uint8_t, 4> payload{0x01, 0x55, 0xAA, 0xFF};
    assert(::write(master_fd, payload.data(), payload.size()) ==
           static_cast<ssize_t>(payload.size()));

    for (int attempt = 0; attempt < 200 &&
                              !capture.received.load(std::memory_order_acquire);
         ++attempt) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    assert(capture.received.load(std::memory_order_acquire));
    assert(capture.size.load(std::memory_order_relaxed) == payload.size());
    assert(std::memcmp(capture.bytes.data(), payload.data(), payload.size()) ==
           0);
    assert(capture.timestamp_ns.load(std::memory_order_relaxed) >= 0);

    port.stop_receive();
    port.close();
    ::close(master_fd);
    ::close(slave_fd);
}

void test_invalid_cpu_affinity_is_rejected_before_thread_start() {
    int master_fd = -1;
    int slave_fd = -1;
    char slave_name[128]{};
    assert(openpty(&master_fd, &slave_fd, slave_name, nullptr, nullptr) == 0);

    serial_package::SerialPort port;
    serial_package::SerialPortConfig config;
    config.device = slave_name;
    assert(port.open(config));

    serial_package::ReceiveThreadConfig receive_config;
    receive_config.cpu_affinity = CPU_SETSIZE;
    assert(!port.start_receive(capture_callback, nullptr, receive_config));
    assert(!port.receive_running());

    port.close();
    ::close(master_fd);
    ::close(slave_fd);
}

}  // namespace

int main() {
    test_async_receive_delivers_raw_bytes_and_timestamp();
    test_invalid_cpu_affinity_is_rejected_before_thread_start();
    return 0;
}
