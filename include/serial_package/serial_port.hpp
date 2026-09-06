#ifndef SERIAL_PACKAGE_SERIAL_PORT_HPP
#define SERIAL_PACKAGE_SERIAL_PORT_HPP

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <sys/types.h>

namespace serial_package {

enum class Parity {
    none,
    even,
    odd,
};

enum class StopBits {
    one,
    two,
};

enum class FlowControl {
    none,
    software,
    hardware,
};

struct SerialPortConfig {
    std::string device;
    uint32_t baud_rate = 115200;
    uint8_t data_bits = 8;
    StopBits stop_bits = StopBits::one;
    Parity parity = Parity::none;
    FlowControl flow_control = FlowControl::none;
    int read_timeout_ms = 100;
    int write_timeout_ms = 100;
};

class SerialPort {
public:
    SerialPort() = default;
    ~SerialPort();

    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    // 使用配置打开 Linux 串口设备，例如 /dev/ttyS3。
    bool open(const SerialPortConfig& config);

    // 关闭串口。调用方需要保证关闭时没有并发读写。
    void close() noexcept;

    bool is_open() const noexcept;

    // 读取一段原始二进制数据。返回读取字节数，0 表示超时，-1 表示失败。
    // timeout_ms 为负数时使用配置中的 read_timeout_ms。
    ssize_t read(void* buffer, size_t size, int timeout_ms = -1);

    // 查询 Linux 内核输入队列中的字节数；失败返回 -1。
    ssize_t input_bytes_available();

    // 写入一段原始二进制数据。返回实际写入字节数，-1 表示失败。
    // timeout_ms 为负数时使用配置中的 write_timeout_ms。
    ssize_t write(const void* data, size_t size, int timeout_ms = -1);

    // 持续写入直到全部数据发送完成、超时或失败。
    bool write_all(const void* data, size_t size, int timeout_ms = -1);

    bool flush_input();
    bool flush_output();

    // 返回最近一次错误信息；成功操作不会自动清除之前的错误。
    std::string last_error() const;

    // 返回底层文件描述符，仅用于调试或集成 poll/epoll；未打开时返回 -1。
    int native_handle() const noexcept;

private:
    bool configure(const SerialPortConfig& config);
    bool wait_for(short events, int timeout_ms, bool* timed_out);
    void set_error(const std::string& message);

    int fd_ = -1;
    SerialPortConfig config_;
    mutable std::mutex error_mutex_;
    mutable std::mutex write_mutex_;
    std::string last_error_;
};

}  // namespace serial_package

#endif  // SERIAL_PACKAGE_SERIAL_PORT_HPP
