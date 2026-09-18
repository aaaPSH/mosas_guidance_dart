#include <serial_package/serial_port.hpp>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <limits>
#include <pthread.h>
#include <sys/ioctl.h>
#include <poll.h>
#include <sched.h>
#include <sstream>
#include <termios.h>
#include <unistd.h>

namespace serial_package {
namespace {

bool valid_config(const SerialPortConfig& config) {
    return !config.device.empty() && config.baud_rate != 0 &&
           config.data_bits >= 5 && config.data_bits <= 8 &&
           config.read_timeout_ms >= 0 && config.write_timeout_ms >= 0;
}

bool baud_rate_to_speed(uint32_t baud_rate, speed_t* speed) {
    if (speed == nullptr) {
        return false;
    }

    switch (baud_rate) {
        case 50:
            *speed = B50;
            return true;
        case 75:
            *speed = B75;
            return true;
        case 110:
            *speed = B110;
            return true;
        case 134:
            *speed = B134;
            return true;
        case 150:
            *speed = B150;
            return true;
        case 200:
            *speed = B200;
            return true;
        case 300:
            *speed = B300;
            return true;
        case 600:
            *speed = B600;
            return true;
        case 1200:
            *speed = B1200;
            return true;
        case 1800:
            *speed = B1800;
            return true;
        case 2400:
            *speed = B2400;
            return true;
        case 4800:
            *speed = B4800;
            return true;
        case 9600:
            *speed = B9600;
            return true;
        case 19200:
            *speed = B19200;
            return true;
        case 38400:
            *speed = B38400;
            return true;
        case 57600:
            *speed = B57600;
            return true;
        case 115200:
            *speed = B115200;
            return true;
        case 230400:
            *speed = B230400;
            return true;
        case 460800:
            *speed = B460800;
            return true;
        case 921600:
            *speed = B921600;
            return true;
        default:
            return false;
    }
}

std::string errno_message(const char* operation) {
    std::ostringstream message;
    message << operation << ": " << std::strerror(errno);
    return message.str();
}

std::int64_t monotonic_timestamp_ns() noexcept {
    timespec timestamp{};
    if (clock_gettime(CLOCK_MONOTONIC, &timestamp) != 0 ||
        timestamp.tv_sec < 0 || timestamp.tv_nsec < 0 ||
        timestamp.tv_nsec >= 1'000'000'000 ||
        timestamp.tv_sec >
            std::numeric_limits<std::int64_t>::max() / 1'000'000'000) {
        return -1;
    }
    const auto seconds_ns = static_cast<std::int64_t>(timestamp.tv_sec) *
                            1'000'000'000;
    if (seconds_ns > std::numeric_limits<std::int64_t>::max() -
                         timestamp.tv_nsec) {
        return -1;
    }
    return seconds_ns + timestamp.tv_nsec;
}

std::int64_t wire_byte_period_ns(const SerialPortConfig& config) noexcept {
    const std::int64_t parity_bits = config.parity == Parity::none ? 0 : 1;
    const std::int64_t stop_bits = config.stop_bits == StopBits::two ? 2 : 1;
    const std::int64_t bits_per_byte =
        1 + static_cast<std::int64_t>(config.data_bits) + parity_bits +
        stop_bits;
    const std::int64_t numerator = bits_per_byte * 1'000'000'000LL;
    const std::int64_t baud = static_cast<std::int64_t>(config.baud_rate);
    return (numerator + baud - 1) / baud;
}

std::int64_t subtract_wire_delay(std::int64_t timestamp_ns,
                                 std::size_t bytes,
                                 std::int64_t byte_period_ns) noexcept {
    if (timestamp_ns < 0 || bytes == 0 || byte_period_ns <= 0) {
        return timestamp_ns;
    }
    const auto maximum = std::numeric_limits<std::int64_t>::max();
    if (bytes > static_cast<std::size_t>(maximum / byte_period_ns)) {
        return 0;
    }
    const auto delay = byte_period_ns * static_cast<std::int64_t>(bytes);
    return delay >= timestamp_ns ? 0 : timestamp_ns - delay;
}

}  // namespace

SerialPort::~SerialPort() {
    close();
}

bool SerialPort::open(const SerialPortConfig& config) {
    close();
    if (!valid_config(config)) {
        set_error("invalid serial port configuration");
        return false;
    }

    int descriptor = ::open(config.device.c_str(), O_RDWR | O_NOCTTY | O_CLOEXEC);
    if (descriptor < 0) {
        set_error(errno_message("open serial port"));
        return false;
    }

    fd_ = descriptor;
    config_ = config;
    if (!configure(config_)) {
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    // 独占访问由设备驱动决定；TIOCEXCL 不适用于所有伪终端，因此不强制设置。
    {
        std::lock_guard<std::mutex> lock(error_mutex_);
        last_error_.clear();
    }
    return true;
}

void SerialPort::close() noexcept {
    stop_receive();
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool SerialPort::is_open() const noexcept {
    return fd_ >= 0;
}

ssize_t SerialPort::read(void* buffer, size_t size, int timeout_ms) {
    if (buffer == nullptr && size != 0) {
        set_error("read buffer is null");
        return -1;
    }
    if (!is_open()) {
        set_error("serial port is not open");
        return -1;
    }
    if (size == 0) {
        return 0;
    }

    const int effective_timeout = timeout_ms < 0 ? config_.read_timeout_ms : timeout_ms;
    bool timed_out = false;
    if (!wait_for(POLLIN, effective_timeout, &timed_out)) {
        return timed_out ? 0 : -1;
    }

    const ssize_t count = ::read(fd_, buffer, size);
    if (count < 0) {
        if (errno == EINTR) {
            return 0;
        }
        set_error(errno_message("read serial port"));
    }
    return count;
}

ssize_t SerialPort::input_bytes_available() {
    if (!is_open()) {
        set_error("serial port is not open");
        return -1;
    }

    int bytes = 0;
    if (::ioctl(fd_, FIONREAD, &bytes) != 0) {
        set_error(errno_message("query serial input bytes"));
        return -1;
    }
    if (bytes < 0) {
        set_error("serial input byte count is negative");
        return -1;
    }
    return static_cast<ssize_t>(bytes);
}

ssize_t SerialPort::write(const void* data, size_t size, int timeout_ms) {
    if (data == nullptr && size != 0) {
        set_error("write data is null");
        return -1;
    }
    if (!is_open()) {
        set_error("serial port is not open");
        return -1;
    }
    if (size == 0) {
        return 0;
    }

    std::lock_guard<std::mutex> lock(write_mutex_);
    const int effective_timeout = timeout_ms < 0 ? config_.write_timeout_ms : timeout_ms;
    bool timed_out = false;
    if (!wait_for(POLLOUT, effective_timeout, &timed_out)) {
        return timed_out ? 0 : -1;
    }

    const ssize_t count = ::write(fd_, data, size);
    if (count < 0) {
        if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
            return 0;
        }
        set_error(errno_message("write serial port"));
    }
    return count;
}

bool SerialPort::write_all(const void* data, size_t size, int timeout_ms) {
    if (data == nullptr && size != 0) {
        set_error("write data is null");
        return false;
    }
    const auto start = std::chrono::steady_clock::now();
    size_t written = 0;
    while (written < size) {
        int remaining_timeout = timeout_ms;
        if (timeout_ms >= 0) {
            const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - start);
            remaining_timeout = timeout_ms - static_cast<int>(elapsed.count());
            if (remaining_timeout < 0) {
                remaining_timeout = 0;
            }
        }

        const ssize_t count = write(static_cast<const uint8_t*>(data) + written,
                                    size - written, remaining_timeout);
        if (count < 0) {
            return false;
        }
        if (count == 0) {
            set_error("write serial port timeout");
            return false;
        }
        written += static_cast<size_t>(count);
    }
    return true;
}

bool SerialPort::start_receive(ReceiveCallback callback, void* context,
                               const ReceiveThreadConfig& config) {
    if (callback == nullptr) {
        set_error("receive callback is null");
        return false;
    }
    if (!is_open()) {
        set_error("serial port is not open");
        return false;
    }
    if (config.read_chunk_size == 0) {
        set_error("receive chunk size must be positive");
        return false;
    }
    if (config.realtime_priority < 0 || config.realtime_priority > 99) {
        set_error("receive realtime priority is out of range");
        return false;
    }

    std::lock_guard<std::mutex> lock(receive_mutex_);
    if (receive_thread_.joinable()) {
        set_error("serial receive thread is already running");
        return false;
    }

    receive_callback_ = callback;
    receive_context_ = context;
    receive_config_ = config;
    receive_realtime_priority_applied_.store(false,
                                             std::memory_order_release);
    receive_priority_error_.store(false, std::memory_order_release);
    receive_stop_requested_.store(false, std::memory_order_release);
    receive_running_.store(true, std::memory_order_release);
    try {
        receive_thread_ = std::thread(&SerialPort::receive_loop, this);
    } catch (...) {
        receive_running_.store(false, std::memory_order_release);
        receive_callback_ = nullptr;
        receive_context_ = nullptr;
        set_error("failed to start serial receive thread");
        return false;
    }
    return true;
}

void SerialPort::stop_receive() noexcept {
    std::thread receive_thread;
    {
        std::lock_guard<std::mutex> lock(receive_mutex_);
        receive_stop_requested_.store(true, std::memory_order_release);
        receive_thread.swap(receive_thread_);
    }
    if (receive_thread.joinable()) {
        receive_thread.join();
    }
    receive_running_.store(false, std::memory_order_release);
    receive_callback_ = nullptr;
    receive_context_ = nullptr;
}

bool SerialPort::receive_running() const noexcept {
    return receive_running_.load(std::memory_order_acquire);
}

bool SerialPort::configure_receive_thread(
    const ReceiveThreadConfig& config) noexcept {
    (void)pthread_setname_np(pthread_self(), "mosas-serial-rx");

    if (config.realtime_priority > 0) {
        sched_param parameters{};
        parameters.sched_priority = config.realtime_priority;
        const int result = pthread_setschedparam(
            pthread_self(), SCHED_FIFO, &parameters);
        if (result != 0) {
            receive_priority_error_.store(true, std::memory_order_release);
            if (config.require_realtime_priority) {
                set_error("failed to set serial receive realtime priority");
                return false;
            }
        } else {
            receive_realtime_priority_applied_.store(
                true, std::memory_order_release);
        }
    }

#ifdef __linux__
    if (config.cpu_affinity >= 0) {
        cpu_set_t cpu_set;
        CPU_ZERO(&cpu_set);
        CPU_SET(config.cpu_affinity, &cpu_set);
        const int result = pthread_setaffinity_np(
            pthread_self(), sizeof(cpu_set), &cpu_set);
        if (result != 0) {
            receive_priority_error_.store(true, std::memory_order_release);
            if (config.require_realtime_priority) {
                set_error("failed to set serial receive CPU affinity");
                return false;
            }
        }
    }
#endif
    return true;
}

void SerialPort::receive_loop() noexcept {
    const ReceiveThreadConfig receive_config = receive_config_;
    const ReceiveCallback callback = receive_callback_;
    void* const context = receive_context_;
    const std::size_t chunk_size = std::min<std::size_t>(
        receive_config.read_chunk_size, std::size_t{256});
    const std::int64_t byte_period_ns = wire_byte_period_ns(config_);
    std::array<std::uint8_t, 256> buffer{};

    if (!configure_receive_thread(receive_config)) {
        receive_running_.store(false, std::memory_order_release);
        return;
    }

    while (!receive_stop_requested_.load(std::memory_order_acquire)) {
        bool timed_out = false;
        if (!wait_for(POLLIN, config_.read_timeout_ms, &timed_out)) {
            if (receive_stop_requested_.load(std::memory_order_acquire)) {
                break;
            }
            if (timed_out) {
                continue;
            }
            break;
        }

        const std::int64_t read_start_ns = monotonic_timestamp_ns();
        if (read_start_ns < 0) {
            set_error("failed to read monotonic timestamp for serial input");
            break;
        }
        const ssize_t pending_bytes = input_bytes_available();
        if (pending_bytes < 0) {
            break;
        }

        const ssize_t count = ::read(fd_, buffer.data(), chunk_size);
        if (count < 0) {
            if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
                continue;
            }
            set_error(errno_message("read serial port"));
            break;
        }
        if (count == 0) {
            continue;
        }

        const std::int64_t first_byte_ns = subtract_wire_delay(
            read_start_ns, static_cast<std::size_t>(pending_bytes),
            byte_period_ns);
        callback(context, buffer.data(), static_cast<std::size_t>(count),
                 first_byte_ns);
    }

    receive_running_.store(false, std::memory_order_release);
}

bool SerialPort::flush_input() {
    if (!is_open()) {
        set_error("serial port is not open");
        return false;
    }
    if (tcflush(fd_, TCIFLUSH) != 0) {
        set_error(errno_message("flush serial input"));
        return false;
    }
    return true;
}

bool SerialPort::flush_output() {
    if (!is_open()) {
        set_error("serial port is not open");
        return false;
    }
    if (tcflush(fd_, TCOFLUSH) != 0) {
        set_error(errno_message("flush serial output"));
        return false;
    }
    return true;
}

std::string SerialPort::last_error() const {
    std::lock_guard<std::mutex> lock(error_mutex_);
    return last_error_;
}

int SerialPort::native_handle() const noexcept {
    return fd_;
}

bool SerialPort::configure(const SerialPortConfig& config) {
    termios options{};
    if (tcgetattr(fd_, &options) != 0) {
        set_error(errno_message("get serial attributes"));
        return false;
    }

    speed_t speed{};
    if (!baud_rate_to_speed(config.baud_rate, &speed)) {
        set_error("unsupported baud rate");
        return false;
    }

    cfmakeraw(&options);
    options.c_cflag |= CLOCAL | CREAD;
    options.c_cflag &= ~CSIZE;
    switch (config.data_bits) {
        case 5:
            options.c_cflag |= CS5;
            break;
        case 6:
            options.c_cflag |= CS6;
            break;
        case 7:
            options.c_cflag |= CS7;
            break;
        case 8:
            options.c_cflag |= CS8;
            break;
        default:
            set_error("unsupported data bits");
            return false;
    }

    if (config.stop_bits == StopBits::two) {
        options.c_cflag |= CSTOPB;
    } else {
        options.c_cflag &= ~CSTOPB;
    }

    if (config.parity == Parity::none) {
        options.c_cflag &= ~PARENB;
    } else {
        options.c_cflag |= PARENB;
        if (config.parity == Parity::odd) {
            options.c_cflag |= PARODD;
        } else {
            options.c_cflag &= ~PARODD;
        }
    }

    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_cflag &= ~CRTSCTS;
    if (config.flow_control == FlowControl::software) {
        options.c_iflag |= IXON | IXOFF;
    } else if (config.flow_control == FlowControl::hardware) {
        options.c_cflag |= CRTSCTS;
    }

    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;
    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);

    if (tcsetattr(fd_, TCSANOW, &options) != 0) {
        set_error(errno_message("set serial attributes"));
        return false;
    }
    return true;
}

bool SerialPort::wait_for(short events, int timeout_ms, bool* timed_out) {
    if (timed_out != nullptr) {
        *timed_out = false;
    }
    pollfd descriptor{fd_, events, 0};
    while (true) {
        const int result = ::poll(&descriptor, 1, timeout_ms);
        if (result > 0) {
            if ((descriptor.revents & events) != 0) {
                return true;
            }
            set_error("serial port reported an I/O error");
            return false;
        }
        if (result == 0) {
            if (timed_out != nullptr) {
                *timed_out = true;
            }
            return false;
        }
        if (errno == EINTR) {
            continue;
        }
        set_error(errno_message("poll serial port"));
        return false;
    }
}

void SerialPort::set_error(const std::string& message) {
    std::lock_guard<std::mutex> lock(error_mutex_);
    last_error_ = message;
}

}  // namespace serial_package
