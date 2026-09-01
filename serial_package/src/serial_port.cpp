#include "serial_package/serial_port.hpp"

#include <cerrno>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
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
