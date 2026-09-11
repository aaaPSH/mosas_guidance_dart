#include <serial_package/serial_port.hpp>
#include <serial_package/serial_protocol.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <charconv>
#include <chrono>
#include <cstdint>
#include <cerrno>
#include <csignal>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <poll.h>
#include <sstream>
#include <string>
#include <termios.h>
#include <thread>
#include <unistd.h>

namespace {

volatile std::sig_atomic_t g_stop_requested = 0;

void handle_signal(int) {
    g_stop_requested = 1;
}

struct Options {
    std::string device = "/dev/ttyS3";
    std::uint32_t baud_rate = 115200;
    int read_timeout_ms = 100;
    int write_timeout_ms = 100;
    int send_interval_ms = 2;
    bool show_help = false;
};

void print_usage(const char* program) {
    std::cout
        << "用法：" << program << " [选项]\n"
        << "\n"
        << "程序先接收并打印 20 字节 IMU 帧，按任意键后开始周期发送固定控制帧。\n"
        << "固定控制帧的三个数据均为 1.0，发送时不打印帧字节。\n"
        << "\n"
        << "选项：\n"
        << "  --device PATH             串口设备，默认 /dev/ttyS3\n"
        << "  --baud-rate RATE          波特率，默认 115200\n"
        << "  --read-timeout-ms MS      接收超时，默认 100\n"
        << "  --write-timeout-ms MS     发送超时，默认 100\n"
        << "  --send-interval-ms MS     发送间隔，默认 2（约 500 Hz）\n"
        << "  -h, --help                显示帮助\n";
}

template <typename Integer>
bool parse_integer(const std::string& text, Integer* value) {
    if (value == nullptr || text.empty()) {
        return false;
    }

    Integer parsed{};
    const char* begin = text.data();
    const char* end = begin + text.size();
    const auto result = std::from_chars(begin, end, parsed);
    if (result.ec != std::errc{} || result.ptr != end) {
        return false;
    }
    *value = parsed;
    return true;
}

bool take_option_value(int argc, char** argv, int* index,
                       const char* option, std::string* value) {
    if (index == nullptr || value == nullptr || *index + 1 >= argc) {
        std::cerr << option << " 缺少值\n";
        return false;
    }
    ++(*index);
    *value = argv[*index];
    return true;
}

bool parse_positive_option(int argc, char** argv, int* index,
                           const char* option, int* destination) {
    std::string text;
    if (!take_option_value(argc, argv, index, option, &text) ||
        !parse_integer(text, destination) || *destination <= 0) {
        std::cerr << option << " 必须是大于 0 的整数\n";
        return false;
    }
    return true;
}

bool parse_options(int argc, char** argv, Options* options) {
    if (options == nullptr) {
        return false;
    }

    for (int index = 1; index < argc; ++index) {
        const std::string argument(argv[index]);
        if (argument == "-h" || argument == "--help") {
            options->show_help = true;
        } else if (argument == "--device") {
            if (!take_option_value(argc, argv, &index, "--device",
                                   &options->device)) {
                return false;
            }
            if (options->device.empty()) {
                std::cerr << "--device 不能为空\n";
                return false;
            }
        } else if (argument == "--baud-rate") {
            std::string text;
            if (!take_option_value(argc, argv, &index, "--baud-rate",
                                   &text) ||
                !parse_integer(text, &options->baud_rate) ||
                options->baud_rate == 0) {
                std::cerr << "--baud-rate 必须是大于 0 的整数\n";
                return false;
            }
        } else if (argument == "--read-timeout-ms") {
            if (!parse_positive_option(argc, argv, &index, argument.c_str(),
                                       &options->read_timeout_ms)) {
                return false;
            }
        } else if (argument == "--write-timeout-ms") {
            if (!parse_positive_option(argc, argv, &index, argument.c_str(),
                                       &options->write_timeout_ms)) {
                return false;
            }
        } else if (argument == "--send-interval-ms") {
            if (!parse_positive_option(argc, argv, &index, argument.c_str(),
                                       &options->send_interval_ms)) {
                return false;
            }
        } else {
            std::cerr << "未知参数：" << argument << "\n";
            return false;
        }
    }
    return true;
}

std::string hex_dump(const std::uint8_t* data, std::size_t size) {
    std::ostringstream stream;
    stream << std::hex << std::uppercase << std::setfill('0');
    for (std::size_t index = 0; index < size; ++index) {
        if (index != 0) {
            stream << ' ';
        }
        stream << std::setw(2) << static_cast<unsigned int>(data[index]);
    }
    return stream.str();
}

void print_line(std::mutex* output_mutex, const std::string& message) {
    std::lock_guard<std::mutex> lock(*output_mutex);
    std::cout << message << '\n' << std::flush;
}

bool stopping(const std::atomic<bool>& stop_requested) {
    return stop_requested.load() || g_stop_requested != 0;
}

class ImuFrameCollector {
public:
    static constexpr std::size_t kCapacity = 4096;
    using Frame = std::array<std::uint8_t, serial_package::kImuFrameSize>;

    void append(const std::uint8_t* data, std::size_t size) {
        if (data == nullptr || size == 0) {
            return;
        }

        for (std::size_t index = 0; index < size; ++index) {
            if (buffered_size_ == buffer_.size()) {
                discard_prefix(1);
            }
            buffer_[buffered_size_++] = data[index];
        }
    }

    bool peek(Frame* frame) {
        if (frame == nullptr) {
            return false;
        }

        while (buffered_size_ > 0) {
            if (buffer_[0] != 0x55) {
                discard_prefix(1);
                continue;
            }
            if (buffered_size_ == 1) {
                return false;
            }
            if (buffer_[1] != 0xAA) {
                discard_prefix(1);
                continue;
            }
            break;
        }

        if (buffered_size_ < serial_package::kImuFrameSize) {
            return false;
        }

        std::copy_n(buffer_.begin(), serial_package::kImuFrameSize,
                    frame->begin());
        return true;
    }

    void discard(std::size_t count) { discard_prefix(count); }

private:
    void discard_prefix(std::size_t count) {
        if (count >= buffered_size_) {
            buffered_size_ = 0;
            return;
        }
        std::memmove(buffer_.data(), buffer_.data() + count,
                     buffered_size_ - count);
        buffered_size_ -= count;
    }

    std::array<std::uint8_t, kCapacity> buffer_{};
    std::size_t buffered_size_ = 0;
};

bool format_received_frame(const ImuFrameCollector::Frame& frame,
                           std::string* formatted_message) {
    if (formatted_message == nullptr) {
        return false;
    }

    std::ostringstream message;
    message << "[接收] IMU bytes=" << hex_dump(frame.data(), frame.size());

    serial_package::ImuFrameValues values;
    std::string error;
    if (!serial_package::decode_imu_frame(frame.data(), frame.size(), &values,
                                         &error)) {
        message << " 校验失败: " << error;
        *formatted_message = message.str();
        return false;
    }

    message << std::fixed << std::setprecision(3)
            << " acceleration_mps2=(" << values.acceleration_x_mps2 << ','
            << values.acceleration_y_mps2 << ','
            << values.acceleration_z_mps2 << ") angular_velocity_rad_s=("
            << values.angular_velocity_x_rad_s << ','
            << values.angular_velocity_y_rad_s << ','
            << values.angular_velocity_z_rad_s << ")";
    message << " initialization_g_raw=0x" << std::hex << std::uppercase
            << std::setfill('0') << std::setw(4)
            << static_cast<unsigned int>(values.initialization_g_raw)
            << " crc=0x" << std::setw(4)
            << static_cast<unsigned int>(values.crc);
    *formatted_message = message.str();
    return true;
}

void receive_loop(serial_package::SerialPort* port, const Options& options,
                  std::atomic<bool>* stop_requested,
                  std::mutex* output_mutex) {
    std::array<std::uint8_t, 256> read_buffer{};
    ImuFrameCollector collector;

    while (!stopping(*stop_requested)) {
        const ssize_t count = port->read(read_buffer.data(), read_buffer.size(),
                                         options.read_timeout_ms);
        if (count < 0) {
            const std::string error = port->last_error();
            print_line(output_mutex,
                       "[串口调试] 接收失败: " +
                           (error.empty() ? "未知串口错误" : error));
            stop_requested->store(true);
            return;
        }
        if (count == 0) {
            continue;
        }

        collector.append(read_buffer.data(), static_cast<std::size_t>(count));
        ImuFrameCollector::Frame frame{};
        while (collector.peek(&frame)) {
            std::string formatted_message;
            const bool valid =
                format_received_frame(frame, &formatted_message);
            print_line(output_mutex, formatted_message);
            collector.discard(valid ? frame.size() : std::size_t{1});
        }
    }
}

void send_loop(serial_package::SerialPort* port, const Options& options,
               const std::array<std::uint8_t, serial_package::kControlFrameSize>&
                   frame,
               std::atomic<bool>* stop_requested, std::mutex* output_mutex) {
    std::size_t sent_count = 0;
    auto next_send_at = std::chrono::steady_clock::now();
    const auto interval = std::chrono::milliseconds(options.send_interval_ms);

    while (!stopping(*stop_requested)) {
        if (!port->write_all(frame.data(), frame.size(),
                             options.write_timeout_ms)) {
            const std::string error = port->last_error();
            print_line(output_mutex,
                       "[串口调试] 发送失败: " +
                           (error.empty() ? "控制帧写入失败" : error));
            stop_requested->store(true);
            return;
        }
        ++sent_count;

        next_send_at += interval;
        const auto now = std::chrono::steady_clock::now();
        if (next_send_at <= now) {
            next_send_at = now + interval;
        }
        std::this_thread::sleep_until(next_send_at);
    }

    std::ostringstream message;
    message << "[串口调试] 发送线程已停止，共发送 " << sent_count << " 帧";
    print_line(output_mutex, message.str());
}

class TerminalModeGuard {
public:
    explicit TerminalModeGuard(int descriptor) : descriptor_(descriptor) {
        if (!::isatty(descriptor_)) {
            return;
        }
        if (::tcgetattr(descriptor_, &original_) != 0) {
            return;
        }

        termios raw = original_;
        raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
        raw.c_cc[VMIN] = 0;
        raw.c_cc[VTIME] = 0;
        if (::tcsetattr(descriptor_, TCSANOW, &raw) == 0) {
            active_ = true;
        }
    }

    ~TerminalModeGuard() {
        if (active_) {
            ::tcsetattr(descriptor_, TCSANOW, &original_);
        }
    }

    bool active() const noexcept { return active_; }

private:
    int descriptor_ = -1;
    termios original_{};
    bool active_ = false;
};

bool wait_for_any_key(const std::atomic<bool>& stop_requested,
                      std::mutex* output_mutex) {
    TerminalModeGuard terminal_mode(STDIN_FILENO);
    if (::isatty(STDIN_FILENO) && !terminal_mode.active()) {
        print_line(output_mutex, "[串口调试] 设置终端按键模式失败");
        return false;
    }

    while (!stopping(stop_requested)) {
        pollfd descriptor{STDIN_FILENO, POLLIN, 0};
        const int result = ::poll(&descriptor, 1, 100);
        if (result < 0) {
            if (errno == EINTR) {
                continue;
            }
            print_line(output_mutex, "[串口调试] 等待按键失败");
            return false;
        }
        if (result == 0) {
            continue;
        }
        if ((descriptor.revents & (POLLIN | POLLHUP)) == 0) {
            continue;
        }

        std::uint8_t key = 0;
        const ssize_t count = ::read(STDIN_FILENO, &key, sizeof(key));
        if (count == 1) {
            return true;
        }
        if (count == 0) {
            print_line(output_mutex, "[串口调试] 标准输入已关闭，程序退出");
            return false;
        }
        if (errno != EINTR) {
            print_line(output_mutex, "[串口调试] 读取按键失败");
            return false;
        }
    }
    return false;
}

}  // 匿名命名空间结束

int main(int argc, char** argv) {
    Options options;
    if (!parse_options(argc, argv, &options)) {
        print_usage(argv[0]);
        return 2;
    }
    if (options.show_help) {
        print_usage(argv[0]);
        return 0;
    }

    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    serial_package::SerialPortConfig port_config;
    port_config.device = options.device;
    port_config.baud_rate = options.baud_rate;
    port_config.read_timeout_ms = options.read_timeout_ms;
    port_config.write_timeout_ms = options.write_timeout_ms;

    serial_package::SerialPort port;
    if (!port.open(port_config)) {
        std::cerr << "[串口调试] 打开串口失败: " << port.last_error() << '\n';
        return 1;
    }

    std::mutex output_mutex;
    std::atomic<bool> stop_requested{false};
    std::thread receiver(receive_loop, &port, std::cref(options),
                         &stop_requested, &output_mutex);

    print_line(&output_mutex,
               "[串口调试] 已打开 " + options.device +
                   "，等待接收 IMU 帧；按任意键开始发送");
    if (!wait_for_any_key(stop_requested, &output_mutex)) {
        stop_requested.store(true);
        receiver.join();
        return g_stop_requested == 0 ? 1 : 0;
    }

    serial_package::ControlFrameValues values{1.0, 1.0, 1.0};
    std::array<std::uint8_t, serial_package::kControlFrameSize> frame{};
    std::string encode_error;
    if (!serial_package::encode_control_frame(values, &frame, &encode_error)) {
        print_line(&output_mutex,
                   "[串口调试] 固定控制帧编码失败: " + encode_error);
        stop_requested.store(true);
        receiver.join();
        return 1;
    }

    std::ostringstream start_message;
    start_message << "[串口调试] 开始发送：三个控制数据均为 1.0，间隔 "
                  << options.send_interval_ms << " ms";
    print_line(&output_mutex, start_message.str());

    std::thread sender(send_loop, &port, std::cref(options), std::cref(frame),
                       &stop_requested, &output_mutex);
    while (!stopping(stop_requested)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    stop_requested.store(true);
    sender.join();
    receiver.join();
    return 0;
}
