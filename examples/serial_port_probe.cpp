#include <serial_package/serial_port.hpp>

#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

namespace {

bool parse_baud_rate(const char* text, uint32_t* baud_rate) {
    if (text == nullptr || baud_rate == nullptr) {
        return false;
    }
    char* end = nullptr;
    const unsigned long value = std::strtoul(text, &end, 10);
    if (*text == '\0' || *end != '\0' || value > UINT32_MAX) {
        return false;
    }
    *baud_rate = static_cast<uint32_t>(value);
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 4) {
        std::cerr << "usage: serial_port_probe <device> <baud_rate> <message>\n";
        return 2;
    }

    uint32_t baud_rate = 0;
    if (!parse_baud_rate(argv[2], &baud_rate)) {
        std::cerr << "invalid baud rate\n";
        return 2;
    }

    serial_package::SerialPortConfig config;
    config.device = argv[1];
    config.baud_rate = baud_rate;

    serial_package::SerialPort port;
    if (!port.open(config)) {
        std::cerr << "open failed: " << port.last_error() << '\n';
        return 1;
    }

    const std::string message = argv[3];
    if (!port.write_all(message.data(), message.size(), 1000)) {
        std::cerr << "write failed: " << port.last_error() << '\n';
        return 1;
    }

    std::vector<uint8_t> response;
    uint8_t buffer[256]{};
    while (true) {
        const ssize_t count = port.read(buffer, sizeof(buffer), 500);
        if (count == 0) {
            break;
        }
        if (count < 0) {
            std::cerr << "read failed: " << port.last_error() << '\n';
            return 1;
        }
        response.insert(response.end(), buffer, buffer + count);
    }

    std::cout << "received " << response.size() << " bytes:";
    for (const uint8_t byte : response) {
        std::cout << ' ' << std::hex << std::setw(2) << std::setfill('0')
                  << static_cast<unsigned int>(byte);
    }
    std::cout << std::dec << '\n';
    return 0;
}
