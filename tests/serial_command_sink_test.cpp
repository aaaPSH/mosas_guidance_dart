#include <mosas/adapters/host_adapters.hpp>
#include <serial_package/serial_protocol.hpp>

#include "test_support.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <fcntl.h>
#include <memory>
#include <poll.h>
#include <sstream>
#include <thread>
#include <unistd.h>
#include <vector>

namespace {

using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;
using Frame = std::array<std::uint8_t, serial_package::kControlFrameSize>;

/** @brief Exercise the real serial writer using a hardware-free Linux PTY. */
class SerialFixture {
public:
    ~SerialFixture() {
        port->close();
        if (master_ >= 0) {
            ::close(master_);
        }
    }

    /** @return True when the PTY slave has been opened as a serial port. */
    bool open() {
        master_ = ::posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
        if (master_ < 0 || ::grantpt(master_) != 0 || ::unlockpt(master_) != 0) {
            return false;
        }
        const char* slave = ::ptsname(master_);
        if (slave == nullptr) {
            return false;
        }
        serial_package::SerialPortConfig config;
        config.device = slave;
        return port->open(config);
    }

    /**
     * @brief Read one control frame, preserving partial reads across timeouts.
     * @param frame Receives the next complete frame.
     * @param timeout Maximum duration to wait for a complete frame.
     * @return True when a complete frame was read.
     */
    bool read_frame(Frame& frame, Clock::duration timeout = 500ms) {
        const auto deadline = Clock::now() + timeout;
        while (Clock::now() < deadline) {
            pollfd descriptor{master_, POLLIN, 0};
            const auto remaining = std::chrono::duration_cast<
                std::chrono::milliseconds>(deadline - Clock::now());
            const int ready = ::poll(&descriptor, 1,
                                     std::max(1, static_cast<int>(remaining.count())));
            if (ready < 0 && errno == EINTR) {
                continue;
            }
            if (ready <= 0 || (descriptor.revents & POLLIN) == 0) {
                return false;
            }
            const auto count = ::read(master_, partial_.data() + filled_,
                                      partial_.size() - filled_);
            if (count < 0 && (errno == EINTR || errno == EAGAIN)) {
                continue;
            }
            if (count <= 0) {
                return false;
            }
            filled_ += static_cast<std::size_t>(count);
            if (filled_ == partial_.size()) {
                frame = partial_;
                filled_ = 0;
                return true;
            }
        }
        return false;
    }

    /** @return Buffered frames, stopping after a 2 ms quiet period. */
    std::vector<Frame> drain() {
        std::vector<Frame> frames;
        Frame frame{};
        while (read_frame(frame, 2ms)) {
            frames.push_back(frame);
        }
        return frames;
    }

    /** @return True if the expected frame arrives within the timeout. */
    bool wait_for(const Frame& expected, Clock::duration timeout = 500ms) {
        const auto deadline = Clock::now() + timeout;
        Frame frame{};
        while (read_frame(frame, deadline - Clock::now())) {
            if (frame == expected) {
                return true;
            }
        }
        return false;
    }

    std::shared_ptr<serial_package::SerialPort> port =
        std::make_shared<serial_package::SerialPort>();

private:
    int master_ = -1;
    Frame partial_{};
    std::size_t filled_ = 0;
};

/** @return Protocol reference frame, including CRC, for the supplied values. */
Frame encoded(bool launched, bool target, double yaw = 0.0, double pitch = 0.0) {
    Frame frame{};
    serial_package::encode_control_frame({launched, target, yaw, pitch}, &frame);
    return frame;
}

/** @brief Verify startup, repeat-without-refresh, invalidation, and generation reset. */
int test_periodic_lifecycle() {
    SerialFixture serial;
    CHECK(serial.open());
    std::ostringstream errors;
    mosas::app::SerialCommandSink sink(serial.port, {}, errors);
    CHECK(sink.start());
    Frame frame{};
    const auto started = Clock::now();
    for (int i = 0; i < 3; ++i) {
        CHECK(serial.read_frame(frame));
        CHECK(frame == encoded(false, false));
    }
    CHECK(Clock::now() - started >= 10ms);

    mosas::runtime::GuidanceCommand command;
    command.valid = true;
    command.launched = true;
    command.target_found = true;
    command.overload_g = {0.0, 0.23, -0.11};
    const auto submitted_at = Clock::now();
    CHECK(sink.try_submit(command));
    CHECK(serial.wait_for(encoded(true, true, -0.11, 0.23)));
    for (int i = 0; i < 2; ++i) {
        CHECK(serial.read_frame(frame));
        CHECK(frame == encoded(true, true, -0.11, 0.23));
    }
    // Periodic repeats must not renew freshness; launch remains latched on expiry.
    CHECK(serial.wait_for(encoded(true, false), 250ms));
    CHECK(Clock::now() - submitted_at >= 80ms);

    command.valid = false;
    CHECK(sink.try_submit(command));
    CHECK(serial.wait_for(encoded(true, true)));
    command.valid = true;
    CHECK(sink.try_submit(command));
    CHECK(serial.wait_for(encoded(true, true, -0.11, 0.23)));
    command.target_found = false;
    command.valid = false;
    CHECK(sink.try_submit(command));
    CHECK(serial.wait_for(encoded(true, false), 80ms));

    command.target_found = true;
    command.valid = true;
    CHECK(sink.try_submit(command));
    CHECK(serial.wait_for(encoded(true, true, -0.11, 0.23)));
    sink.set_active_generation(1);
    serial.drain();
    CHECK(serial.read_frame(frame));
    CHECK(frame == encoded(false, false));
    CHECK(!sink.try_submit(command));

    // Even an erroneously valid pre-launch command must not produce nonzero output.
    command.generation = 1;
    command.launched = false;
    CHECK(sink.try_submit(command));
    CHECK(serial.wait_for(encoded(false, true)));
    command.launched = true;
    CHECK(sink.try_submit(command));
    CHECK(serial.wait_for(encoded(true, true, -0.11, 0.23)));
    sink.cancel();
    CHECK(!sink.try_submit(command));
    sink.stop();
    serial.drain();
    CHECK(!serial.read_frame(frame, 30ms));

    CHECK(sink.start());
    CHECK(serial.read_frame(frame));
    CHECK(frame == encoded(false, false));
    sink.stop();
    CHECK(sink.diagnostics().send_failure_count == 0);
    CHECK(errors.str().empty());
    return 0;
}

/** @brief Verify a 120 Hz producer is sampled at 100 Hz without a FIFO backlog. */
int test_120_hz_to_100_hz() {
    SerialFixture serial;
    CHECK(serial.open());
    std::ostringstream errors;
    mosas::app::SerialCommandSink sink(serial.port, {}, errors);
    CHECK(sink.start());
    mosas::runtime::GuidanceCommand command;
    command.valid = command.launched = command.target_found = true;
    bool all_submitted = true;
    const auto started = Clock::now();
    for (int i = 1; i <= 36; ++i) {
        command.overload_g = {0.0, i * 0.01, 0.0};
        all_submitted = sink.try_submit(command) && all_submitted;
        std::this_thread::sleep_until(started + std::chrono::nanoseconds(
            static_cast<std::int64_t>(i) * 1'000'000'000 / 120));
    }
    sink.stop();
    const auto frames = serial.drain();
    CHECK(all_submitted);
    // Allow host scheduling jitter, but not one transmission per 120 Hz submission.
    CHECK(frames.size() >= 20);
    CHECK(frames.size() <= 34);
    bool skipped_intermediate = false;
    int previous = 0;
    for (const auto& frame : frames) {
        if (frame == encoded(false, false)) {
            continue;
        }
        const int sequence = frame[6] | (static_cast<int>(frame[7]) << 8);
        CHECK(sequence >= previous);
        CHECK(sequence >= 1 && sequence <= 36);
        CHECK(frame == encoded(true, true, 0.0, sequence * 0.01));
        skipped_intermediate = skipped_intermediate || sequence > previous + 1;
        previous = sequence;
    }
    CHECK(skipped_intermediate);
    CHECK(previous >= 33);
    CHECK(sink.diagnostics().send_failure_count == 0);
    return 0;
}

/** @brief Verify configured cadence, publish-time expiry, and interruptible shutdown. */
int test_configured_cadence() {
    SerialFixture serial;
    CHECK(serial.open());
    std::ostringstream errors;
    mosas::app::CommandAppConfig config;
    config.send_rate_hz = 10;
    config.command_timeout_ms = 1;
    mosas::app::SerialCommandSink sink(serial.port, config, errors);
    CHECK(sink.start());
    Frame frame{};
    CHECK(serial.read_frame(frame));
    CHECK(frame == encoded(false, false));
    mosas::runtime::GuidanceCommand command;
    command.valid = command.launched = command.target_found = true;
    command.overload_g = {0.0, 0.2, 0.1};
    const auto submitted_at = Clock::now();
    CHECK(sink.try_submit(command));
    CHECK(serial.read_frame(frame));
    // The mailbox value is already stale on its first consumption at the next tick.
    CHECK(frame == encoded(true, false));
    CHECK(Clock::now() - submitted_at >= 60ms);
    sink.stop();
    serial.drain();

    config.send_rate_hz = 1;
    mosas::app::SerialCommandSink slow_sink(serial.port, config, errors);
    CHECK(slow_sink.start());
    CHECK(serial.read_frame(frame));
    const auto stopped_at = Clock::now();
    slow_sink.stop();
    CHECK(Clock::now() - stopped_at < 200ms);
    CHECK(errors.str().empty());

    config.send_rate_hz = 0;
    mosas::app::SerialCommandSink invalid_sink(serial.port, config, errors);
    CHECK(!invalid_sink.start());
    CHECK(!invalid_sink.last_error().empty());
    return 0;
}

}  // namespace

int main() {
    if (test_periodic_lifecycle() != 0 || test_120_hz_to_100_hz() != 0) {
        return 1;
    }
    return test_configured_cadence();
}
