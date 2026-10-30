#include <mosas/adapters/serial_imu_source.hpp>
#include "imu_frame_fixture.hpp"
#include "test_support.hpp"
#include <chrono>
#include <cstdlib>
#include <fcntl.h>
#include <thread>
#include <unistd.h>

int main() {
    using namespace mosas;
    using namespace std::chrono_literals;
    const int master = ::posix_openpt(O_RDWR | O_NOCTTY | O_CLOEXEC);
    CHECK(master >= 0 && ::grantpt(master) == 0 && ::unlockpt(master) == 0);
    auto port = std::make_shared<serial_package::SerialPort>();
    serial_package::SerialPortConfig serial;
    serial.device = ::ptsname(master);
    CHECK(port->open(serial));
    auto frame = test::kValidImuFrame;
    CHECK(::write(master, frame.data(), frame.size()) == static_cast<ssize_t>(frame.size()));
    // Old kernel bytes must be flushed, not used to establish the first-frame baseline.
    app::SerialImuSource source(port, false, 1'000'000, {256, 0, false},
                               {200, 30'000'000, 20'000'000, 50'000'000, 1'000'000'000});
    auto sync = std::make_shared<domain::AcquisitionSync>(2);
    source.bind_acquisition_sync(sync);
    CHECK(source.start().status == runtime::SourceStatus::ok);
    std::this_thread::sleep_for(10ms);
    runtime::ImuAngularVelocitySample sample;
    CHECK(!source.try_pop(&sample));
    CHECK(source.diagnostics().received_chunk_count == 0);
    bool have_sample = false;
    std::int64_t previous_time = 0;
    std::uint64_t previous_sequence = 0;
    unsigned delivered = 0;
    auto deadline = std::chrono::steady_clock::now();
    for (unsigned i = 0; i < 40; ++i) {
        deadline += 5ms;
        const auto& bytes = test::kValidImuFrame;
        CHECK(::write(master, bytes.data(), bytes.size()) == static_cast<ssize_t>(bytes.size()));
        std::this_thread::sleep_until(deadline);
        while (source.try_pop(&sample)) {
            if (have_sample) CHECK(sample.timestamp_ns - previous_time ==
                static_cast<std::int64_t>(sample.sample_sequence - previous_sequence) * 5'000'000);
            else CHECK(sync->latch_launch(sample.acquisition_epoch));
            have_sample = true; ++delivered;
            previous_time = sample.timestamp_ns; previous_sequence = sample.sample_sequence;
        }
    }
    source.stop();
    CHECK(have_sample && delivered > 10);
    CHECK(source.diagnostics().timing.baselines == 1);
    CHECK(source.diagnostics().timing.warmup_discarded > 0);
    port->close(); ::close(master);
    return 0;
}
