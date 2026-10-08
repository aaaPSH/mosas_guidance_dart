#include <mosas/output/imu_csv_recorder.hpp>
#include <mosas/runtime/async_info_logger.hpp>

#include "test_support.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <sstream>
#include <streambuf>
#include <string>
#include <thread>
#include <vector>

namespace {

class TemporaryDirectory {
public:
    TemporaryDirectory() {
        path_ = std::filesystem::temp_directory_path() /
                ("mosas-output-diagnostics-" +
                 std::to_string(std::chrono::steady_clock::now()
                                    .time_since_epoch()
                                    .count()));
        std::filesystem::create_directories(path_);
    }

    ~TemporaryDirectory() {
        std::error_code error;
        std::filesystem::remove_all(path_, error);
    }

    const std::filesystem::path& path() const noexcept { return path_; }

private:
    std::filesystem::path path_;
};

class BlockingStreamBuffer final : public std::streambuf {
public:
    bool wait_until_blocked(std::chrono::milliseconds timeout) {
        std::unique_lock<std::mutex> lock(mutex_);
        return condition_.wait_for(lock, timeout, [this] { return blocked_; });
    }

    void release() {
        std::lock_guard<std::mutex> lock(mutex_);
        released_ = true;
        condition_.notify_all();
    }

protected:
    std::streamsize xsputn(const char*, std::streamsize count) override {
        std::unique_lock<std::mutex> lock(mutex_);
        blocked_ = true;
        condition_.notify_all();
        condition_.wait(lock, [this] { return released_; });
        return count;
    }

    int_type overflow(int_type value) override {
        return traits_type::eq_int_type(value, traits_type::eof())
                   ? traits_type::not_eof(value)
                   : value;
    }
    int sync() override { return 0; }

private:
    std::mutex mutex_;
    std::condition_variable condition_;
    bool blocked_ = false;
    bool released_ = false;
};

class BlockingStreamRelease final {
public:
    explicit BlockingStreamRelease(BlockingStreamBuffer& stream_buffer)
        : stream_buffer_(stream_buffer) {}
    ~BlockingStreamRelease() { stream_buffer_.release(); }

private:
    BlockingStreamBuffer& stream_buffer_;
};

class FailingStreamBuffer final : public std::streambuf {
protected:
    std::streamsize xsputn(const char*, std::streamsize) override { return 0; }
    int_type overflow(int_type) override { return traits_type::eof(); }
    int sync() override { return -1; }
};

class FlushFailingStreamBuffer final : public std::streambuf {
protected:
    std::streamsize xsputn(const char*, std::streamsize count) override {
        return count;
    }
    int_type overflow(int_type value) override {
        return traits_type::eq_int_type(value, traits_type::eof())
                   ? traits_type::not_eof(value)
                   : value;
    }
    int sync() override { return -1; }
};

mosas::domain::ImuKinematicSample make_imu_sample(std::int64_t timestamp) {
    mosas::domain::ImuKinematicSample sample{};
    sample.time.monotonic_ns = timestamp;
    sample.acceleration = {1.0, 2.0, 3.0};
    sample.angular_velocity = {0.1, 0.2, 0.3};
    sample.servo_control_p_normalized = 0.25;
    sample.servo_control_y_normalized = -0.25;
    return sample;
}

std::vector<std::filesystem::path> find_csv_files(
    const std::filesystem::path& directory) {
    std::vector<std::filesystem::path> files;
    for (const auto& entry : std::filesystem::directory_iterator(directory)) {
        if (entry.path().extension() == ".csv") {
            files.push_back(entry.path());
        }
    }
    return files;
}

bool wait_for_unhealthy(mosas::runtime::AsyncInfoLogger& logger,
                        std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (!logger.diagnostics().healthy) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return !logger.diagnostics().healthy;
}

}  // namespace

int main() {
    using namespace std::chrono_literals;
    using namespace mosas;

    {
        TemporaryDirectory temporary_directory;
        output::ImuCsvRecorder recorder(
            (temporary_directory.path() / "flight.avi").string());
        CHECK(recorder.start());
        auto diagnostics = recorder.diagnostics();
        CHECK(diagnostics.enabled);
        CHECK(diagnostics.healthy);
        CHECK(diagnostics.accepted_count == 0);
        CHECK(diagnostics.dropped_count == 0);
        CHECK(diagnostics.failure_count == 0);
        CHECK(diagnostics.queue_depth == 0);
        CHECK(diagnostics.queue_capacity > 0);
        CHECK(diagnostics.last_error.empty());
        CHECK(recorder.try_record(make_imu_sample(100)));
        CHECK(recorder.diagnostics().accepted_count == 1);
        recorder.stop();
        diagnostics = recorder.diagnostics();
        CHECK(diagnostics.healthy);
        CHECK(diagnostics.accepted_count == 1);
        CHECK(diagnostics.queue_depth == 0);
        CHECK(diagnostics.last_error.empty());
        const auto files = find_csv_files(temporary_directory.path());
        CHECK(files.size() == 1);
        std::ifstream csv(files.front());
        std::string header;
        std::string row;
        CHECK(std::getline(csv, header));
        CHECK(std::getline(csv, row));
        CHECK(header.find("sample_index,timestamp_ns,") == 0);
        CHECK(row.find("0,100,") == 0);

        CHECK(recorder.start());
        diagnostics = recorder.diagnostics();
        CHECK(diagnostics.healthy);
        CHECK(diagnostics.accepted_count == 0);
        CHECK(diagnostics.dropped_count == 0);
        CHECK(diagnostics.failure_count == 0);
        CHECK(recorder.try_record(make_imu_sample(200)));
        recorder.stop();
        CHECK(recorder.diagnostics().accepted_count == 1);
        CHECK(recorder.diagnostics().healthy);
    }

    {
        TemporaryDirectory temporary_directory;
        output::ImuCsvRecorder recorder(
            (temporary_directory.path() / "queue_full.avi").string(), 1);
        CHECK(recorder.start());
        bool rejected = false;
        for (std::int64_t index = 0; index < 1'000'000 && !rejected;
             ++index) {
            rejected = !recorder.try_record(make_imu_sample(index));
        }
        CHECK(rejected);
        auto diagnostics = recorder.diagnostics();
        CHECK(!diagnostics.healthy);
        CHECK(diagnostics.accepted_count > 0);
        CHECK(diagnostics.dropped_count == 1);
        CHECK(diagnostics.failure_count == 1);
        CHECK(diagnostics.last_error.find("队列已满") != std::string::npos);
        recorder.stop();
        CHECK(recorder.diagnostics().queue_depth == 0);
    }

    {
        TemporaryDirectory temporary_directory;
        const auto blocker = temporary_directory.path() / "not_a_directory";
        {
            std::ofstream file(blocker);
            file << "file";
        }
        output::ImuCsvRecorder recorder((blocker / "flight.avi").string());
        CHECK(!recorder.start());
        const auto diagnostics = recorder.diagnostics();
        CHECK(diagnostics.enabled);
        CHECK(!diagnostics.healthy);
        CHECK(diagnostics.failure_count == 1);
        CHECK(!diagnostics.last_error.empty());
    }

    {
        BlockingStreamBuffer stream_buffer;
        std::ostream output_stream(&stream_buffer);
        runtime::AsyncInfoLogger logger(output_stream, 1);
        BlockingStreamRelease release_on_exit(stream_buffer);
        CHECK(logger.start());
        CHECK(logger.try_submit_text(1, "first"));
        CHECK(stream_buffer.wait_until_blocked(1s));
        CHECK(logger.try_submit_text(2, "second"));
        CHECK(!logger.try_submit_text(3, "dropped"));
        auto diagnostics = logger.diagnostics();
        CHECK(diagnostics.enabled);
        CHECK(diagnostics.healthy);
        CHECK(diagnostics.accepted_count == 2);
        CHECK(diagnostics.dropped_count == 1);
        CHECK(diagnostics.failure_count == 0);
        CHECK(diagnostics.queue_depth == 1);
        CHECK(diagnostics.queue_capacity == 1);
        stream_buffer.release();
        logger.stop_and_flush();
        diagnostics = logger.diagnostics();
        CHECK(diagnostics.healthy);
        CHECK(diagnostics.queue_depth == 0);
        CHECK(diagnostics.accepted_count == 2);
        CHECK(diagnostics.dropped_count == 1);
    }

    {
        FailingStreamBuffer stream_buffer;
        std::ostream output_stream(&stream_buffer);
        runtime::AsyncInfoLogger logger(output_stream, 4);
        CHECK(logger.start());
        CHECK(logger.try_submit_text(4, "write failure"));
        CHECK(wait_for_unhealthy(logger, 1s));
        logger.stop_and_flush();
        const auto diagnostics = logger.diagnostics();
        CHECK(!diagnostics.healthy);
        CHECK(diagnostics.accepted_count == 1);
        CHECK(diagnostics.failure_count == 1);
        CHECK(!diagnostics.last_error.empty());
    }

    {
        FlushFailingStreamBuffer stream_buffer;
        std::ostream output_stream(&stream_buffer);
        runtime::AsyncInfoLogger logger(output_stream, 4);
        CHECK(logger.start());
        CHECK(logger.try_submit_text(4, "flush failure"));
        logger.stop_and_flush();
        const auto diagnostics = logger.diagnostics();
        CHECK(!diagnostics.healthy);
        CHECK(diagnostics.accepted_count == 1);
        CHECK(diagnostics.failure_count == 1);
        CHECK(diagnostics.last_error.find("flush failed") !=
              std::string::npos);
    }

    {
        std::ostringstream output_stream;
        runtime::AsyncInfoLogger logger(output_stream, 4);
        CHECK(logger.start());
        CHECK(logger.try_submit_text(5, "first run"));
        logger.stop_and_flush();
        CHECK(logger.diagnostics().accepted_count == 1);
        CHECK(logger.start());
        auto diagnostics = logger.diagnostics();
        CHECK(diagnostics.healthy);
        CHECK(diagnostics.accepted_count == 0);
        CHECK(diagnostics.dropped_count == 0);
        CHECK(diagnostics.failure_count == 0);
        CHECK(diagnostics.queue_depth == 0);
        CHECK(diagnostics.last_error.empty());
        CHECK(logger.try_submit_text(6, "second run"));
        logger.stop_and_flush();
        CHECK(logger.diagnostics().accepted_count == 1);
    }

    {
        std::ostringstream output_stream;
        runtime::AsyncInfoLogger logger(output_stream, 512);
        CHECK(logger.start());
        constexpr std::size_t producer_count = 4;
        constexpr std::size_t events_per_producer = 100;
        std::atomic<std::size_t> rejected_count{0};
        std::vector<std::thread> producers;
        for (std::size_t producer = 0; producer < producer_count; ++producer) {
            producers.emplace_back([&, producer] {
                for (std::size_t event = 0; event < events_per_producer; ++event) {
                    const auto timestamp = static_cast<std::int64_t>(
                        producer * events_per_producer + event);
                    if (!logger.try_submit_text(timestamp, "concurrent event")) {
                        rejected_count.fetch_add(1, std::memory_order_relaxed);
                    }
                }
            });
        }
        for (std::thread& producer : producers) {
            producer.join();
        }
        CHECK(rejected_count.load(std::memory_order_relaxed) == 0);
        logger.stop_and_flush();
        const auto diagnostics = logger.diagnostics();
        CHECK(diagnostics.accepted_count ==
              producer_count * events_per_producer);
        CHECK(diagnostics.dropped_count == 0);
        CHECK(diagnostics.queue_depth == 0);
        const std::string logged_output = output_stream.str();
        CHECK(static_cast<std::size_t>(
                  std::count(logged_output.begin(), logged_output.end(), '\n')) ==
              producer_count * events_per_producer);
    }

    return 0;
}
