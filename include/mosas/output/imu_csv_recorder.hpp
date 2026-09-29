#ifndef MOSAS_OUTPUT_IMU_CSV_RECORDER_HPP
#define MOSAS_OUTPUT_IMU_CSV_RECORDER_HPP

#include <mosas/runtime/imu_angular_velocity.hpp>
#include <mosas/runtime/spsc_ring.hpp>

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>

namespace mosas::output {

// 在独立线程中批量写入每条 IMU 运动样本。
class ImuCsvRecorder final {
public:
    explicit ImuCsvRecorder(std::string recording_base_path);
    ~ImuCsvRecorder();

    ImuCsvRecorder(const ImuCsvRecorder&) = delete;
    ImuCsvRecorder& operator=(const ImuCsvRecorder&) = delete;

    bool start();
    bool try_record(const runtime::ImuAngularVelocitySample& sample) noexcept;
    void stop() noexcept;

    std::string last_error() const;

private:
    void worker_loop() noexcept;
    void set_error(const char* message) noexcept;
    void set_error(const std::string& message) noexcept;

    static constexpr std::size_t kQueueCapacity = 16'384;
    static constexpr std::size_t kBatchSampleCount = 256;

    std::string recording_base_path_;
    std::string imu_csv_path_;
    runtime::SpscRing<runtime::ImuAngularVelocitySample> sample_queue_;
    std::ofstream writer_;
    std::thread worker_;
    std::atomic<bool> accepting_samples_{false};
    std::atomic<bool> stop_requested_{false};
    std::atomic<bool> failed_{false};
    mutable std::mutex error_mutex_;
    std::string last_error_;
};

}  // namespace mosas::output

#endif  // MOSAS_OUTPUT_IMU_CSV_RECORDER_HPP
