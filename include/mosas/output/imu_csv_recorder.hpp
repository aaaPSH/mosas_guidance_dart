#ifndef MOSAS_OUTPUT_IMU_CSV_RECORDER_HPP
#define MOSAS_OUTPUT_IMU_CSV_RECORDER_HPP

#include <mosas/runtime/output_ports.hpp>
#include <mosas/runtime/spsc_ring.hpp>

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>

namespace mosas::output {

/// 在专属后台线程中批量写入 IMU 运动样本的 CSV sink。
class ImuCsvRecorder final : public runtime::IImuRecorder {
public:
    static constexpr std::size_t kQueueCapacity = 16'384;

    /**
     * @brief 创建带有界样本队列的 IMU CSV 记录器。
     * @param recording_base_path 本次运行的录像基础路径，用于派生 IMU CSV 文件名。
     * @param queue_capacity 样本队列容量；满队列时 try_record 拒绝新样本并计数。
     */
    explicit ImuCsvRecorder(
        std::string recording_base_path,
        std::size_t queue_capacity = kQueueCapacity);
    ~ImuCsvRecorder();

    ImuCsvRecorder(const ImuCsvRecorder&) = delete;
    ImuCsvRecorder& operator=(const ImuCsvRecorder&) = delete;

    bool start() override;
    bool try_record(
        const domain::ImuKinematicSample& sample) noexcept override;
    runtime::SinkDiagnostics diagnostics() const noexcept override;
    void cancel() noexcept override;
    void stop() noexcept override;

private:
    void worker_loop() noexcept;
    void set_error(const char* message) noexcept;
    void set_error(const std::string& message) noexcept;

    static constexpr std::size_t kBatchSampleCount = 256;

    std::string recording_base_path_;
    std::string imu_csv_path_;
    runtime::SpscRing<domain::ImuKinematicSample> sample_queue_;
    std::ofstream writer_;
    std::thread worker_;
    std::atomic<bool> accepting_samples_{false};
    std::atomic<bool> stop_requested_{false};
    std::atomic<bool> failed_{false};
    std::atomic<std::uint64_t> accepted_count_{0};
    std::atomic<std::uint64_t> dropped_count_{0};
    std::atomic<std::uint64_t> failure_count_{0};
    mutable std::mutex error_mutex_;
    std::string last_error_;
};

}  // namespace mosas::output

#endif  // MOSAS_OUTPUT_IMU_CSV_RECORDER_HPP
