#include <mosas/output/imu_csv_recorder.hpp>

#include "recording_paths.hpp"

#include <chrono>
#include <exception>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <locale>
#include <sstream>
#include <system_error>
#include <utility>

namespace mosas::output {
namespace {

constexpr char kImuCsvHeader[] =
    "sample_index,timestamp_ns,acceleration_x_mps2,acceleration_y_mps2,"
    "acceleration_z_mps2,angular_velocity_x_rad_s,"
    "angular_velocity_y_rad_s,angular_velocity_z_rad_s,"
    "servo_control_p_normalized,servo_control_y_normalized\n";

std::filesystem::path make_imu_csv_path(const std::string& base_path) {
    std::filesystem::path configured_path(base_path);
    if (configured_path.empty()) {
        configured_path = "recordings/mosas_guidance.avi";
    }

    std::string stem = configured_path.stem().string();
    if (stem.empty()) {
        stem = "mosas_guidance";
    }
    return configured_path.parent_path() /
           (stem + "_imu_" + detail::make_session_id() + ".csv");
}

}  // namespace

ImuCsvRecorder::ImuCsvRecorder(std::string recording_base_path)
    : recording_base_path_(std::move(recording_base_path)),
      sample_queue_(kQueueCapacity) {}

ImuCsvRecorder::~ImuCsvRecorder() { stop(); }

bool ImuCsvRecorder::start() {
    stop();
    sample_queue_.reset();
    failed_.store(false, std::memory_order_release);
    stop_requested_.store(false, std::memory_order_release);
    {
        std::lock_guard<std::mutex> lock(error_mutex_);
        last_error_.clear();
    }

    try {
        const std::filesystem::path output_path =
            make_imu_csv_path(recording_base_path_);
        if (!output_path.parent_path().empty()) {
            std::error_code directory_error;
            std::filesystem::create_directories(output_path.parent_path(),
                                                directory_error);
            if (directory_error) {
                set_error("创建 IMU CSV 目录失败: " +
                          output_path.parent_path().string() + " (" +
                          directory_error.message() + ")");
                return false;
            }
        }

        imu_csv_path_ = output_path.string();
        writer_.open(imu_csv_path_, std::ios::out | std::ios::trunc);
        if (!writer_.is_open()) {
            set_error("打开 IMU CSV 失败: " + imu_csv_path_);
            return false;
        }
        writer_.imbue(std::locale::classic());
        writer_ << kImuCsvHeader;
        writer_.flush();
        if (!writer_) {
            writer_.close();
            set_error("写入 IMU CSV 表头失败: " + imu_csv_path_);
            return false;
        }

        try {
            worker_ = std::thread(&ImuCsvRecorder::worker_loop, this);
        } catch (const std::exception&) {
            writer_.close();
            set_error("启动 IMU CSV 写入线程失败");
            return false;
        } catch (...) {
            writer_.close();
            set_error("启动 IMU CSV 写入线程失败");
            return false;
        }

        accepting_samples_.store(true, std::memory_order_release);
        std::clog << "IMU 全量 CSV 记录: " << imu_csv_path_ << '\n';
        return true;
    } catch (const std::exception&) {
        set_error("创建 IMU CSV 失败");
    } catch (...) {
        set_error("创建 IMU CSV 失败");
    }
    return false;
}

bool ImuCsvRecorder::try_record(
    const runtime::ImuAngularVelocitySample& sample) noexcept {
    if (!accepting_samples_.load(std::memory_order_acquire) ||
        failed_.load(std::memory_order_acquire)) {
        return false;
    }
    if (!sample_queue_.try_push(sample)) {
        set_error("IMU CSV 写入队列已满");
        return false;
    }
    return true;
}

void ImuCsvRecorder::stop() noexcept {
    accepting_samples_.store(false, std::memory_order_release);
    stop_requested_.store(true, std::memory_order_release);
    if (worker_.joinable()) {
        worker_.join();
    }
    if (writer_.is_open()) {
        writer_.flush();
        if (!writer_) {
            set_error("刷新 IMU CSV 失败");
        }
        writer_.close();
        if (writer_.fail()) {
            set_error("关闭 IMU CSV 失败");
        }
    }
}

std::string ImuCsvRecorder::last_error() const {
    std::lock_guard<std::mutex> lock(error_mutex_);
    return last_error_;
}

void ImuCsvRecorder::set_error(const char* message) noexcept {
    failed_.store(true, std::memory_order_release);
    accepting_samples_.store(false, std::memory_order_release);
    try {
        std::lock_guard<std::mutex> lock(error_mutex_);
        if (last_error_.empty()) {
            last_error_ = message;
        }
    } catch (...) {
    }
}

void ImuCsvRecorder::set_error(const std::string& message) noexcept {
    failed_.store(true, std::memory_order_release);
    accepting_samples_.store(false, std::memory_order_release);
    try {
        std::lock_guard<std::mutex> lock(error_mutex_);
        if (last_error_.empty()) {
            last_error_ = message;
        }
    } catch (...) {
    }
}

void ImuCsvRecorder::worker_loop() noexcept {
    std::string batch;
    std::size_t batch_sample_count = 0;
    std::uint64_t sample_index = 0;
    auto next_flush_at = std::chrono::steady_clock::now() +
                         std::chrono::seconds(1);

    const auto flush_batch = [this, &batch, &batch_sample_count,
                              &next_flush_at]() {
        if (!batch.empty()) {
            writer_.write(batch.data(),
                          static_cast<std::streamsize>(batch.size()));
            writer_.flush();
            if (!writer_) {
                set_error("写入 IMU CSV 失败");
                return false;
            }
            batch.clear();
            batch_sample_count = 0;
        }
        next_flush_at =
            std::chrono::steady_clock::now() + std::chrono::seconds(1);
        return true;
    };

    try {
        batch.reserve(64 * 1024);
        while (!stop_requested_.load(std::memory_order_acquire) ||
               !sample_queue_.empty()) {
            runtime::ImuAngularVelocitySample sample{};
            if (sample_queue_.try_pop(&sample)) {
                std::ostringstream row;
                row.imbue(std::locale::classic());
                row << sample_index++ << ',' << sample.timestamp_ns << ','
                    << std::setprecision(
                           std::numeric_limits<double>::max_digits10)
                    << sample.acceleration.x << ',' << sample.acceleration.y
                    << ',' << sample.acceleration.z << ','
                    << sample.angular_velocity.x << ','
                    << sample.angular_velocity.y << ','
                    << sample.angular_velocity.z << ','
                    << sample.servo_control_p_normalized << ','
                    << sample.servo_control_y_normalized << '\n';
                batch += row.str();
                ++batch_sample_count;

                if (batch_sample_count >= kBatchSampleCount &&
                    !flush_batch()) {
                    return;
                }
                continue;
            }

            if (batch_sample_count != 0 &&
                std::chrono::steady_clock::now() >= next_flush_at &&
                !flush_batch()) {
                return;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        if (!flush_batch()) {
            return;
        }
    } catch (const std::exception&) {
        set_error("IMU CSV 写入线程异常");
    } catch (...) {
        set_error("IMU CSV 写入线程异常");
    }
}

}  // namespace mosas::output
