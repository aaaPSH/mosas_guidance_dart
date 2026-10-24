#include <mosas/app/camera_imu_sync_session.hpp>
#include <mosas/vision/opencv_image_frame.hpp>

#include "test_support.hpp"
#include "camera_imu_sync_queue.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <string>

#if defined(__linux__)
#include <sys/resource.h>
#include <sys/wait.h>
#include <unistd.h>
#endif

namespace {
namespace fs = std::filesystem;

/** @brief 即使断言提前返回，也清理本次测试产生的全部采集文件。 */
struct TemporaryDirectory {
    fs::path path = fs::temp_directory_path() /
        ("mosas-sync-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    TemporaryDirectory() { fs::create_directory(path); }
    ~TemporaryDirectory() { std::error_code error; fs::remove_all(path, error); }
};

std::string text(const fs::path& path) {
    std::ifstream input(path, std::ios::binary);
    return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}

mosas::app::AppConfig configuration() {
    mosas::app::AppConfig config;
    config.camera.width = 3;
    config.camera.height = 2;
    config.camera.fps = 120;
    config.camera.auto_exposure = false;
    config.camera.frame_timestamp_domain = mosas::runtime::CameraTimestampDomain::monotonic;
    config.camera.camera_timestamp_bias_ns = -2'000'000;
    config.imu.mode = mosas::app::ImuMode::serial;
    config.imu.measurement_latency_ns = 7'000'000;
    return config;
}

mosas::runtime::CameraFrame camera_frame(std::int64_t timestamp) {
    mosas::runtime::CameraFrame frame;
    frame.timestamp_ns = timestamp;
    frame.received_timestamp_ns = timestamp + 3'000'000;
    frame.timestamp_source = mosas::runtime::CameraTimestampSource::sdk_monotonic;
    // 使用非连续 ROI，同时验证行末填充字节不会被写入采集文件。
    cv::Mat backing(2, 5, CV_8UC3, cv::Scalar(90, 90, 90));
    cv::Mat roi = backing(cv::Rect(0, 0, 3, 2));
    roi.row(0).setTo(cv::Scalar(10, 10, 10));
    roi.row(1).setTo(cv::Scalar(200, 200, 200));
    mosas::vision::assign_bgr8_mat(&frame.image, roi);
    return frame;
}

mosas::runtime::ImuAngularVelocitySample imu_sample(std::int64_t timestamp) {
    mosas::runtime::ImuAngularVelocitySample sample;
    sample.timestamp_ns = timestamp;
    sample.acceleration = {0, 9.8, 0};
    sample.angular_velocity = {0.003, -0.004, 0.002};
    return sample;
}

/** @brief 验证关闭文件后的原始像素、可逆时间补偿及正常完成标记。 @return 通过时返回 0。 */
int capture_test() {
    using namespace mosas;
    TemporaryDirectory temporary;
    app::CameraImuSyncOptions options;
    options.directory = temporary.path / "capture";
    options.seconds = 10;
    options.fps = 60;
    app::CameraImuSyncSession session(options, configuration());
    session.start();
    CHECK(text(options.directory / "metadata.json").find("\"complete\": false") != std::string::npos);
    for (int index = 0; index < 2; ++index) {
        auto frame = camera_frame(1'000'000'000 + index * 20'000'000);
        CHECK(session.record_camera(frame, 1'010'000'000 + index * 20'000'000));
        auto view = vision::make_bgr8_mat_view(frame.image);
        cv::flip(view, view, -1);  // 正常 prepare() 可能立即原地修改共享像素。
        CHECK(session.record_imu(imu_sample(1'000'000'000 + index * 5'000'000),
                                 1'010'000'000 + index * 5'000'000));
    }
    app::CameraImuSyncSummary summary;
    summary.imu.available = true;
    summary.imu.delivered_sample_count = 2;
    CHECK(session.finish(app::CameraImuSyncEnd::duration_elapsed, summary));
    CHECK(session.finish(app::CameraImuSyncEnd::interrupted, {}));  // 重复完成会话时保留首次结果。
    const auto pixels = text(options.directory / "frames.gray8");
    CHECK(pixels.size() == 12);
    CHECK(static_cast<unsigned char>(pixels[0]) == 10);
    CHECK(static_cast<unsigned char>(pixels[3]) == 200);
    const auto frames = text(options.directory / "frames.csv");
    CHECK(frames.find("1000000000,1002000000") != std::string::npos);
    const auto imu = text(options.directory / "imu.csv");
    CHECK(imu.find("1000000000,1007000000") != std::string::npos);
    const auto metadata = text(options.directory / "metadata.json");
    CHECK(metadata.find("\"complete\": true") != std::string::npos);
    CHECK(metadata.find("runtime_prepared_180") != std::string::npos);
    bool refused = false;
    try { app::validate_camera_imu_sync_options(options, configuration()); }
    catch (const std::invalid_argument&) { refused = true; }
    CHECK(refused);
    return 0;
}

/** @brief 验证中断、无效时间戳及实际 SDK 尺寸超限均不能生成完整清单。 @return 通过时返回 0。 */
int rejection_test() {
    using namespace mosas;
    TemporaryDirectory temporary;
    for (int scenario = 0; scenario < 5; ++scenario) {
        app::CameraImuSyncOptions options;
        options.directory = temporary.path / std::to_string(scenario);
        options.seconds = 10;
        options.max_bytes = 2 * 1024 * 1024;
        auto config = configuration();
        if (scenario == 4) config.imu.measurement_latency_ns = std::numeric_limits<std::int64_t>::max();
        app::CameraImuSyncSession session(options, config);
        session.start();
        if (scenario == 1) {
            CHECK(session.record_imu(imu_sample(100), 200));
            CHECK(!session.record_imu(imu_sample(100), 300));
            CHECK(session.error() == app::CameraImuSyncError::invalid_imu);
        } else if (scenario == 2) {
            auto frame = camera_frame(1'000'000'000);
            cv::Mat large(2048, 2048, CV_8UC3);
            vision::assign_bgr8_mat(&frame.image, large);
            CHECK(!session.record_camera(frame, 1'010'000'000));
            CHECK(session.error() == app::CameraImuSyncError::budget_exceeded);
        } else if (scenario == 3) {
            auto sample = imu_sample(100);
            sample.angular_velocity.x = std::numeric_limits<double>::quiet_NaN();
            CHECK(!session.record_imu(sample, 200));
        } else if (scenario == 4) {
            CHECK(!session.record_imu(imu_sample(100), 200));
        }
        CHECK(!session.finish(scenario == 0 ? app::CameraImuSyncEnd::interrupted
                                          : app::CameraImuSyncEnd::duration_elapsed, {}));
        CHECK(text(options.directory / "metadata.json").find("\"complete\": false") != std::string::npos);
    }
    return 0;
}
/** @brief 即使已接收记录全部写出，真实数据源丢失仍必须使采集无效。 @return 通过时返回 0。 */
int source_drop_rejection_test() {
    using namespace mosas;
    TemporaryDirectory temporary;
    app::CameraImuSyncOptions options;
    options.directory = temporary.path / "source-drop";
    app::CameraImuSyncSession session(options, configuration());
    session.start();
    for (int index = 0; index < 2; ++index) {
        const auto timestamp = 1'000'000'000 + index * 100'000'000;
        CHECK(session.record_camera(camera_frame(timestamp), timestamp + 10'000'000));
        CHECK(session.record_imu(imu_sample(timestamp), timestamp + 10'000'000));
    }
    CHECK(session.error() == app::CameraImuSyncError::none);
    app::CameraImuSyncSummary summary;
    summary.imu.available = true;
    summary.imu.delivered_sample_count = 2;
    summary.imu.dropped_chunk_count = 1;
    CHECK(!session.finish(app::CameraImuSyncEnd::duration_elapsed, summary));
    CHECK(session.error() == app::CameraImuSyncError::invalid_imu);
    const auto metadata = text(options.directory / "metadata.json");
    CHECK(metadata.find("\"complete\": false") != std::string::npos);
    CHECK(metadata.find("\"imu_written\":2") != std::string::npos);
    CHECK(metadata.find("\"dropped_chunk_count\":1") != std::string::npos);
    return 0;
}

/** @brief 验证即使单帧可容纳，累计数据仍不能突破存储预算。 @return 通过时返回 0。 */
int cumulative_budget_test() {
    using namespace mosas;
    TemporaryDirectory temporary;
    app::CameraImuSyncOptions options;
    options.directory = temporary.path / "budget";
    options.seconds = 10;
    options.fps = 60;
    options.max_bytes = 2 * 1024 * 1024;
    app::CameraImuSyncSession session(options, configuration());
    session.start();
    for (int index = 0; index < 3; ++index) {
        auto frame = camera_frame(1'000'000'000 + index * 20'000'000);
        cv::Mat image(768, 1024, CV_8UC3, cv::Scalar(25, 25, 25));
        vision::assign_bgr8_mat(&frame.image, image);
        CHECK(session.record_camera(frame, 1'010'000'000 + index * 20'000'000));
    }
    CHECK(!session.finish(app::CameraImuSyncEnd::duration_elapsed, {}));
    CHECK(session.error() == app::CameraImuSyncError::budget_exceeded);
    CHECK(fs::file_size(options.directory / "frames.gray8") <= options.max_bytes);
    return 0;
}

/** @brief 不启动磁盘消费者，确定性验证记录队列溢出后的错误锁存。 @return 通过时返回 0。 */
int overflow_test() {
    using namespace mosas::app;
    for (auto kind : {CameraImuSyncError::image_queue_full, CameraImuSyncError::imu_queue_full}) {
        std::atomic<CameraImuSyncError> error{CameraImuSyncError::none};
        detail::SyncRecordQueue<int> queue(1, error, kind);
        CHECK(queue.try_push(1));
        CHECK(queue.full());
        CHECK(!queue.try_push(2));
        CHECK(error.load() == kind);
        CHECK(queue.dropped() == 1 && queue.high_water() == 1);
        int value = 0;
        CHECK(queue.try_pop(&value) && value == 1);
        CHECK(queue.empty());
        // 排空队列不能抹去已经发生的数据丢失，也不能使采集重新有效。
        CHECK(error.load() == kind);
    }
    return 0;
}

#if defined(__linux__)
/** @brief 在隔离子进程中强制触发真实写失败，不填满磁盘。 @param directory 新采集目录。 @return 通过时返回 0。 */
int limited_writer(const fs::path& directory) {
    using namespace mosas;
    app::CameraImuSyncOptions options;
    options.directory = directory;
    app::CameraImuSyncSession session(options, configuration());
    session.start();
    struct rlimit limit {};
    CHECK(getrlimit(RLIMIT_FSIZE, &limit) == 0);
    limit.rlim_cur = std::min<rlim_t>(limit.rlim_max, 8192);
    std::signal(SIGXFSZ, SIG_IGN);
    CHECK(setrlimit(RLIMIT_FSIZE, &limit) == 0);
    auto frame = camera_frame(1'000'000'000);
    cv::Mat image(512, 512, CV_8UC3, cv::Scalar(25, 25, 25));
    vision::assign_bgr8_mat(&frame.image, image);
    CHECK(session.record_camera(frame, 1'010'000'000));
    CHECK(!session.finish(app::CameraImuSyncEnd::duration_elapsed, {}));
    CHECK(session.error() == app::CameraImuSyncError::write_failure);
    CHECK(text(directory / "metadata.json").find("\"complete\": false") != std::string::npos);
    return 0;
}

int writer_failure_test() {
    TemporaryDirectory temporary;
    const auto child = fork();
    CHECK(child >= 0);
    if (child == 0) _exit(limited_writer(temporary.path / "limited"));
    int status = 0;
    CHECK(waitpid(child, &status, 0) == child);
    CHECK(WIFEXITED(status) && WEXITSTATUS(status) == 0);
    return 0;
}
#endif
}  // namespace

/** @brief 运行无硬件依赖的标定记录回归测试。 @return 通过时返回 0。 */
int main() {
    cv::setNumThreads(1);
    if (capture_test() || rejection_test() || source_drop_rejection_test() ||
        cumulative_budget_test() || overflow_test()) return 1;
#if defined(__linux__)
    if (writer_failure_test()) return 1;
#endif
    return 0;
}
