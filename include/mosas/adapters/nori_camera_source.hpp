#ifndef MOSAS_ADAPTERS_NORI_CAMERA_SOURCE_HPP
#define MOSAS_ADAPTERS_NORI_CAMERA_SOURCE_HPP

#include <mosas/app/app_config.hpp>
#include <mosas/adapters/nori_camera_sdk.hpp>
#include <mosas/runtime/frame_timing.hpp>
#include <mosas/runtime/frame_rate_meter.hpp>
#include <mosas/runtime/timing_distribution.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <opencv2/core/mat.hpp>

namespace mosas::app {

struct NoriCameraTiming {
    runtime::TimingDistributionSummary callback_wait;
    runtime::TimingDistributionSummary callback_queue_delay;
    runtime::TimingDistributionSummary decode_convert;
    std::uint64_t callback_queue_dropped = 0;
    /** @brief Rejected timestamps; healthy warmup/synchronization waiting is counted separately. */
    std::uint64_t timestamp_dropped = 0;
};

struct CameraSdkConfiguration {
    uint32_t width = 0;
    uint32_t height = 0;
    float fps = 0.0F;
    uint32_t exposure_us = 0;
    uint32_t gain = 0;
};

class NoriSdkCameraSource final : public runtime::CameraSource {
public:
    /**
     * @brief 使用生产环境 Nori SDK 创建相机源。
     * @param config 设备索引、视频模式、队列和时间戳配置。
     */
    explicit NoriSdkCameraSource(CameraAppConfig config);

    /**
     * @brief 使用指定 SDK API 创建相机源。
     * @param config 相机应用配置。
     * @param sdk SDK API 实例；可注入 fake 实现以隔离硬件测试。
     */
    NoriSdkCameraSource(CameraAppConfig config, std::shared_ptr<NoriSdkApi> sdk);

    /** @brief 取消采集、释放 SDK 资源并解除回调 owner 关联。 */
    ~NoriSdkCameraSource() override;

    NoriSdkCameraSource(const NoriSdkCameraSource&) = delete;
    NoriSdkCameraSource& operator=(const NoriSdkCameraSource&) = delete;

    /**
     * @brief 初始化 Nori SDK 会话、匹配视频模式并启动相机回调。
     * @param config 自由运行或硬件触发采集模式。
     * @return 配置状态和错误文本；SDK/设备配置失败时返回 fatal。
     * @note 会访问相机 SDK 并建立 realtime 到 monotonic 的当前进程时钟映射；调用前须处于非实时生命周期线程。
     */
    runtime::SourceResult configure(
        const runtime::CameraCaptureConfig& config) override;

    /**
     * @brief 从回调队列取帧、解码为拥有型 BGR8 图像并返回时间戳。
     * @param frame 非空输出指针；成功时图像所有权由返回帧持有。
     * @return 捕获状态；队列等待期间可能阻塞，取消通过 SourceStatus::cancelled 返回。
     * @note 采样时间由首帧基线及采集序号推算；sensor_time.source_time_ns 保留未加 bias 的 SDK 单调转换值。
     *       received_timestamp_ns 保留原始主机回调时间，不随序号时钟内部的补偿观察一起平移。
     */
    runtime::SourceResult capture(runtime::CameraFrame* frame) override;

    /**
     * @brief 返回由 SDK 回调帧计数计算的有效采集帧率。
     * @return 实测 FPS；采样不足或统计异常时为 0。
     */
    double capture_fps() const noexcept override;
    bool uses_sequence_time() const noexcept override { return true; }
    /** @param sync Shared prelaunch barrier; bind before configure. */
    void bind_acquisition_sync(std::shared_ptr<domain::AcquisitionSync> sync) override {
        acquisition_sync_ = sync;
        timeline_.bind(std::move(sync), 1);
    }
    /** @return Fixed-clock observations in the bias-shifted domain; preserved after stop.
     * @note Receive residual = (raw host receive + bias) - fixed sample time; fixed bias cancels out.
     */
    domain::SequenceTimeDiagnostics time_diagnostics() const noexcept {
        auto result = timeline_.diagnostics();
        result.baseline_compensation_ns = config_.camera_timestamp_bias_ns;
        return result;
    }

    /**
     * @brief 对捕获帧执行尺寸调整及相机安装方向修正。
     * @param frame 输入/输出相机帧；图像必须是有效 BGR8。
     * @return 预处理状态；失败时携带适配器错误信息。
     */
    runtime::SourceResult prepare(runtime::CameraFrame* frame) override;

    /** @brief 非阻塞请求取消捕获并关闭待取帧队列。 */
    void cancel() noexcept override;
    /** @brief 等待捕获/回调退出并释放 SDK 视频会话。 */
    void stop() noexcept override;

    /**
     * @brief 获取 SDK 实际选中的分辨率、帧率、曝光和增益。
     * @return 最近一次成功配置所记录的相机参数。
     */
    CameraSdkConfiguration sdk_configuration() const noexcept;

    /**
     * @brief 获取回调等待、队列延迟、解码和丢帧统计。
     * @return 相机阶段时序及丢帧计数快照。
     */
    NoriCameraTiming camera_timing() const;

private:
    struct CallbackContext {
        mutable std::mutex mutex;
        std::condition_variable condition;
        NoriSdkCameraSource* owner = nullptr;
        bool callbacks_enabled = false;
        uint32_t callbacks_in_flight = 0;
        std::atomic<bool> cancel_requested{true};
    };

    struct PendingFrame {
        std::vector<uint8_t> bytes;
        uint32_t media_type = 0;
        uint32_t width = 0;
        uint32_t height = 0;
        runtime::TimestampNs timestamp_ns = 0;
        runtime::TimestampNs raw_timestamp_ns = -1; ///< Uncompensated SDK time mapped to monotonic ns.
        std::uint64_t sequence = 0;
        std::uint64_t acquisition_epoch = 0;
        runtime::TimestampNs received_timestamp_ns = 0;
        std::chrono::steady_clock::time_point enqueued_at{};
        runtime::CameraTimestampSource timestamp_source =
            runtime::CameraTimestampSource::unknown;
    };

    class CaptureGuard {
    public:
        explicit CaptureGuard(NoriSdkCameraSource* owner) noexcept
            : owner_(owner) {}
        ~CaptureGuard() noexcept;

        CaptureGuard(const CaptureGuard&) = delete;
        CaptureGuard& operator=(const CaptureGuard&) = delete;

    private:
        NoriSdkCameraSource* owner_;
    };

    /**
     * @brief Nori SDK 回调入口，将回调交给仍有效的 source 实例处理。
     * @param sdk_context SDK 回调上下文；当前实现不读取该参数。
     * @param frame SDK 提供的帧描述；数据只在回调期有效。
     * @param user 注册回调时提供的 CallbackContext。
     * @return SDK 回调状态码；无效或已取消回调按 NORI_OK 忽略。
     * @note 此处只保留帧数据、记录时间并入队，不执行图像解码或制导处理。
     */
    static uint32_t frame_callback(PVOID sdk_context,
                                  FRAME_BUFFER_DATA* frame, PVOID user);
    static CallbackContext* create_callback_context();
    static bool enter_callback(CallbackContext* context,
                               NoriSdkCameraSource** owner);
    /**
     * @brief 复制 SDK 临时帧缓冲并附上采集/接收时间后放入有界队列。
     * @param frame SDK 回调提供的帧描述。
     * @return SDK 回调状态码；队列满时丢弃最旧帧并累计诊断。
     */
    uint32_t enqueue_frame(FRAME_BUFFER_DATA* frame);
    void leave_callback() noexcept;
    void wait_for_callbacks() noexcept;
    void detach_callback_owner() noexcept;
    bool configure_exposure();
    bool enter_capture() noexcept;
    void leave_capture() noexcept;
    void wait_for_captures() noexcept;
    std::string error_message() const;
    runtime::SourceResult fatal_result() const;
    void set_error(const std::string& error);
    void record_camera_timing(
        runtime::TimingDistribution* distribution,
        std::chrono::steady_clock::duration duration) noexcept;
    void close_queue();
    void cancel_locked() noexcept;
    void stop_locked() noexcept;

    CameraAppConfig config_;
    domain::SequenceTime timeline_;
    std::shared_ptr<domain::AcquisitionSync> acquisition_sync_;
    // Serializes concurrent SDK callbacks and their sequence-clock state.
    std::mutex timestamp_mutex_;
    domain::AcquisitionSequence acquisition_sequence_;
    std::uint64_t sequence_epoch_ = 0;
    std::uint64_t previous_sequence_ = 0;
    std::shared_ptr<NoriSdkApi> sdk_;
    NoriSdkSession session_;
    uint32_t device_id_ = 0;
    VIDEO_INFO selected_video_{};
    CameraSdkConfiguration sdk_configuration_{};
    bool session_acquired_ = false;
    bool device_video_initialized_ = false;
    bool video_started_ = false;
    bool configured_ = false;
    runtime::FrameRateMeter callback_capture_fps_meter_;
    runtime::CameraClockMapping camera_clock_mapping_{};
    std::atomic<std::uint64_t> dropped_timestamp_count_{0};
    std::atomic<std::uint64_t> dropped_callback_queue_count_{0};
    mutable std::mutex camera_timing_mutex_;
    runtime::TimingDistribution callback_wait_timing_;
    runtime::TimingDistribution callback_queue_delay_timing_;
    runtime::TimingDistribution decode_convert_timing_;

    mutable std::mutex lifecycle_mutex_;
    mutable std::mutex capture_mutex_;
    std::condition_variable capture_condition_;
    std::size_t captures_in_flight_ = 0;
    bool capture_stopping_ = true;
    CallbackContext* callback_context_ = nullptr;
    mutable std::mutex queue_mutex_;
    std::condition_variable queue_condition_;
    std::deque<PendingFrame> pending_frames_;
    bool queue_closed_ = true;

    mutable std::mutex error_mutex_;
    std::string last_error_;
};

}  // mosas::app 命名空间结束

#endif  // MOSAS_ADAPTERS_NORI_CAMERA_SOURCE_HPP 头文件保护结束
