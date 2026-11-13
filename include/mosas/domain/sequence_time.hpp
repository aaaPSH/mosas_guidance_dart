#ifndef MOSAS_DOMAIN_SEQUENCE_TIME_HPP
#define MOSAS_DOMAIN_SEQUENCE_TIME_HPP

#include <array>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>

namespace mosas::domain {

/** @brief 固定周期采集配置；所有时长均以单调时钟的纳秒为单位。 */
struct SequenceTimeConfig {
    double frequency_hz = 1000.0;
    std::int64_t warmup_ns = 500'000'000;
    std::int64_t stable_window_ns = 200'000'000;
    std::int64_t tolerance_ns = 50'000'000;
    std::int64_t frequency_window_ns = 2'000'000'000;
};

/** @brief 启动前共享同步屏障；启动状态一旦锁存，将保持至重启。 */
class AcquisitionSync final {
public:
    struct State { std::uint64_t epoch; bool ready; bool launched; };
    /** @param mask 参与屏障同步的硬件源位掩码（相机=1，IMU=2）。 */
    explicit AcquisitionSync(unsigned mask) : required_(mask) {}
    /** @return 一致的同步状态快照。 */
    State state() const noexcept;
    /** @brief 重置屏障，仅允许在所有采集和估计器工作线程均已停止时调用。 */
    void restart() noexcept;
    /** @param source 参与同步的数据源位。@param epoch 观测到的同步轮次；忽略过期轮次的就绪通知。 */
    void ready(unsigned source, std::uint64_t epoch) noexcept;
    /** @brief 仅在启动前使两个数据源的就绪状态失效。@return 失效操作是否生效。 */
    bool invalidate() noexcept;
    /** @param epoch 估计器的同步轮次。@return 仅当两个数据源均在该轮次就绪时返回真。 */
    bool latch_launch(std::uint64_t epoch) noexcept;
private:
    mutable std::mutex mutex_;
    unsigned required_;
    unsigned ready_ = 0;
    std::uint64_t epoch_ = 0;
    bool launched_ = false;
};

/** @brief 最终或实时的时序观测结果；接收残差并非真实的采样误差。 */
struct SequenceTimeDiagnostics {
    double configured_hz = 0;
    std::int64_t baseline_compensation_ns = 0;
    std::int64_t warmup_ns = 0, stable_window_ns = 0, tolerance_ns = 0, frequency_window_ns = 0;
    double estimated_hz = 0;
    double frequency_error_ppm = 0;
    double drift_ns_per_second = 0;
    std::uint64_t frequency_windows = 0;
    std::uint64_t observed = 0, published = 0, warmup_discarded = 0;
    std::uint64_t known_missing = 0, unknown_discontinuities = 0;
    std::uint64_t sequence_errors = 0, residual_exceeded = 0, post_launch_anomalies = 0;
    std::uint64_t baselines = 0;
    std::int64_t baseline_ns = -1, first_receive_ns = -1, last_receive_ns = -1;
    std::int64_t first_residual_ns = 0, last_residual_ns = 0;
    std::int64_t minimum_residual_ns = 0, maximum_residual_ns = 0;
    std::uint64_t invalid_capture_times = 0;
    std::int64_t minimum_capture_residual_ns = 0, maximum_capture_residual_ns = 0;
    bool capture_residual_available = false;
    bool residual_available = false;
    bool synchronized = false;
};

struct SequenceTimestamp {
    std::int64_t timestamp_ns;
    std::uint64_t epoch;
};

/** @brief 展开 32 位采集计数器；拒绝重复、倒退或存在歧义的跳变。 */
class AcquisitionSequence final {
public:
    /** @brief 开始新的启动前采集区段。 */
    void reset() noexcept { initialized_ = false; unwrapped_ = 0; }
    /** @param raw 设备采集计数值。@return 展开后的采样序号；顺序无效时返回空值。 */
    std::optional<std::uint64_t> push(std::uint32_t raw) noexcept {
        if (!initialized_) { initialized_ = true; previous_ = raw; return unwrapped_; }
        const std::uint32_t delta = raw - previous_;
        if (delta == 0 || delta >= 0x80000000u || unwrapped_ > UINT64_MAX - delta) return std::nullopt;
        previous_ = raw;
        unwrapped_ += delta;
        return unwrapped_;
    }
private:
    bool initialized_ = false;
    std::uint32_t previous_ = 0;
    std::uint64_t unwrapped_ = 0;
};

/** @brief 单写入者的固定周期序列时钟，支持同步获取诊断快照。
 * @note 基线取稳定窗口内相对首个有效序号归一化的最小候选，仅为估计，不保证绝对采样时间准确。
 *       发射后，接收观测值不会调整已确定的基准时间或周期。
 */
class SequenceTime final {
public:
    /** @param config 固定周期及同步判定条件。 */
    explicit SequenceTime(SequenceTimeConfig config);
    /** @param config 待验证的配置。@return 算术运算参数及时间窗口是否合理。 */
    static bool valid_config(const SequenceTimeConfig& config) noexcept;
    /** @param sync 共享同步屏障。@param source 数据源位（1 为相机，2 为 IMU）；须在启动前绑定。 */
    void bind(std::shared_ptr<AcquisitionSync> sync, unsigned source);
    /** @param now_ns 采集开始时间；清空所有统计数据，但不重置共享屏障。 */
    void start(std::int64_t now_ns) noexcept;
    /** @brief 观测一帧，并在条件满足时生成其固定周期时间戳。
     * @param sequence 展开后的硬件序号或本地逻辑采样序号。
     * @param candidate_ns 当前帧补偿后的时间估计，仅用于稳定窗口内的基线估计。
     * @param receive_ns 补偿后的接收观测时间，独立于生成的时间戳。
     * @param now_ns 同一补偿时间域内的当前主机时间，用于限制排队数据的滞留时长。
     * @return 预热后的时间戳及同步轮次；未同步或观测无效时返回空值。
     */
    std::optional<SequenceTimestamp> observe(std::uint64_t sequence, std::int64_t candidate_ns,
                                            std::int64_t receive_ns, std::int64_t now_ns) noexcept;
    /** @brief 记录数量未知的丢失样本；仅在启动前使同步屏障失效。 */
    void discontinuity() noexcept;
    /** @param count 已确认的丢失样本数；序号偏移必须已包含这些采样位置。 */
    void missing(std::uint64_t count) noexcept;
    /** @brief 记录被拒绝的序号，不推测其前向偏移。 */
    void sequence_error() noexcept;
    /** @param capture_ns 独立转换的 SDK 采集时间，不可用时为负值。
     * @param fixed_ns 生成的采样时间；仅记录观测依据，不改变时钟。
     */
    void observe_capture_time(std::int64_t capture_ns, std::int64_t fixed_ns) noexcept;
    /** @return 线程安全的快照；没有可信时间窗口时，频率估计值为零。 */
    SequenceTimeDiagnostics diagnostics() const noexcept;
private:
    std::optional<std::int64_t> offset(std::uint64_t delta) const noexcept;
    void reset_segment(std::int64_t now_ns, std::uint64_t epoch) noexcept;
    void observe_frequency(std::uint64_t sequence, std::int64_t receive_ns, bool clean) noexcept;
    bool launched() const noexcept;
    SequenceTimeConfig config_;
    std::shared_ptr<AcquisitionSync> sync_;
    unsigned source_ = 0;
    mutable std::mutex mutex_;
    SequenceTimeDiagnostics stats_{};
    std::uint64_t epoch_ = 0, origin_sequence_ = 0, previous_sequence_ = 0;
    std::int64_t start_ns_ = 0, stable_start_ns_ = -1, stable_receive_ns_ = -1;
    std::int64_t stable_baseline_ns_ = -1;
    std::uint64_t stable_sequence_ = 0;
    bool have_sequence_ = false;
    bool frequency_dirty_ = false;
    std::uint64_t window_sequence_ = 0;
    std::int64_t window_receive_ns_ = -1;
    std::array<double, 9> slopes_{};
    std::size_t slope_count_ = 0;
};
}
#endif
