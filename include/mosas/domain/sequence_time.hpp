#ifndef MOSAS_DOMAIN_SEQUENCE_TIME_HPP
#define MOSAS_DOMAIN_SEQUENCE_TIME_HPP

#include <array>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>

namespace mosas::domain {

/** @brief Fixed-period acquisition settings; all durations are monotonic nanoseconds. */
struct SequenceTimeConfig {
    double frequency_hz = 1000.0;
    std::int64_t warmup_ns = 500'000'000;
    std::int64_t stable_window_ns = 200'000'000;
    std::int64_t tolerance_ns = 50'000'000;
    std::int64_t frequency_window_ns = 2'000'000'000;
};

/** @brief Shared prelaunch synchronization barrier; launch is latched until restart. */
class AcquisitionSync final {
public:
    struct State { std::uint64_t epoch; bool ready; bool launched; };
    /** @param mask Hardware sources participating in the barrier (camera=1, IMU=2). */
    explicit AcquisitionSync(unsigned mask) : required_(mask) {}
    /** @return A coherent synchronization snapshot. */
    State state() const noexcept;
    /** @brief Reset the barrier only while all acquisition and estimator workers are stopped. */
    void restart() noexcept;
    /** @param source Participating source bit. @param epoch Observed epoch; stale readiness is ignored. */
    void ready(unsigned source, std::uint64_t epoch) noexcept;
    /** @brief Invalidate both sources before launch only. @return Whether invalidation took effect. */
    bool invalidate() noexcept;
    /** @param epoch Estimator's epoch. @return True only if both sources are ready in this epoch. */
    bool latch_launch(std::uint64_t epoch) noexcept;
private:
    mutable std::mutex mutex_;
    unsigned required_;
    unsigned ready_ = 0;
    std::uint64_t epoch_ = 0;
    bool launched_ = false;
};

/** @brief Final or live timing observations; receive residuals are not true sampling errors. */
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

/** @brief Unwrap a 32-bit acquisition counter; duplicate/backward/ambiguous jumps are rejected. */
class AcquisitionSequence final {
public:
    /** @brief Begin a new prelaunch acquisition segment. */
    void reset() noexcept { initialized_ = false; unwrapped_ = 0; }
    /** @param raw Device acquisition counter. @return Unwrapped slot or no value for invalid order. */
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

/** @brief Single-writer fixed sequence clock with synchronized diagnostic snapshots.
 * @note Reception observations never adjust a committed baseline or period after launch.
 */
class SequenceTime final {
public:
    /** @param config Fixed period and synchronization criteria. */
    explicit SequenceTime(SequenceTimeConfig config);
    /** @param config Candidate settings. @return Whether arithmetic and windows are meaningful. */
    static bool valid_config(const SequenceTimeConfig& config) noexcept;
    /** @param sync Shared barrier. @param source Source bit (1 camera, 2 IMU); bind before start. */
    void bind(std::shared_ptr<AcquisitionSync> sync, unsigned source);
    /** @param now_ns Start of acquisition; clears all statistics, not the shared barrier. */
    void start(std::int64_t now_ns) noexcept;
    /** @brief Observe a frame and optionally produce its fixed-period timestamp.
     * @param sequence Unwrapped hardware sequence or local logical sampling sequence.
     * @param candidate_ns Compensated first-frame time, used only to establish the baseline.
     * @param receive_ns Compensated reception observation, independent of the generated timestamp.
     * @param now_ns Current host time in the same compensated domain; bounds queued-data age.
     * @return Timestamp and epoch after warmup, or no value for unsynchronized/invalid observations.
     */
    std::optional<SequenceTimestamp> observe(std::uint64_t sequence, std::int64_t candidate_ns,
                                            std::int64_t receive_ns, std::int64_t now_ns) noexcept;
    /** @brief Record unknown missing samples; prelaunch only, invalidate the synchronization barrier. */
    void discontinuity() noexcept;
    /** @param count Confirmed missing samples; sequence offsets must already include these slots. */
    void missing(std::uint64_t count) noexcept;
    /** @brief Record a rejected sequence without guessing a forward offset. */
    void sequence_error() noexcept;
    /** @param capture_ns Independently converted SDK capture time, negative if unavailable.
     * @param fixed_ns Generated sample time; records evidence only, never alters the clock.
     */
    void observe_capture_time(std::int64_t capture_ns, std::int64_t fixed_ns) noexcept;
    /** @return Thread-safe snapshot; frequency estimate is zero if no trustworthy window exists. */
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
