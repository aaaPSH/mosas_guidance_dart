#include <mosas/domain/sequence_time.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace mosas::domain {
AcquisitionSync::State AcquisitionSync::state() const noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    return {epoch_, (ready_ & required_) == required_, launched_};
}
void AcquisitionSync::restart() noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    ready_ = 0;
    epoch_ = 0;
    launched_ = false;
}
void AcquisitionSync::ready(unsigned source, std::uint64_t epoch) noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    if (epoch == epoch_) ready_ |= source;
}
bool AcquisitionSync::invalidate() noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    if (launched_) return false;
    ++epoch_;
    ready_ = 0;
    return true;
}
bool AcquisitionSync::latch_launch(std::uint64_t epoch) noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    if (epoch != epoch_ || (ready_ & required_) != required_) return false;
    launched_ = true;
    return true;
}

SequenceTime::SequenceTime(SequenceTimeConfig config) : config_(config) {
    start(0);
}
bool SequenceTime::valid_config(const SequenceTimeConfig& c) noexcept {
    return std::isfinite(c.frequency_hz) && c.frequency_hz > 0 && c.frequency_hz <= 1e9 &&
           c.warmup_ns >= 0 && c.stable_window_ns >= 0 && c.tolerance_ns > 0 &&
           c.frequency_window_ns > 0;
}
void SequenceTime::bind(std::shared_ptr<AcquisitionSync> sync, unsigned source) {
    sync_ = std::move(sync);
    source_ = source;
}
void SequenceTime::reset_segment(std::int64_t now_ns, std::uint64_t epoch) noexcept {
    epoch_ = epoch;
    start_ns_ = now_ns;
    stable_start_ns_ = -1;
    stable_receive_ns_ = -1;
    have_sequence_ = false;
    stats_.synchronized = false;
    window_receive_ns_ = -1;
    frequency_dirty_ = false;
}
void SequenceTime::start(std::int64_t now_ns) noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    stats_ = {};
    stats_.configured_hz = config_.frequency_hz;
    stats_.warmup_ns = config_.warmup_ns;
    stats_.stable_window_ns = config_.stable_window_ns;
    stats_.tolerance_ns = config_.tolerance_ns;
    stats_.frequency_window_ns = config_.frequency_window_ns;
    slope_count_ = 0;
    reset_segment(now_ns, sync_ ? sync_->state().epoch : 0);
}
bool SequenceTime::launched() const noexcept { return sync_ && sync_->state().launched; }
std::optional<std::int64_t> SequenceTime::offset(std::uint64_t delta) const noexcept {
    const long double ns = std::round(static_cast<long double>(delta) * 1e9L / config_.frequency_hz);
    if (!std::isfinite(ns) || ns < 0 || ns >= static_cast<long double>(std::numeric_limits<std::int64_t>::max()))
        return std::nullopt;
    return static_cast<std::int64_t>(ns);
}
void SequenceTime::observe_frequency(std::uint64_t sequence, std::int64_t receive, bool clean) noexcept {
    if (!clean || frequency_dirty_) {
        window_receive_ns_ = -1;
        frequency_dirty_ = false;
    }
    if (!clean) return;
    if (window_receive_ns_ < 0) {
        window_receive_ns_ = receive;
        window_sequence_ = sequence;
        return;
    }
    if (receive <= window_receive_ns_ || sequence <= window_sequence_) {
        window_receive_ns_ = -1;
        return;
    }
    const auto elapsed = receive - window_receive_ns_;
    if (elapsed < config_.frequency_window_ns) return;
    const double hz = static_cast<double>(sequence - window_sequence_) * 1e9 / elapsed;
    slopes_[slope_count_++ % slopes_.size()] = hz;
    const auto count = std::min(slope_count_, slopes_.size());
    auto sorted = slopes_;
    std::sort(sorted.begin(), sorted.begin() + count);
    stats_.estimated_hz = sorted[count / 2];
    stats_.frequency_error_ppm = (stats_.estimated_hz / config_.frequency_hz - 1) * 1e6;
    stats_.drift_ns_per_second = (1 - stats_.estimated_hz / config_.frequency_hz) * 1e9;
    ++stats_.frequency_windows;
    window_receive_ns_ = receive;
    window_sequence_ = sequence;
}
std::optional<SequenceTimestamp> SequenceTime::observe(
    std::uint64_t sequence, std::int64_t candidate, std::int64_t receive, std::int64_t now) noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    ++stats_.observed;
    if (!valid_config(config_) || receive < 0 || now < 0) return std::nullopt;
    if (stats_.first_receive_ns < 0) stats_.first_receive_ns = receive;
    const auto previous_receive = stats_.last_receive_ns;
    stats_.last_receive_ns = receive;
    if (sync_ && sync_->state().epoch != epoch_) reset_segment(now, sync_->state().epoch);
    if (have_sequence_ && sequence <= previous_sequence_) {
        ++stats_.sequence_errors;
        if (launched()) ++stats_.post_launch_anomalies;
        frequency_dirty_ = true;
        return std::nullopt;
    }
    const auto interval = have_sequence_ ? offset(sequence - previous_sequence_) : std::nullopt;
    const bool clean_cadence = !have_sequence_ || (interval && previous_receive >= 0 &&
        receive > previous_receive && std::abs(static_cast<long double>(receive) - previous_receive - *interval) <= config_.tolerance_ns);
    previous_sequence_ = sequence;
    have_sequence_ = true;
    const bool fresh = std::abs(static_cast<long double>(now) - receive) <= config_.tolerance_ns;
    if (!stats_.synchronized) {
        ++stats_.warmup_discarded;
        if (!fresh || candidate < 0 || now < start_ns_ || now - start_ns_ < config_.warmup_ns) {
            stable_start_ns_ = -1;
            return std::nullopt;
        }
        if (stable_start_ns_ < 0) {
            stable_start_ns_ = now;
            stable_receive_ns_ = receive;
            stable_sequence_ = sequence;
        }
        const auto predicted = offset(sequence - stable_sequence_);
        if (!predicted || std::abs(static_cast<long double>(receive) - stable_receive_ns_ - *predicted) > config_.tolerance_ns) {
            stable_start_ns_ = now;
            stable_receive_ns_ = receive;
            stable_sequence_ = sequence;
            return std::nullopt;
        }
        if (now - stable_start_ns_ < config_.stable_window_ns ||
            receive - stable_receive_ns_ < config_.stable_window_ns) return std::nullopt;
        stats_.baseline_ns = candidate;
        origin_sequence_ = sequence;
        stats_.synchronized = true;
        ++stats_.baselines;
        --stats_.warmup_discarded;
        if (sync_) sync_->ready(source_, epoch_);
    }
    const auto delta = offset(sequence - origin_sequence_);
    if (!delta || stats_.baseline_ns > std::numeric_limits<std::int64_t>::max() - *delta) return std::nullopt;
    const auto timestamp = stats_.baseline_ns + *delta;
    const auto residual = receive - timestamp;
    if (!stats_.residual_available) {
        stats_.first_residual_ns = stats_.minimum_residual_ns = stats_.maximum_residual_ns = residual;
        stats_.residual_available = true;
    }
    stats_.last_residual_ns = residual;
    stats_.minimum_residual_ns = std::min(stats_.minimum_residual_ns, residual);
    stats_.maximum_residual_ns = std::max(stats_.maximum_residual_ns, residual);
    const bool exceeded = std::abs(static_cast<long double>(residual)) > config_.tolerance_ns;
    if (exceeded) {
        ++stats_.residual_exceeded;
        if (launched()) ++stats_.post_launch_anomalies;
        else if (!sync_ || sync_->invalidate()) {
            reset_segment(now, sync_ ? sync_->state().epoch : epoch_ + 1);
            return std::nullopt;
        } else {
            // Launch can latch between the snapshot and invalidation; it wins over resynchronization.
            ++stats_.post_launch_anomalies;
        }
    }
    observe_frequency(sequence, receive, fresh && clean_cadence);
    ++stats_.published;
    return SequenceTimestamp{timestamp, epoch_};
}
void SequenceTime::discontinuity() noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    ++stats_.unknown_discontinuities;
    frequency_dirty_ = true;
    if (launched()) { ++stats_.post_launch_anomalies; return; }
    if (sync_ && !sync_->invalidate()) { ++stats_.post_launch_anomalies; return; }
    reset_segment(stats_.last_receive_ns >= 0 ? stats_.last_receive_ns : start_ns_,
                  sync_ ? sync_->state().epoch : epoch_ + 1);
}
void SequenceTime::missing(std::uint64_t count) noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    stats_.known_missing += count;
    frequency_dirty_ = true;
    if (launched() && count) ++stats_.post_launch_anomalies;
}
void SequenceTime::sequence_error() noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    ++stats_.sequence_errors;
    frequency_dirty_ = true;
    if (launched()) ++stats_.post_launch_anomalies;
}
void SequenceTime::observe_capture_time(std::int64_t capture, std::int64_t fixed) noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    if (capture < 0) { ++stats_.invalid_capture_times; return; }
    const auto residual = capture - fixed;
    if (!stats_.capture_residual_available) {
        stats_.minimum_capture_residual_ns = stats_.maximum_capture_residual_ns = residual;
        stats_.capture_residual_available = true;
    }
    stats_.minimum_capture_residual_ns = std::min(stats_.minimum_capture_residual_ns, residual);
    stats_.maximum_capture_residual_ns = std::max(stats_.maximum_capture_residual_ns, residual);
}
SequenceTimeDiagnostics SequenceTime::diagnostics() const noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    return stats_;
}
}
