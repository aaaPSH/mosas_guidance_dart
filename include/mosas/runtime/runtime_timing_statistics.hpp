#ifndef MOSAS_RUNTIME_TIMING_STATISTICS_HPP
#define MOSAS_RUNTIME_TIMING_STATISTICS_HPP

#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

namespace mosas::runtime {

struct RuntimeTimingSnapshot {
    std::uint64_t samples = 0;
    double average_ms = 0.0;
    double p95_ms = 0.0;
    double p99_ms = 0.0;
    double maximum_ms = 0.0;
    std::uint64_t deadline_miss_count = 0;
};

// Fixed-cost duration statistics. The first 2048 buckets are 0.1 ms wide;
// the final bucket records values at or above 204.8 ms. Percentiles are
// reported as the upper bound of the selected bucket.
class RuntimeTimingAccumulator {
public:
    static constexpr std::uint64_t kHistogramBucketWidthNs = 100'000;
    static constexpr std::size_t kHistogramBucketCount = 2'048;

    RuntimeTimingAccumulator() noexcept {
        reset();
    }

    RuntimeTimingAccumulator(const RuntimeTimingAccumulator&) = delete;
    RuntimeTimingAccumulator& operator=(const RuntimeTimingAccumulator&) =
        delete;

    void reset() noexcept {
        samples_.store(0, std::memory_order_relaxed);
        total_ns_.store(0, std::memory_order_relaxed);
        maximum_ns_.store(0, std::memory_order_relaxed);
        deadline_miss_count_.store(0, std::memory_order_relaxed);
        for (auto& bucket : buckets_) {
            bucket.store(0, std::memory_order_relaxed);
        }
    }

    void record(std::uint64_t duration_ns,
                std::uint64_t deadline_ns = 0) noexcept {
        buckets_[bucket_index(duration_ns)].fetch_add(
            1, std::memory_order_relaxed);
        total_ns_.fetch_add(duration_ns, std::memory_order_relaxed);

        std::uint64_t maximum = maximum_ns_.load(std::memory_order_relaxed);
        while (maximum < duration_ns &&
               !maximum_ns_.compare_exchange_weak(
                   maximum, duration_ns, std::memory_order_relaxed,
                   std::memory_order_relaxed)) {
        }

        if (deadline_ns != 0 && duration_ns > deadline_ns) {
            deadline_miss_count_.fetch_add(1, std::memory_order_relaxed);
        }
        samples_.fetch_add(1, std::memory_order_relaxed);
    }

    RuntimeTimingSnapshot snapshot() const noexcept {
        const std::uint64_t samples =
            samples_.load(std::memory_order_relaxed);
        if (samples == 0) {
            return {};
        }

        const std::uint64_t total_ns =
            total_ns_.load(std::memory_order_relaxed);
        const std::uint64_t maximum_ns =
            maximum_ns_.load(std::memory_order_relaxed);
        return {samples,
                static_cast<double>(total_ns) /
                    static_cast<double>(samples) / 1e6,
                percentile_ms(samples, 95, maximum_ns),
                percentile_ms(samples, 99, maximum_ns),
                static_cast<double>(maximum_ns) / 1e6,
                deadline_miss_count_.load(std::memory_order_relaxed)};
    }

private:
    static constexpr std::size_t kOverflowBucket = kHistogramBucketCount;
    static constexpr std::size_t kTotalBucketCount =
        kHistogramBucketCount + 1;

    static std::size_t bucket_index(std::uint64_t duration_ns) noexcept {
        const std::uint64_t bucket =
            duration_ns / kHistogramBucketWidthNs;
        return bucket >= kHistogramBucketCount
                   ? kOverflowBucket
                   : static_cast<std::size_t>(bucket);
    }

    double percentile_ms(std::uint64_t samples, std::uint64_t percentile,
                         std::uint64_t maximum_ns) const noexcept {
        const std::uint64_t rank = std::max<std::uint64_t>(
            1, (samples * percentile + 99) / 100);
        std::uint64_t cumulative = 0;
        for (std::size_t index = 0; index < kTotalBucketCount; ++index) {
            cumulative += buckets_[index].load(std::memory_order_relaxed);
            if (cumulative >= rank) {
                if (index == kOverflowBucket) {
                    return static_cast<double>(maximum_ns) / 1e6;
                }
                return static_cast<double>((index + 1) *
                                           kHistogramBucketWidthNs) /
                       1e6;
            }
        }
        return static_cast<double>(maximum_ns) / 1e6;
    }

    std::atomic<std::uint64_t> samples_{0};
    std::atomic<std::uint64_t> total_ns_{0};
    std::atomic<std::uint64_t> maximum_ns_{0};
    std::atomic<std::uint64_t> deadline_miss_count_{0};
    std::array<std::atomic<std::uint64_t>, kTotalBucketCount> buckets_{};
};

}  // namespace mosas::runtime

#endif  // MOSAS_RUNTIME_TIMING_STATISTICS_HPP
