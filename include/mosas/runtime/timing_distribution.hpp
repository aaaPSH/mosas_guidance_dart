#ifndef MOSAS_RUNTIME_TIMING_DISTRIBUTION_HPP
#define MOSAS_RUNTIME_TIMING_DISTRIBUTION_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace mosas::runtime {

struct TimingDistributionSummary {
    std::uint64_t samples = 0;
    std::uint64_t minimum_ns = 0;
    double average_ns = 0.0;
    std::uint64_t p50_ns = 0;
    std::uint64_t p95_ns = 0;
    std::uint64_t p99_ns = 0;
    std::uint64_t maximum_ns = 0;
};

// 使用固定大小的指数直方图统计纳秒耗时；调用方负责提供外部同步。
class TimingDistribution {
public:
    // 记录一个纳秒耗时，不申请动态内存，也不执行内部加锁。
    void record_ns(std::uint64_t duration_ns) noexcept {
        increment_saturating(samples_);

        if (duration_ns == 0) {
            increment_saturating(zero_samples_);
        } else {
            increment_saturating(histogram_[bucket_index(duration_ns)]);
        }

        if (samples_ == 1) {
            minimum_ns_ = duration_ns;
            maximum_ns_ = duration_ns;
            average_ns_ = static_cast<double>(duration_ns);
            return;
        }

        if (duration_ns < minimum_ns_) {
            minimum_ns_ = duration_ns;
        }
        if (duration_ns > maximum_ns_) {
            maximum_ns_ = duration_ns;
        }

        // 在线均值避免累计耗时求和溢出。
        average_ns_ += (static_cast<double>(duration_ns) - average_ns_) /
                       static_cast<double>(samples_);
    }

    // 返回纳秒统计；百分位值是对应直方图桶的代表值。
    TimingDistributionSummary summary() const noexcept {
        TimingDistributionSummary result;
        result.samples = samples_;
        if (samples_ == 0) {
            return result;
        }

        result.minimum_ns = minimum_ns_;
        result.average_ns = average_ns_;
        result.p50_ns = percentile_ns(50);
        result.p95_ns = percentile_ns(95);
        result.p99_ns = percentile_ns(99);
        result.maximum_ns = maximum_ns_;
        return result;
    }

    // 清空样本和所有统计值。
    void reset() noexcept {
        samples_ = 0;
        zero_samples_ = 0;
        minimum_ns_ = 0;
        maximum_ns_ = 0;
        average_ns_ = 0.0;
        for (auto& bucket : histogram_) {
            bucket = 0;
        }
    }

private:
    static constexpr std::size_t kBucketsPerExponent = 64;
    static constexpr std::size_t kExponentCount = 64;
    static constexpr std::size_t kBucketCount =
        kBucketsPerExponent * kExponentCount;

    static void increment_saturating(std::uint64_t& value) noexcept {
        if (value != std::numeric_limits<std::uint64_t>::max()) {
            ++value;
        }
    }

    // 求非零整数的最高置位位置，兼容 C++17。
    static unsigned int floor_log2(std::uint64_t value) noexcept {
        unsigned int exponent = 0;
        if (value >= (std::uint64_t{1} << 32)) {
            value >>= 32;
            exponent += 32;
        }
        if (value >= (std::uint64_t{1} << 16)) {
            value >>= 16;
            exponent += 16;
        }
        if (value >= (std::uint64_t{1} << 8)) {
            value >>= 8;
            exponent += 8;
        }
        if (value >= (std::uint64_t{1} << 4)) {
            value >>= 4;
            exponent += 4;
        }
        if (value >= (std::uint64_t{1} << 2)) {
            value >>= 2;
            exponent += 2;
        }
        if (value >= (std::uint64_t{1} << 1)) {
            ++exponent;
        }
        return exponent;
    }

    // 每个 [2^e, 2^(e+1)) 区间划分为 64 个桶；小值保持整数精度。
    static std::size_t bucket_index(std::uint64_t value) noexcept {
        const unsigned int exponent = floor_log2(value);
        const std::uint64_t base = std::uint64_t{1} << exponent;
        const std::uint64_t offset = value - base;
        const unsigned int sub_bucket = exponent < 6
            ? static_cast<unsigned int>(offset << (6 - exponent))
            : static_cast<unsigned int>(offset >> (exponent - 6));
        return static_cast<std::size_t>(exponent) * kBucketsPerExponent +
               sub_bucket;
    }

    // 返回桶中点附近的代表纳秒值；小于 128 ns 的桶精确表示整数值。
    static std::uint64_t bucket_representative_ns(
        std::size_t index) noexcept {
        const unsigned int exponent = static_cast<unsigned int>(
            index / kBucketsPerExponent);
        const std::uint64_t sub_bucket = static_cast<std::uint64_t>(
            index % kBucketsPerExponent);
        const std::uint64_t base = std::uint64_t{1} << exponent;

        if (exponent < 6) {
            return base + (sub_bucket >> (6 - exponent));
        }

        const std::uint64_t width = std::uint64_t{1} << (exponent - 6);
        const std::uint64_t lower = base + sub_bucket * width;
        return lower + (width - 1) / 2;
    }

    static std::uint64_t percentile_rank(std::uint64_t samples,
                                         unsigned int percentile) noexcept {
        const std::uint64_t whole = (samples / 100) * percentile;
        const std::uint64_t remainder =
            ((samples % 100) * percentile + 99) / 100;
        const std::uint64_t rank = whole + remainder;
        return rank == 0 ? 1 : rank;
    }

    std::uint64_t percentile_ns(unsigned int percentile) const noexcept {
        const std::uint64_t target = percentile_rank(samples_, percentile);
        std::uint64_t cumulative = zero_samples_;
        if (cumulative >= target) {
            return 0;
        }

        for (std::size_t index = 0; index < histogram_.size(); ++index) {
            const std::uint64_t bucket_samples = histogram_[index];
            if (bucket_samples >
                std::numeric_limits<std::uint64_t>::max() - cumulative) {
                cumulative = std::numeric_limits<std::uint64_t>::max();
            } else {
                cumulative += bucket_samples;
            }
            if (cumulative >= target) {
                return bucket_representative_ns(index);
            }
        }

        // 仅在计数器饱和等极端情况下到达此处。
        return maximum_ns_;
    }

    std::array<std::uint64_t, kBucketCount> histogram_{};
    std::uint64_t samples_ = 0;
    std::uint64_t zero_samples_ = 0;
    std::uint64_t minimum_ns_ = 0;
    std::uint64_t maximum_ns_ = 0;
    double average_ns_ = 0.0;
};

}  // namespace mosas::runtime

#endif  // MOSAS_RUNTIME_TIMING_DISTRIBUTION_HPP
