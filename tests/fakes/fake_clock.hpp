#ifndef MOSAS_TEST_FAKES_FAKE_CLOCK_HPP
#define MOSAS_TEST_FAKES_FAKE_CLOCK_HPP

#include <mosas/ports/clock.hpp>

#include <atomic>
#include <cstdint>

namespace mosas::test {

class FakeClock final : public ports::IMonotonicClock {
public:
    explicit FakeClock(std::int64_t initial_ns = 0, domain::ClockDomain domain = domain::ClockDomain::monotonic) noexcept
        : now_ns_(initial_ns), domain_(domain) {}

    domain::ClockDomain clock_domain() const noexcept override { return domain_; }

    std::int64_t now_ns() const noexcept override {
        return now_ns_.load(std::memory_order_relaxed);
    }

    void set(std::int64_t timestamp_ns) noexcept {
        now_ns_.store(timestamp_ns, std::memory_order_relaxed);
    }

    void advance(std::int64_t delta_ns) noexcept {
        now_ns_.fetch_add(delta_ns, std::memory_order_relaxed);
    }

private:
    std::atomic<std::int64_t> now_ns_;
    domain::ClockDomain domain_;
};

}  // namespace mosas::test

#endif  // MOSAS_TEST_FAKES_FAKE_CLOCK_HPP
