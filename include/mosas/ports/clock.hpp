#ifndef MOSAS_PORTS_CLOCK_HPP
#define MOSAS_PORTS_CLOCK_HPP

#include <cstdint>
#include <mosas/domain/time.hpp>

namespace mosas::ports {

class IMonotonicClock {
public:
    virtual ~IMonotonicClock() = default;
    virtual std::int64_t now_ns() const noexcept = 0;
    /** @return 当前纳秒值所处的已换算时间轴，不是传感器原始时钟域。 */
    virtual domain::ClockDomain clock_domain() const noexcept = 0;
};

}  // namespace mosas::ports

#endif  // MOSAS_PORTS_CLOCK_HPP
