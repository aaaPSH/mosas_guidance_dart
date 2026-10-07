#ifndef MOSAS_PORTS_CLOCK_HPP
#define MOSAS_PORTS_CLOCK_HPP

#include <cstdint>

namespace mosas::ports {

class IMonotonicClock {
public:
    virtual ~IMonotonicClock() = default;
    virtual std::int64_t now_ns() const noexcept = 0;
};

}  // namespace mosas::ports

#endif  // MOSAS_PORTS_CLOCK_HPP
