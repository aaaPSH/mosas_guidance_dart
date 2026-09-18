#ifndef MOSAS_RUNTIME_RAW_SERIAL_CHUNK_HPP
#define MOSAS_RUNTIME_RAW_SERIAL_CHUNK_HPP

#include <mosas/runtime/clock.hpp>

#include <array>
#include <cstddef>
#include <cstdint>

namespace mosas::runtime {

struct RawSerialChunk {
    static constexpr std::size_t kMaxChunkBytes = 256;

    TimestampNs first_byte_timestamp_ns = -1;
    std::uint64_t sequence = 0;
    std::uint16_t size = 0;
    std::array<std::uint8_t, kMaxChunkBytes> bytes{};
};

}  // namespace mosas::runtime

#endif  // MOSAS_RUNTIME_RAW_SERIAL_CHUNK_HPP
