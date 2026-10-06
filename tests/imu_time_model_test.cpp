#include <mosas/app/imu_stream_decoder.hpp>
#include <mosas/domain/imu_time_model.hpp>

#include "imu_frame_fixture.hpp"
#include "test_support.hpp"

#include <algorithm>
#include <cstdint>
#include <limits>

int main() {
    using namespace mosas::domain;
    const SensorTime estimated = estimate_imu_sample_time(
        100'000'000LL, 7'500'000LL);
    CHECK(estimated.monotonic_ns == 92'500'000LL);
    CHECK(estimated.source_time_ns == -1);
    CHECK(estimated.uncertainty_ns == -1);
    CHECK(estimated.source_domain == ClockDomain::monotonic);
    CHECK(estimated.source == TimestampSource::imu_uart_receive_estimate);
    CHECK(estimated.quality == TimestampQuality::estimated);

    const SensorTime underflow = estimate_imu_sample_time(1, 2);
    CHECK(underflow.monotonic_ns == -1);
    CHECK(!underflow.is_valid());

    const auto saturated = estimate_imu_sample_time(
        std::numeric_limits<std::int64_t>::max(), -1);
    CHECK(saturated.monotonic_ns == std::numeric_limits<std::int64_t>::max());
    const auto minimum_latency = estimate_imu_sample_time(
        -1, std::numeric_limits<std::int64_t>::min());
    CHECK(minimum_latency.monotonic_ns ==
          std::numeric_limits<std::int64_t>::max());

    mosas::app::ImuStreamDecoder decoder(0, 2'000'000LL, 2);
    mosas::runtime::RawSerialChunk chunk{};
    chunk.first_byte_timestamp_ns = 100'000'000LL;
    chunk.sequence = 0;
    chunk.size = static_cast<std::uint16_t>(mosas::test::kValidImuFrame.size());
    std::copy(mosas::test::kValidImuFrame.begin(),
              mosas::test::kValidImuFrame.end(), chunk.bytes.begin());
    decoder.consume(chunk);

    mosas::runtime::ImuAngularVelocitySample sample{};
    CHECK(decoder.try_pop(&sample));
    CHECK(sample.timestamp_ns == 98'000'000LL);
    CHECK(!decoder.try_pop(&sample));

    auto bad_crc_chunk = chunk;
    bad_crc_chunk.bytes[2] ^= 0x01;
    mosas::app::ImuStreamDecoder crc_decoder(0, 0, 2);
    crc_decoder.consume(bad_crc_chunk);
    CHECK(!crc_decoder.try_pop(&sample));
    CHECK(crc_decoder.parse_error_count() == 1);

    mosas::app::ImuStreamDecoder underflow_decoder(0, 2, 2);
    chunk.first_byte_timestamp_ns = 1;
    underflow_decoder.consume(chunk);
    CHECK(!underflow_decoder.try_pop(&sample));
    CHECK(underflow_decoder.parse_error_count() == 1);
    return 0;
}
