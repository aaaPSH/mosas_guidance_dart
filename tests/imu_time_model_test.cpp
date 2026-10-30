#include <mosas/adapters/imu_stream_decoder.hpp>
#include <mosas/domain/imu_time_model.hpp>
#include "imu_frame_fixture.hpp"
#include "test_support.hpp"
#include <algorithm>
#include <vector>

namespace {
/** @brief Feed a chunk with explicitly controlled reception time and sequence. */
void feed(mosas::app::ImuStreamDecoder& decoder, const std::vector<std::uint8_t>& bytes,
          std::uint64_t sequence, std::int64_t time) {
    mosas::runtime::RawSerialChunk chunk{};
    chunk.first_byte_timestamp_ns = time;
    chunk.sequence = sequence;
    chunk.size = static_cast<std::uint16_t>(bytes.size());
    std::copy(bytes.begin(), bytes.end(), chunk.bytes.begin());
    decoder.consume(chunk, time);
}
}
int main() {
    using namespace mosas;
    const auto estimate = domain::estimate_imu_sample_time(100'000'000, 2'000'000);
    CHECK(estimate.monotonic_ns == 98'000'000);
    CHECK(!domain::estimate_imu_sample_time(1, 2).is_valid());
    domain::SequenceTimeConfig timing{1000, 0, 0, 50'000'000, 1'000'000'000};
    const std::vector<std::uint8_t> valid(test::kValidImuFrame.begin(), test::kValidImuFrame.end());
    auto bad = valid; bad[2] ^= 1;
    auto sync = std::make_shared<domain::AcquisitionSync>(2);
    app::ImuStreamDecoder decoder(0, 2'000'000, 2, timing);
    decoder.bind_sync(sync); decoder.reset(100'000'000);
    feed(decoder, valid, 0, 100'000'000);
    runtime::ImuAngularVelocitySample sample;
    CHECK(decoder.try_pop(&sample)); CHECK(sample.timestamp_ns == 98'000'000);
    CHECK(sample.sample_sequence == 0 && sample.sensor_time.has_value());
    CHECK(sync->latch_launch(0));
    feed(decoder, bad, 1, 101'300'000);
    CHECK(!decoder.try_pop(&sample));
    feed(decoder, valid, 2, 103'000'000);
    CHECK(decoder.try_pop(&sample));
    CHECK(sample.timestamp_ns == 100'000'000 && sample.sample_sequence == 2);
    CHECK(decoder.crc_error_count() == 1);
    CHECK(decoder.time_diagnostics().known_missing == 1);

    // Unknown byte loss after launch does not guess slots or change the baseline.
    feed(decoder, {1, 2, 3, 4}, 3, 104'000'000);
    feed(decoder, valid, 4, 180'000'000);
    CHECK(decoder.try_pop(&sample));
    CHECK(sample.timestamp_ns == 101'000'000 && sample.sample_sequence == 3);
    CHECK(decoder.time_diagnostics().unknown_discontinuities == 1);
    CHECK(decoder.time_diagnostics().baselines == 1);

    // Decoded queue overflow consumes sample slots, including frames in a glued chunk.
    std::vector<std::uint8_t> glued;
    for (int i = 0; i < 4; ++i) glued.insert(glued.end(), valid.begin(), valid.end());
    feed(decoder, glued, 5, 181'000'000);
    CHECK(decoder.dropped_sample_count() > 0);
    while (decoder.try_pop(&sample)) {}
    feed(decoder, valid, 6, 182'000'000);
    CHECK(decoder.try_pop(&sample)); CHECK(sample.sample_sequence == 8);
    CHECK(sample.timestamp_ns == 106'000'000);

    app::ImuStreamDecoder split(0, 0, 4, timing); split.reset(100'000'000);
    feed(split, std::vector<std::uint8_t>(valid.begin(), valid.begin() + 7), 0, 100'000'000);
    CHECK(!split.try_pop(&sample));
    feed(split, std::vector<std::uint8_t>(valid.begin() + 7, valid.end()), 1, 100'100'000);
    CHECK(split.try_pop(&sample)); CHECK(sample.timestamp_ns == 100'000'000);
    feed(split, valid, 3, 105'000'000); // Lost raw chunk: re-anchor only before launch.
    CHECK(split.try_pop(&sample)); CHECK(sample.timestamp_ns == 105'000'000);
    CHECK(sample.acquisition_epoch == 1);
    CHECK(split.sequence_gap_count() == 1);
    split.reset(200'000'000);
    CHECK(split.time_diagnostics().baselines == 0);
    feed(split, valid, 0, 200'000'000);
    CHECK(split.try_pop(&sample)); CHECK(sample.sample_sequence == 0);
    return 0;
}
