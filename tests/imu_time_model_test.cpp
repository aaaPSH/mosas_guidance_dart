#include <mosas/adapters/imu_stream_decoder.hpp>
#include <mosas/domain/imu_time_model.hpp>
#include "imu_frame_fixture.hpp"
#include "test_support.hpp"
#include <algorithm>
#include <vector>

namespace {
using namespace mosas;
const domain::SequenceTimeConfig kTiming{1000, 0, 0, 50'000'000, 5'000'000};

/** @brief 将指定设备序号编码为测试字节流。 @param index 设备帧号。 @return 完整有效帧。 */
std::vector<std::uint8_t> frame(std::uint32_t index) {
    const auto bytes = test::make_imu_frame(index);
    return {bytes.begin(), bytes.end()};
}

/**
 * @brief 注入一个接收块，独立控制块序号和接收时刻。
 * @param decoder 被测解码器。
 * @param bytes 接收字节。
 * @param sequence 原始块序号，不是 IMU 设备帧号。
 * @param time 首字节接收时刻及消费时刻，单位 ns。
 */
void feed(app::ImuStreamDecoder& decoder, const std::vector<std::uint8_t>& bytes,
          std::uint64_t sequence, std::int64_t time) {
    runtime::RawSerialChunk chunk{};
    chunk.first_byte_timestamp_ns = time;
    chunk.sequence = sequence;
    chunk.size = static_cast<std::uint16_t>(bytes.size());
    std::copy(bytes.begin(), bytes.end(), chunk.bytes.begin());
    decoder.consume(chunk, time);
}

/** @brief 验证窗口末帧接收延迟增大后回落不会推晚基线，发射后仍保持固定周期。 @return 通过时为 0。 */
int check_stable_window_delay() {
    auto sync = std::make_shared<domain::AcquisitionSync>(2);
    app::ImuStreamDecoder decoder(0, 2'000'000, 2,
        {200, 0, 10'000'000, 50'000'000, 1'000'000'000});
    decoder.bind_sync(sync);
    decoder.reset(100'000'000);
    runtime::ImuAngularVelocitySample sample;
    feed(decoder, frame(100), 0, 100'000'000);
    CHECK(!decoder.try_pop(&sample));
    feed(decoder, frame(101), 1, 105'500'000);
    CHECK(!decoder.try_pop(&sample));
    feed(decoder, frame(102), 2, 114'000'000);
    CHECK(decoder.try_pop(&sample));
    CHECK(sample.timestamp_ns == 108'000'000 && sample.sample_sequence == 2);
    CHECK(decoder.time_diagnostics().baseline_ns == 98'000'000);
    CHECK(decoder.time_diagnostics().warmup_discarded == 2);
    CHECK(sync->latch_launch(0));

    // 末帧额外延迟 4 ms，下一帧仅延迟 1 ms；旧基线会生成晚于主机接收的 117 ms。
    feed(decoder, frame(103), 3, 116'000'000);
    CHECK(decoder.try_pop(&sample));
    CHECK(sample.timestamp_ns <= 116'000'000);
    CHECK(sample.timestamp_ns == 113'000'000 && sample.acquisition_epoch == 0);
    CHECK(sample.sensor_time->source_time_ns == 114'000'000);
    feed(decoder, frame(104), 4, 260'000'000);
    CHECK(decoder.try_pop(&sample));
    CHECK(sample.timestamp_ns == 118'000'000);
    feed(decoder, frame(107), 5, 270'000'000);
    CHECK(decoder.try_pop(&sample));
    CHECK(sample.timestamp_ns == 133'000'000 && sample.sample_sequence == 7);
    CHECK(decoder.time_diagnostics().known_missing == 2);
    CHECK(decoder.time_diagnostics().baselines == 1);
    CHECK(decoder.time_diagnostics().post_launch_anomalies > 0);
    return 0;
}

/** @brief 验证设备序号驱动丢帧、固定周期和异常拒绝，不信任坏帧计数器。 @return 通过时为 0。 */
int check_device_timeline() {
    auto sync = std::make_shared<domain::AcquisitionSync>(2);
    app::ImuStreamDecoder decoder(0, 2'000'000, 2, kTiming);
    decoder.bind_sync(sync);
    decoder.reset(100'000'000);
    feed(decoder, frame(100), 0, 100'000'000);
    runtime::ImuAngularVelocitySample sample;
    CHECK(decoder.try_pop(&sample));
    CHECK(sample.timestamp_ns == 98'000'000 && sample.sample_sequence == 0);
    CHECK(sample.sensor_time.has_value());
    CHECK(sample.sensor_time->source == domain::TimestampSource::sequence_fixed_period);
    CHECK(sample.sensor_time->source_time_ns == 98'000'000);
    CHECK(sample.imu_index == 100 && sample.trigger_flag == 1);
    CHECK_NEAR(sample.outer_loop_y_rad_s, -1.234, 1e-12);
    CHECK_NEAR(sample.outer_loop_p_rad_s, 2.345, 1e-12);
    CHECK(sync->latch_launch(0));

    auto bad = frame(101);
    bad[23] ^= 0x40; // 损坏序号而不更新 CRC，不能污染后续解回绕状态。
    feed(decoder, bad, 1, 101'300'000);
    CHECK(!decoder.try_pop(&sample));
    feed(decoder, frame(104), 2, 103'000'000);
    CHECK(decoder.try_pop(&sample));
    CHECK(sample.timestamp_ns == 102'000'000 && sample.sample_sequence == 4);
    CHECK(decoder.crc_error_count() == 1);
    CHECK(decoder.time_diagnostics().known_missing == 3);

    feed(decoder, frame(104), 3, 104'000'000);
    feed(decoder, frame(103), 4, 105'000'000);
    CHECK(!decoder.try_pop(&sample));
    CHECK(decoder.time_diagnostics().sequence_errors == 2);

    // 字节失步及接收抖动不压缩设备时间轴，也不触发额外的丢帧猜测或换基线。
    feed(decoder, {1, 2, 3, 4}, 5, 106'000'000);
    feed(decoder, frame(105), 6, 180'000'000);
    CHECK(decoder.try_pop(&sample));
    CHECK(sample.timestamp_ns == 103'000'000 && sample.sample_sequence == 5);
    CHECK(decoder.time_diagnostics().unknown_discontinuities == 0);
    CHECK(decoder.time_diagnostics().known_missing == 3);
    CHECK(decoder.time_diagnostics().baselines == 1);

    std::vector<std::uint8_t> glued;
    for (unsigned index = 106; index <= 109; ++index) {
        const auto bytes = frame(index);
        glued.insert(glued.end(), bytes.begin(), bytes.end());
    }
    feed(decoder, glued, 7, 181'000'000);
    CHECK(decoder.dropped_sample_count() == 2);
    while (decoder.try_pop(&sample)) {}
    feed(decoder, frame(110), 8, 182'000'000);
    CHECK(decoder.try_pop(&sample));
    CHECK(sample.timestamp_ns == 108'000'000 && sample.sample_sequence == 10);
    CHECK(decoder.time_diagnostics().known_missing == 3);
    return 0;
}

/** @brief 验证拆包和接收缺块后仍按设备序号恢复，不在发射前重新锚定。 @return 通过时为 0。 */
int check_chunk_recovery() {
    app::ImuStreamDecoder decoder(0, 0, 4, kTiming);
    decoder.reset(100'000'000);
    const auto valid = frame(500);
    feed(decoder, {valid.begin(), valid.begin() + 7}, 0, 100'000'000);
    runtime::ImuAngularVelocitySample sample;
    CHECK(!decoder.try_pop(&sample));
    feed(decoder, {valid.begin() + 7, valid.end()}, 1, 100'100'000);
    CHECK(decoder.try_pop(&sample));
    CHECK(sample.timestamp_ns == 100'000'000);
    feed(decoder, frame(504), 3, 105'000'000);
    CHECK(decoder.try_pop(&sample));
    CHECK(sample.timestamp_ns == 104'000'000 && sample.sample_sequence == 4);
    CHECK(sample.acquisition_epoch == 0);
    CHECK(decoder.sequence_gap_count() == 1);
    CHECK(decoder.time_diagnostics().known_missing == 3);
    CHECK(decoder.time_diagnostics().baselines == 1);

    decoder.reset(200'000'000);
    CHECK(decoder.time_diagnostics().baselines == 0);
    CHECK(decoder.time_diagnostics().known_missing == 0);
    feed(decoder, frame(7), 0, 200'000'000);
    CHECK(decoder.try_pop(&sample));
    CHECK(sample.sample_sequence == 0 && sample.imu_index == 7);
    return 0;
}

/** @brief 验证 uint32 回绕、跨回绕丢帧及按设备序号统计频率。 @return 通过时为 0。 */
int check_wrap_and_frequency() {
    app::ImuStreamDecoder decoder(0, 0, 4, kTiming);
    decoder.reset(100'000'000);
    runtime::ImuAngularVelocitySample sample;
    feed(decoder, frame(UINT32_MAX - 1), 0, 100'000'000);
    CHECK(decoder.try_pop(&sample));
    feed(decoder, frame(1), 1, 103'000'000);
    CHECK(decoder.try_pop(&sample));
    CHECK(sample.timestamp_ns == 103'000'000 && sample.sample_sequence == 3);
    CHECK(sample.imu_index == 1);
    CHECK(decoder.time_diagnostics().known_missing == 2);
    feed(decoder, frame(2), 2, 104'000'000);
    CHECK(decoder.try_pop(&sample));
    CHECK(sample.timestamp_ns == 104'000'000 && sample.sample_sequence == 4);

    decoder.reset(200'000'000);
    for (unsigned i = 0; i <= 10; ++i) {
        feed(decoder, frame(900 + i), i, 200'000'000 + i * 1'000'000);
        CHECK(decoder.try_pop(&sample));
    }
    const auto diagnostics = decoder.time_diagnostics();
    CHECK(diagnostics.frequency_windows == 2);
    CHECK_NEAR(diagnostics.estimated_hz, 1000, 1e-12);
    CHECK(diagnostics.sequence_errors == 0);
    return 0;
}
}  // namespace

int main() {
    const auto estimate = mosas::domain::estimate_imu_sample_time(100'000'000, 2'000'000);
    CHECK(estimate.monotonic_ns == 98'000'000);
    CHECK(!mosas::domain::estimate_imu_sample_time(1, 2).is_valid());
    CHECK(check_stable_window_delay() == 0);
    CHECK(check_device_timeline() == 0);
    CHECK(check_chunk_recovery() == 0);
    CHECK(check_wrap_and_frequency() == 0);
    return 0;
}
