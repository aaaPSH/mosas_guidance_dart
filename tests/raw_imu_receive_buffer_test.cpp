#include <mosas/adapters/raw_imu_receive_buffer.hpp>

#include "test_support.hpp"

#include <array>
#include <cstdint>
#include <future>

namespace {
using mosas::app::RawImuReceiveBuffer;
using mosas::runtime::RawSerialChunk;
constexpr std::array<std::uint8_t, 3> kBytes{0x55, 0xAA, 0x12};

/** @brief 验证关闭后晚到的读取回调不改变已接收数据及统计。 @return 通过时返回 0。 */
int late_callback_test() {
    RawImuReceiveBuffer buffer(1);
    buffer.receive(kBytes.data(), kBytes.size(), 100);

    // 模拟已发起的串口读取，强制其回调在取消并关闭缓冲区后才交付。
    std::promise<void> deliver;
    auto ready = deliver.get_future();
    auto callback = std::async(std::launch::async, [&] {
        ready.wait();
        RawImuReceiveBuffer::receive_callback(&buffer, kBytes.data(), kBytes.size(), 200);
        RawImuReceiveBuffer::receive_callback(&buffer, nullptr, 0, -1);
    });
    buffer.close();
    buffer.close();
    deliver.set_value();
    callback.get();

    CHECK(buffer.received_chunk_count() == 1);
    CHECK(buffer.dropped_chunk_count() == 0);
    CHECK(buffer.queued_chunk_count() == 1);
    CHECK(!buffer.wait_for_data());
    RawSerialChunk chunk;
    CHECK(buffer.try_pop(&chunk));
    CHECK(chunk.sequence == 0);
    CHECK(chunk.first_byte_timestamp_ns == 100);
    CHECK(chunk.size == kBytes.size());
    for (std::size_t index = 0; index < kBytes.size(); ++index) {
        CHECK(chunk.bytes[index] == kBytes[index]);
    }
    CHECK(!buffer.try_pop(&chunk));
    return 0;
}

/** @brief 验证关闭不能抹去真实溢出计数及序号缺口。 @return 通过时返回 0。 */
int overflow_test() {
    RawImuReceiveBuffer buffer(1);
    buffer.receive(kBytes.data(), kBytes.size(), 100);
    buffer.receive(kBytes.data(), kBytes.size(), 200);
    CHECK(buffer.received_chunk_count() == 2);
    CHECK(buffer.dropped_chunk_count() == 1);
    RawSerialChunk chunk;
    CHECK(buffer.try_pop(&chunk));
    CHECK(chunk.sequence == 0);

    buffer.receive(kBytes.data(), kBytes.size(), 300);
    buffer.close();
    buffer.receive(kBytes.data(), kBytes.size(), 400);
    CHECK(buffer.received_chunk_count() == 3);
    CHECK(buffer.dropped_chunk_count() == 1);
    CHECK(buffer.try_pop(&chunk));
    CHECK(chunk.sequence == 2);
    CHECK(chunk.first_byte_timestamp_ns == 300);
    CHECK(!buffer.try_pop(&chunk));
    return 0;
}

/** @brief 拒绝采集期间的无效块，不将停机后的回调误算为错误。 @return 通过时返回 0。 */
int invalid_chunk_test() {
    RawImuReceiveBuffer buffer(1);
    std::array<std::uint8_t, RawSerialChunk::kMaxChunkBytes + 1> oversized{};
    buffer.receive(nullptr, 1, 100);
    buffer.receive(kBytes.data(), 0, 100);
    buffer.receive(oversized.data(), oversized.size(), 100);
    buffer.receive(kBytes.data(), kBytes.size(), -1);
    CHECK(buffer.received_chunk_count() == 4);
    CHECK(buffer.dropped_chunk_count() == 4);
    CHECK(buffer.queued_chunk_count() == 0);

    buffer.receive(kBytes.data(), kBytes.size(), 200);
    buffer.close();
    buffer.receive(nullptr, 0, -1);
    CHECK(buffer.received_chunk_count() == 5);
    CHECK(buffer.dropped_chunk_count() == 4);
    RawSerialChunk chunk;
    CHECK(buffer.try_pop(&chunk));
    CHECK(chunk.sequence == 0);
    CHECK(!buffer.try_pop(&chunk));
    return 0;
}

/** @brief 将已停止的缓冲区重置为计数清零、序号从零开始的新会话。 @return 通过时返回 0。 */
int reset_test() {
    RawImuReceiveBuffer buffer(1);
    buffer.receive(kBytes.data(), kBytes.size(), 100);
    buffer.receive(kBytes.data(), kBytes.size(), 200);
    buffer.close();
    buffer.reset();
    CHECK(buffer.received_chunk_count() == 0);
    CHECK(buffer.dropped_chunk_count() == 0);
    CHECK(buffer.queued_chunk_count() == 0);

    buffer.receive(kBytes.data(), kBytes.size(), 300);
    CHECK(buffer.wait_for_data());
    CHECK(buffer.received_chunk_count() == 1);
    CHECK(buffer.dropped_chunk_count() == 0);
    RawSerialChunk chunk;
    CHECK(buffer.try_pop(&chunk));
    CHECK(chunk.sequence == 0);
    CHECK(chunk.first_byte_timestamp_ns == 300);
    buffer.close();
    CHECK(!buffer.wait_for_data());
    return 0;
}
}  // namespace

/** @brief 运行无硬件依赖的接收及停机回归测试。 @return 通过时返回 0。 */
int main() {
    if (late_callback_test() || overflow_test() || invalid_chunk_test() || reset_test()) return 1;
    return 0;
}
