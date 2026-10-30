#include <mosas/adapters/imu_stream_decoder.hpp>

#include <mosas/domain/imu_time_model.hpp>

#include <serial_package/serial_protocol.hpp>

#include <algorithm>
#include <array>
#include <cstring>
#include <limits>

namespace mosas::app {
namespace {

runtime::TimestampNs saturating_add(runtime::TimestampNs left,
                                    runtime::TimestampNs right) noexcept {
    if (right > 0 &&
        left > std::numeric_limits<runtime::TimestampNs>::max() - right) {
        return std::numeric_limits<runtime::TimestampNs>::max();
    }
    if (right < 0 &&
        left < std::numeric_limits<runtime::TimestampNs>::min() - right) {
        return std::numeric_limits<runtime::TimestampNs>::min();
    }
    return left + right;
}

}  // namespace

ImuStreamDecoder::ImuStreamDecoder(
    runtime::TimestampNs wire_byte_period_ns,
    runtime::TimestampNs measurement_latency_ns, std::size_t output_capacity,
    domain::SequenceTimeConfig timing)
    : wire_byte_period_ns_(wire_byte_period_ns),
      measurement_latency_ns_(measurement_latency_ns),
      output_queue_(output_capacity), timeline_(timing) {}

void ImuStreamDecoder::reset(runtime::TimestampNs now_ns) noexcept {
    clear_buffer();
    output_queue_.reset();
    has_sequence_ = false;
    expected_sequence_ = 0;
    sample_sequence_ = pending_bad_frames_ = 0;
    boundary_trusted_ = seeking_ = false;
    timeline_.start(now_ns - measurement_latency_ns_);
    parse_error_count_.store(0, std::memory_order_relaxed);
    sequence_gap_count_.store(0, std::memory_order_relaxed);
    crc_error_count_.store(0, std::memory_order_relaxed);
    dropped_sample_count_.store(0, std::memory_order_relaxed);
}

void ImuStreamDecoder::consume(const runtime::RawSerialChunk& chunk, runtime::TimestampNs now_ns) noexcept {
    if (chunk.size == 0 || chunk.size > chunk.bytes.size() ||
        chunk.first_byte_timestamp_ns < 0) {
        parse_error_count_.fetch_add(1, std::memory_order_relaxed);
        return;
    }

    if (!has_sequence_) {
        has_sequence_ = true;
    } else if (chunk.sequence != expected_sequence_) {
        sequence_gap_count_.fetch_add(1, std::memory_order_relaxed);
        lose_alignment();
        clear_buffer();
    }
    expected_sequence_ = chunk.sequence + 1;

    if (!append(chunk)) {
        parse_error_count_.fetch_add(1, std::memory_order_relaxed);
        lose_alignment();
        clear_buffer();
        return;
    }

    while (buffered_size_ >= serial_package::kImuFrameSize) {
        align_buffer();
        if (buffered_size_ < serial_package::kImuFrameSize) {
            return;
        }

        serial_package::ImuFrameValues values{};
        if (!serial_package::decode_imu_frame(
                buffer_.data(), serial_package::kImuFrameSize, &values,
                nullptr)) {
            parse_error_count_.fetch_add(1, std::memory_order_relaxed);
            const bool intact_boundary = buffer_[18] == 0x0D && buffer_[19] == 0x0A;
            if (intact_boundary) crc_error_count_.fetch_add(1, std::memory_order_relaxed);
            if (boundary_trusted_ && intact_boundary) {
                // Keep only a count until a subsequent valid frame confirms the fixed-size boundary.
                ++pending_bad_frames_;
                discard_prefix(serial_package::kImuFrameSize);
            } else {
                lose_alignment();
                discard_prefix(1);
            }
            continue;
        }
        if (pending_bad_frames_) {
            timeline_.missing(pending_bad_frames_);
            sample_sequence_ += pending_bad_frames_;
            pending_bad_frames_ = 0;
        }
        boundary_trusted_ = true;
        seeking_ = false;
        const auto receive = domain::estimate_imu_sample_time(
            buffer_first_timestamp_ns_, measurement_latency_ns_);
        const auto stamp = timeline_.observe(sample_sequence_++, receive.monotonic_ns,
            receive.monotonic_ns, now_ns - measurement_latency_ns_);
        if (!stamp) {
            if (receive.monotonic_ns < 0) parse_error_count_.fetch_add(1, std::memory_order_relaxed);
            discard_prefix(serial_package::kImuFrameSize);
            continue;
        }
        runtime::ImuAngularVelocitySample sample{
            stamp->timestamp_ns,
            {values.acceleration_x_mps2, values.acceleration_y_mps2,
             values.acceleration_z_mps2},
            {values.angular_velocity_x_rad_s,
             values.angular_velocity_y_rad_s,
             values.angular_velocity_z_rad_s},
            values.servo_control_p_normalized,
            values.servo_control_y_normalized};
        sample.sensor_time = receive;
        sample.sensor_time->source = domain::TimestampSource::sequence_fixed_period;
        sample.sensor_time->source_time_ns = receive.monotonic_ns;
        sample.sensor_time->monotonic_ns = stamp->timestamp_ns;
        sample.acquisition_epoch = stamp->epoch;
        sample.sample_sequence = sample_sequence_ - 1;
        if (!publish(sample)) {
            dropped_sample_count_.fetch_add(1, std::memory_order_relaxed);
        }
        discard_prefix(serial_package::kImuFrameSize);
    }
}

bool ImuStreamDecoder::try_pop(
    runtime::ImuAngularVelocitySample* sample) noexcept {
    return output_queue_.try_pop(sample);
}

void ImuStreamDecoder::clear_buffer() noexcept {
    buffered_size_ = 0;
    buffer_first_timestamp_ns_ = -1;
}

void ImuStreamDecoder::discard_prefix(std::size_t count) noexcept {
    if (count == 0) {
        return;
    }
    if (count > buffered_size_) {
        clear_buffer();
        return;
    }

    const std::size_t remaining = buffered_size_ - count;
    if (remaining != 0) {
        std::memmove(buffer_.data(), buffer_.data() + count, remaining);
        const runtime::TimestampNs elapsed =
            wire_byte_period_ns_ > 0 &&
                    count <= static_cast<std::size_t>(
                                 std::numeric_limits<runtime::TimestampNs>::max() /
                                 wire_byte_period_ns_)
                ? wire_byte_period_ns_ * static_cast<runtime::TimestampNs>(count)
                : std::numeric_limits<runtime::TimestampNs>::max();
        buffer_first_timestamp_ns_ =
            saturating_add(buffer_first_timestamp_ns_, elapsed);
    } else {
        buffer_first_timestamp_ns_ = -1;
    }
    buffered_size_ = remaining;
}

void ImuStreamDecoder::align_buffer() noexcept {
    if (buffered_size_ == 0) {
        return;
    }

    std::size_t header_offset = buffered_size_;
    for (std::size_t index = 0; index + 1 < buffered_size_; ++index) {
        if (buffer_[index] == 0x55 && buffer_[index + 1] == 0xAA) {
            header_offset = index;
            break;
        }
    }
    if (header_offset == 0) {
        return;
    }
    if (header_offset < buffered_size_) {
        lose_alignment();
        discard_prefix(header_offset);
        return;
    }

    const std::size_t keep = buffer_[buffered_size_ - 1] == 0x55 ? 1 : 0;
    if (buffered_size_ > keep) {
        lose_alignment();
        discard_prefix(buffered_size_ - keep);
    }
}

void ImuStreamDecoder::lose_alignment() noexcept {
    if (!seeking_) timeline_.discontinuity();
    seeking_ = true;
    boundary_trusted_ = false;
    pending_bad_frames_ = 0;
}

bool ImuStreamDecoder::append(const runtime::RawSerialChunk& chunk) noexcept {
    if (buffered_size_ == 0) {
        buffer_first_timestamp_ns_ = chunk.first_byte_timestamp_ns;
    }
    if (chunk.size > kRawBufferSize - buffered_size_) {
        return false;
    }
    std::memcpy(buffer_.data() + buffered_size_, chunk.bytes.data(), chunk.size);
    buffered_size_ += chunk.size;
    return true;
}

bool ImuStreamDecoder::publish(
    const runtime::ImuAngularVelocitySample& sample) noexcept {
    return output_queue_.try_push(sample);
}

}  // namespace mosas::app
