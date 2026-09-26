#include <mosas/runtime/imu_angular_velocity_history.hpp>

#include <cmath>
#include <cstring>

namespace mosas::runtime {
namespace {

bool finite_vector(const Vector3& value) noexcept {
    return std::isfinite(value.x) && std::isfinite(value.y) &&
           std::isfinite(value.z);
}

std::uint64_t double_bits(double value) noexcept {
    std::uint64_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    return bits;
}

double bits_double(std::uint64_t bits) noexcept {
    double value = 0.0;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

}  // namespace

bool ImuAngularVelocityHistory::push(
    const ImuAngularVelocitySample& sample) noexcept {
    if (sample.timestamp_ns < 0 || !finite_vector(sample.acceleration) ||
        !finite_vector(sample.angular_velocity) ||
        !std::isfinite(sample.servo_control_p_normalized) ||
        !std::isfinite(sample.servo_control_y_normalized) ||
        (has_timestamp_ && sample.timestamp_ns <= last_timestamp_ns_)) {
        return false;
    }

    Slot& slot = slots_[next_index_];
    const std::uint64_t sequence =
        slot.sequence.load(std::memory_order_relaxed);
    slot.sequence.store(sequence + 1, std::memory_order_release);
    slot.timestamp_ns.store(sample.timestamp_ns, std::memory_order_relaxed);
    slot.acceleration_x_bits.store(double_bits(sample.acceleration.x),
                                   std::memory_order_relaxed);
    slot.acceleration_y_bits.store(double_bits(sample.acceleration.y),
                                   std::memory_order_relaxed);
    slot.acceleration_z_bits.store(double_bits(sample.acceleration.z),
                                   std::memory_order_relaxed);
    slot.angular_velocity_x_bits.store(
        double_bits(sample.angular_velocity.x), std::memory_order_relaxed);
    slot.angular_velocity_y_bits.store(
        double_bits(sample.angular_velocity.y), std::memory_order_relaxed);
    slot.angular_velocity_z_bits.store(
        double_bits(sample.angular_velocity.z), std::memory_order_relaxed);
    slot.servo_control_p_bits.store(
        double_bits(sample.servo_control_p_normalized),
        std::memory_order_relaxed);
    slot.servo_control_y_bits.store(
        double_bits(sample.servo_control_y_normalized),
        std::memory_order_relaxed);
    slot.sequence.store(sequence + 2, std::memory_order_release);

    next_index_ = (next_index_ + 1) % capacity_;
    if (size_writer_ < capacity_) {
        ++size_writer_;
        size_.store(size_writer_, std::memory_order_release);
    }
    has_timestamp_ = true;
    last_timestamp_ns_ = sample.timestamp_ns;
    return true;
}

std::optional<ImuAngularVelocityLookup>
ImuAngularVelocityHistory::find_interpolated(
    TimestampNs timestamp_ns) const noexcept {
    if (timestamp_ns < 0 || size_.load(std::memory_order_acquire) == 0) {
        return std::nullopt;
    }

    ImuAngularVelocitySample lower{};
    ImuAngularVelocitySample upper{};
    bool has_lower = false;
    bool has_upper = false;
    for (std::size_t index = 0; index < capacity_; ++index) {
        ImuAngularVelocitySample candidate{};
        if (!read_slot(index, &candidate)) {
            continue;
        }
        if (candidate.timestamp_ns == timestamp_ns) {
            return ImuAngularVelocityLookup{
                candidate, false, candidate.timestamp_ns,
                candidate.timestamp_ns};
        }
        if (candidate.timestamp_ns < timestamp_ns &&
            (!has_lower || candidate.timestamp_ns > lower.timestamp_ns)) {
            lower = candidate;
            has_lower = true;
        }
        if (candidate.timestamp_ns > timestamp_ns &&
            (!has_upper || candidate.timestamp_ns < upper.timestamp_ns)) {
            upper = candidate;
            has_upper = true;
        }
    }

    if (!has_lower || !has_upper || upper.timestamp_ns <= lower.timestamp_ns) {
        return std::nullopt;
    }

    const double fraction = static_cast<double>(timestamp_ns -
                                                lower.timestamp_ns) /
                            static_cast<double>(upper.timestamp_ns -
                                                lower.timestamp_ns);
    if (!std::isfinite(fraction) || fraction < 0.0 || fraction > 1.0) {
        return std::nullopt;
    }
    const auto interpolate = [fraction](double lhs, double rhs) {
        return lhs + (rhs - lhs) * fraction;
    };
    const Vector3 acceleration{
        interpolate(lower.acceleration.x, upper.acceleration.x),
        interpolate(lower.acceleration.y, upper.acceleration.y),
        interpolate(lower.acceleration.z, upper.acceleration.z)};
    const Vector3 angular_velocity{
        interpolate(lower.angular_velocity.x, upper.angular_velocity.x),
        interpolate(lower.angular_velocity.y, upper.angular_velocity.y),
        interpolate(lower.angular_velocity.z, upper.angular_velocity.z)};
    const double servo_control_p_normalized =
        interpolate(lower.servo_control_p_normalized,
                    upper.servo_control_p_normalized);
    const double servo_control_y_normalized =
        interpolate(lower.servo_control_y_normalized,
                    upper.servo_control_y_normalized);
    if (!finite_vector(acceleration) || !finite_vector(angular_velocity) ||
        !std::isfinite(servo_control_p_normalized) ||
        !std::isfinite(servo_control_y_normalized)) {
        return std::nullopt;
    }
    return ImuAngularVelocityLookup{
        {timestamp_ns, acceleration, angular_velocity,
         servo_control_p_normalized, servo_control_y_normalized}, true,
        lower.timestamp_ns, upper.timestamp_ns};
}

void ImuAngularVelocityHistory::reset() noexcept {
    for (std::size_t index = 0; index < capacity_; ++index) {
        slots_[index].sequence.store(0, std::memory_order_relaxed);
        slots_[index].timestamp_ns.store(-1, std::memory_order_relaxed);
        slots_[index].acceleration_x_bits.store(0,
                                                std::memory_order_relaxed);
        slots_[index].acceleration_y_bits.store(0,
                                                std::memory_order_relaxed);
        slots_[index].acceleration_z_bits.store(0,
                                                std::memory_order_relaxed);
        slots_[index].angular_velocity_x_bits.store(0,
                                                     std::memory_order_relaxed);
        slots_[index].angular_velocity_y_bits.store(0,
                                                     std::memory_order_relaxed);
        slots_[index].angular_velocity_z_bits.store(0,
                                                     std::memory_order_relaxed);
        slots_[index].servo_control_p_bits.store(0,
                                                 std::memory_order_relaxed);
        slots_[index].servo_control_y_bits.store(0,
                                                 std::memory_order_relaxed);
    }
    next_index_ = 0;
    size_writer_ = 0;
    last_timestamp_ns_ = -1;
    has_timestamp_ = false;
    size_.store(0, std::memory_order_release);
}

bool ImuAngularVelocityHistory::read_slot(
    std::size_t index, ImuAngularVelocitySample* sample) const noexcept {
    if (sample == nullptr) {
        return false;
    }
    const Slot& slot = slots_[index];
    const std::uint64_t first = slot.sequence.load(std::memory_order_acquire);
    if (first == 0 || (first & 1U) != 0) {
        return false;
    }
    sample->timestamp_ns =
        slot.timestamp_ns.load(std::memory_order_relaxed);
    sample->acceleration.x = bits_double(
        slot.acceleration_x_bits.load(std::memory_order_relaxed));
    sample->acceleration.y = bits_double(
        slot.acceleration_y_bits.load(std::memory_order_relaxed));
    sample->acceleration.z = bits_double(
        slot.acceleration_z_bits.load(std::memory_order_relaxed));
    sample->angular_velocity.x = bits_double(
        slot.angular_velocity_x_bits.load(std::memory_order_relaxed));
    sample->angular_velocity.y = bits_double(
        slot.angular_velocity_y_bits.load(std::memory_order_relaxed));
    sample->angular_velocity.z = bits_double(
        slot.angular_velocity_z_bits.load(std::memory_order_relaxed));
    sample->servo_control_p_normalized = bits_double(
        slot.servo_control_p_bits.load(std::memory_order_relaxed));
    sample->servo_control_y_normalized = bits_double(
        slot.servo_control_y_bits.load(std::memory_order_relaxed));
    const std::uint64_t second = slot.sequence.load(std::memory_order_acquire);
    return first == second && (second & 1U) == 0;
}

}  // namespace mosas::runtime
