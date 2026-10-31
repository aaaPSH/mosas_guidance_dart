#include <mosas/runtime/attitude_history.hpp>
#include <mosas/runtime/vector3.hpp>

#include <cmath>
#include <cstring>

namespace mosas::runtime {
namespace {

bool finite_rotation(const RotationMatrix3& value) noexcept {
    const Vector3 column_0{value.m00, value.m10, value.m20};
    const Vector3 column_1{value.m01, value.m11, value.m21};
    const Vector3 column_2{value.m02, value.m12, value.m22};
    const auto finite_vector = [](const Vector3& vector) {
        return std::isfinite(vector.x) && std::isfinite(vector.y) &&
               std::isfinite(vector.z);
    };
    const auto dot = [](const Vector3& lhs, const Vector3& rhs) {
        return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
    };
    const auto norm = [](const Vector3& vector) {
        return std::hypot(std::hypot(vector.x, vector.y), vector.z);
    };
    const double determinant =
        value.m00 * (value.m11 * value.m22 - value.m12 * value.m21) -
        value.m01 * (value.m10 * value.m22 - value.m12 * value.m20) +
        value.m02 * (value.m10 * value.m21 - value.m11 * value.m20);
    return finite_vector(column_0) && finite_vector(column_1) &&
           finite_vector(column_2) && std::abs(norm(column_0) - 1.0) <= 1e-6 &&
           std::abs(norm(column_1) - 1.0) <= 1e-6 &&
           std::abs(norm(column_2) - 1.0) <= 1e-6 &&
           std::abs(dot(column_0, column_1)) <= 1e-6 &&
           std::abs(dot(column_0, column_2)) <= 1e-6 &&
           std::abs(dot(column_1, column_2)) <= 1e-6 &&
           std::abs(determinant - 1.0) <= 1e-6;
}

Vector3 cross_product(const Vector3& lhs, const Vector3& rhs) noexcept {
    return {lhs.y * rhs.z - lhs.z * rhs.y,
            lhs.z * rhs.x - lhs.x * rhs.z,
            lhs.x * rhs.y - lhs.y * rhs.x};
}

bool normalize_vector(Vector3* vector) noexcept {
    if (vector == nullptr) {
        return false;
    }
    const double norm = std::hypot(std::hypot(vector->x, vector->y), vector->z);
    if (!std::isfinite(norm) || norm < 1e-12) {
        return false;
    }
    vector->x /= norm;
    vector->y /= norm;
    vector->z /= norm;
    return true;
}

bool interpolate_rotation(const RotationMatrix3& lower,
                          const RotationMatrix3& upper, double fraction,
                          RotationMatrix3* result) noexcept {
    if (result == nullptr) {
        return false;
    }
    const auto interpolate = [fraction](double lhs, double rhs) {
        return lhs + (rhs - lhs) * fraction;
    };
    Vector3 column_0{
        interpolate(lower.m00, upper.m00),
        interpolate(lower.m10, upper.m10),
        interpolate(lower.m20, upper.m20)};
    Vector3 column_1{
        interpolate(lower.m01, upper.m01),
        interpolate(lower.m11, upper.m11),
        interpolate(lower.m21, upper.m21)};
    if (!normalize_vector(&column_0)) {
        return false;
    }
    const double projection = column_0.x * column_1.x +
                              column_0.y * column_1.y +
                              column_0.z * column_1.z;
    column_1.x -= projection * column_0.x;
    column_1.y -= projection * column_0.y;
    column_1.z -= projection * column_0.z;
    if (!normalize_vector(&column_1)) {
        return false;
    }
    const Vector3 column_2 = cross_product(column_0, column_1);
    *result = {column_0.x, column_1.x, column_2.x,
               column_0.y, column_1.y, column_2.y,
               column_0.z, column_1.z, column_2.z};
    return finite_rotation(*result);
}

double pitch_from_rotation(const RotationMatrix3& matrix) noexcept {
    return std::atan2(matrix.m10, std::hypot(matrix.m00, matrix.m20));
}

double roll_from_rotation(const RotationMatrix3& matrix) noexcept {
    return std::atan2(-matrix.m12, matrix.m11);
}

}  // 匿名命名空间结束

std::uint64_t AttitudeHistory::double_bits(double value) noexcept {
    std::uint64_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    return bits;
}

double AttitudeHistory::bits_double(std::uint64_t bits) noexcept {
    double value = 0.0;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

bool AttitudeHistory::push(const AttitudeSample& sample) noexcept {
    if (sample.timestamp_ns < 0 || !std::isfinite(sample.pitch_rad) ||
        !std::isfinite(sample.roll_rad) || !std::isfinite(sample.speed_mps) ||
        !finite_rotation(sample.body_to_navigation) ||
        (has_timestamp_ && sample.timestamp_ns <= last_timestamp_ns_)) {
        return false;
    }

    Slot& slot = slots_[next_index_];
    const std::uint64_t sequence =
        slot.sequence.load(std::memory_order_relaxed);
    slot.sequence.store(sequence + 1, std::memory_order_release);
    slot.timestamp_ns.store(sample.timestamp_ns, std::memory_order_relaxed);
    slot.pitch_bits.store(double_bits(sample.pitch_rad),
                          std::memory_order_relaxed);
    slot.roll_bits.store(double_bits(sample.roll_rad),
                         std::memory_order_relaxed);
    slot.speed_bits.store(double_bits(sample.speed_mps),
                          std::memory_order_relaxed);
    slot.matrix_m00_bits.store(double_bits(sample.body_to_navigation.m00),
                               std::memory_order_relaxed);
    slot.matrix_m01_bits.store(double_bits(sample.body_to_navigation.m01),
                               std::memory_order_relaxed);
    slot.matrix_m02_bits.store(double_bits(sample.body_to_navigation.m02),
                               std::memory_order_relaxed);
    slot.matrix_m10_bits.store(double_bits(sample.body_to_navigation.m10),
                               std::memory_order_relaxed);
    slot.matrix_m11_bits.store(double_bits(sample.body_to_navigation.m11),
                               std::memory_order_relaxed);
    slot.matrix_m12_bits.store(double_bits(sample.body_to_navigation.m12),
                               std::memory_order_relaxed);
    slot.matrix_m20_bits.store(double_bits(sample.body_to_navigation.m20),
                               std::memory_order_relaxed);
    slot.matrix_m21_bits.store(double_bits(sample.body_to_navigation.m21),
                               std::memory_order_relaxed);
    slot.matrix_m22_bits.store(double_bits(sample.body_to_navigation.m22),
                               std::memory_order_relaxed);
    slot.launched.store(sample.launched, std::memory_order_relaxed);
    slot.launch_timestamp_ns.store(sample.launch_timestamp_ns,
                                   std::memory_order_relaxed);
    slot.launch_generation.store(sample.launch_generation,
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

std::optional<AttitudeLookup> AttitudeHistory::find_interpolated(
    TimestampNs timestamp_ns) const noexcept {
    if (timestamp_ns < 0 || size_.load(std::memory_order_acquire) == 0) {
        return std::nullopt;
    }

    AttitudeSample lower{};
    AttitudeSample upper{};
    bool has_lower = false;
    bool has_upper = false;
    for (std::size_t index = 0; index < capacity_; ++index) {
        AttitudeSample candidate{};
        if (!read_slot(index, &candidate)) {
            continue;
        }
        if (candidate.timestamp_ns == timestamp_ns) {
            const double pitch = pitch_from_rotation(
                candidate.body_to_navigation);
            const double roll = roll_from_rotation(
                candidate.body_to_navigation);
            if (!std::isfinite(pitch) || !std::isfinite(roll)) {
                return std::nullopt;
            }
            return AttitudeLookup{pitch, roll, candidate.speed_mps,
                                  candidate.body_to_navigation, false,
                                  candidate.timestamp_ns,
                                  candidate.timestamp_ns, candidate.launched,
                                  candidate.launch_timestamp_ns,
                                  candidate.launch_generation};
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

    if (!has_lower || !has_upper ||
        upper.timestamp_ns <= lower.timestamp_ns ||
        lower.launched != upper.launched ||
        lower.launch_generation != upper.launch_generation ||
        lower.launch_timestamp_ns != upper.launch_timestamp_ns) {
        return std::nullopt;
    }

    const double fraction = static_cast<double>(timestamp_ns -
                                                lower.timestamp_ns) /
                            static_cast<double>(upper.timestamp_ns -
                                                lower.timestamp_ns);
    if (!std::isfinite(fraction) || fraction < 0.0 || fraction > 1.0) {
        return std::nullopt;
    }
    RotationMatrix3 matrix{};
    if (!interpolate_rotation(lower.body_to_navigation,
                              upper.body_to_navigation, fraction, &matrix)) {
        return std::nullopt;
    }
    const double pitch = pitch_from_rotation(matrix);
    const double roll = roll_from_rotation(matrix);
    if (!std::isfinite(pitch) || !std::isfinite(roll)) {
        return std::nullopt;
    }
    const double speed = (1.0 - fraction) * lower.speed_mps +
                         fraction * upper.speed_mps;
    return AttitudeLookup{pitch, roll, speed, matrix, true, lower.timestamp_ns,
                          upper.timestamp_ns, lower.launched,
                          lower.launch_timestamp_ns,
                          lower.launch_generation};
}

void AttitudeHistory::reset() noexcept {
    for (std::size_t index = 0; index < capacity_; ++index) {
        slots_[index].sequence.store(0, std::memory_order_relaxed);
        slots_[index].timestamp_ns.store(-1, std::memory_order_relaxed);
        slots_[index].pitch_bits.store(0, std::memory_order_relaxed);
        slots_[index].speed_bits.store(0, std::memory_order_relaxed);
        slots_[index].matrix_m00_bits.store(0, std::memory_order_relaxed);
        slots_[index].matrix_m01_bits.store(0, std::memory_order_relaxed);
        slots_[index].matrix_m02_bits.store(0, std::memory_order_relaxed);
        slots_[index].matrix_m10_bits.store(0, std::memory_order_relaxed);
        slots_[index].matrix_m11_bits.store(0, std::memory_order_relaxed);
        slots_[index].matrix_m12_bits.store(0, std::memory_order_relaxed);
        slots_[index].matrix_m20_bits.store(0, std::memory_order_relaxed);
        slots_[index].matrix_m21_bits.store(0, std::memory_order_relaxed);
        slots_[index].matrix_m22_bits.store(0, std::memory_order_relaxed);
        slots_[index].launched.store(false, std::memory_order_relaxed);
        slots_[index].launch_timestamp_ns.store(-1,
                                                std::memory_order_relaxed);
        slots_[index].launch_generation.store(0,
                                              std::memory_order_relaxed);
    }
    next_index_ = 0;
    size_writer_ = 0;
    last_timestamp_ns_ = -1;
    has_timestamp_ = false;
    size_.store(0, std::memory_order_release);
}

bool AttitudeHistory::read_slot(std::size_t index,
                                AttitudeSample* sample) const noexcept {
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
    sample->pitch_rad =
        bits_double(slot.pitch_bits.load(std::memory_order_relaxed));
    sample->roll_rad =
        bits_double(slot.roll_bits.load(std::memory_order_relaxed));
    sample->speed_mps =
        bits_double(slot.speed_bits.load(std::memory_order_relaxed));
    sample->body_to_navigation = {
        bits_double(slot.matrix_m00_bits.load(std::memory_order_relaxed)),
        bits_double(slot.matrix_m01_bits.load(std::memory_order_relaxed)),
        bits_double(slot.matrix_m02_bits.load(std::memory_order_relaxed)),
        bits_double(slot.matrix_m10_bits.load(std::memory_order_relaxed)),
        bits_double(slot.matrix_m11_bits.load(std::memory_order_relaxed)),
        bits_double(slot.matrix_m12_bits.load(std::memory_order_relaxed)),
        bits_double(slot.matrix_m20_bits.load(std::memory_order_relaxed)),
        bits_double(slot.matrix_m21_bits.load(std::memory_order_relaxed)),
        bits_double(slot.matrix_m22_bits.load(std::memory_order_relaxed)),
    };
    sample->launched = slot.launched.load(std::memory_order_relaxed);
    sample->launch_timestamp_ns =
        slot.launch_timestamp_ns.load(std::memory_order_relaxed);
    sample->launch_generation =
        slot.launch_generation.load(std::memory_order_relaxed);
    const std::uint64_t second = slot.sequence.load(std::memory_order_acquire);
    return first == second && (second & 1U) == 0;
}

}  // namespace mosas::runtime
