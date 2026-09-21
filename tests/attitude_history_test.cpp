#include <mosas/runtime/attitude_history.hpp>

#include <cassert>
#include <cmath>

namespace {

RotationMatrix3 identity_matrix() {
    return {1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0};
}

RotationMatrix3 pitch_matrix(double angle_rad) {
    const double cosine = std::cos(angle_rad);
    const double sine = std::sin(angle_rad);
    return {cosine, -sine, 0.0,
            sine, cosine, 0.0,
            0.0, 0.0, 1.0};
}

mosas::runtime::AttitudeSample sample(
    mosas::runtime::TimestampNs timestamp, double pitch, bool launched) {
    return {timestamp, pitch, pitch_matrix(pitch), launched};
}

void test_interpolates_pitch_between_timestamped_samples() {
    mosas::runtime::AttitudeHistory history(4);
    assert(history.push({10, 0.0, identity_matrix(), false}));
    assert(history.push({20, 0.5 * std::acos(-1.0),
                         pitch_matrix(0.5 * std::acos(-1.0)), false}));

    const auto lookup = history.find_interpolated(15);
    assert(lookup.has_value());
    assert(lookup->interpolated);
    assert(std::abs(lookup->pitch_rad - 0.25 * std::acos(-1.0)) <
           1e-12);
    assert(std::abs(lookup->body_to_navigation.m00 - std::sqrt(0.5)) <
           1e-12);
    assert(std::abs(lookup->body_to_navigation.m10 - std::sqrt(0.5)) <
           1e-12);
    assert(std::abs(lookup->body_to_navigation.m01 + std::sqrt(0.5)) <
           1e-12);
    assert(std::abs(lookup->body_to_navigation.m11 - std::sqrt(0.5)) <
           1e-12);
    assert(lookup->lower_timestamp_ns == 10);
    assert(lookup->upper_timestamp_ns == 20);
    assert(!lookup->launched);
}

void test_does_not_extrapolate_outside_history() {
    mosas::runtime::AttitudeHistory history(4);
    assert(history.push(sample(10, 0.1, false)));
    assert(history.push(sample(20, 0.3, false)));
    assert(!history.find_interpolated(5).has_value());
    assert(!history.find_interpolated(25).has_value());
}

void test_rejects_timestamp_rollback_and_cross_launch_interpolation() {
    mosas::runtime::AttitudeHistory history(4);
    assert(history.push(sample(10, 0.1, false)));
    assert(!history.push(sample(9, 0.2, false)));
    assert(history.push(sample(20, 0.3, true)));
    assert(!history.find_interpolated(15).has_value());
}

void test_overwrites_oldest_sample_at_capacity() {
    mosas::runtime::AttitudeHistory history(2);
    assert(history.push(sample(10, 0.1, true)));
    assert(history.push(sample(20, 0.2, true)));
    assert(history.push(sample(30, 0.3, true)));
    assert(!history.find_interpolated(15).has_value());
    const auto lookup = history.find_interpolated(25);
    assert(lookup.has_value());
    assert(std::abs(lookup->pitch_rad - 0.25) < 1e-12);
}

}  // namespace

int main() {
    test_interpolates_pitch_between_timestamped_samples();
    test_does_not_extrapolate_outside_history();
    test_rejects_timestamp_rollback_and_cross_launch_interpolation();
    test_overwrites_oldest_sample_at_capacity();
    return 0;
}
