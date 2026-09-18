#include <mosas/runtime/imu_angular_velocity_history.hpp>

#include <cassert>
#include <cmath>
#include <stdexcept>

namespace {

mosas::runtime::ImuAngularVelocitySample sample(
    mosas::runtime::TimestampNs timestamp, double x, double y, double z,
    double acceleration_x = 0.0, double acceleration_y = 0.0,
    double acceleration_z = 0.0) {
    return {timestamp,
            {acceleration_x, acceleration_y, acceleration_z},
            {x, y, z}};
}

void test_history_interpolates_without_extrapolation() {
    mosas::runtime::ImuAngularVelocityHistory history(4);
    assert(history.push(sample(10, 1.0, 2.0, 3.0, 10.0, 20.0, 30.0)));
    assert(history.push(sample(20, 3.0, 6.0, 9.0, 30.0, 60.0, 90.0)));

    const auto exact = history.find_interpolated(10);
    assert(exact.has_value());
    assert(!exact->interpolated);
    assert(exact->sample.angular_velocity.x == 1.0);
    assert(exact->sample.acceleration.x == 10.0);

    const auto midpoint = history.find_interpolated(15);
    assert(midpoint.has_value());
    assert(midpoint->interpolated);
    assert(midpoint->lower_timestamp_ns == 10);
    assert(midpoint->upper_timestamp_ns == 20);
    assert(std::abs(midpoint->sample.angular_velocity.x - 2.0) < 1e-12);
    assert(std::abs(midpoint->sample.angular_velocity.y - 4.0) < 1e-12);
    assert(std::abs(midpoint->sample.angular_velocity.z - 6.0) < 1e-12);
    assert(std::abs(midpoint->sample.acceleration.x - 20.0) < 1e-12);
    assert(std::abs(midpoint->sample.acceleration.y - 40.0) < 1e-12);
    assert(std::abs(midpoint->sample.acceleration.z - 60.0) < 1e-12);

    assert(!history.find_interpolated(9).has_value());
    assert(!history.find_interpolated(21).has_value());
}

void test_history_rejects_rollback_and_overwrites_oldest_slot() {
    mosas::runtime::ImuAngularVelocityHistory history(2);
    assert(history.push(sample(10, 1.0, 0.0, 0.0)));
    assert(history.push(sample(20, 2.0, 0.0, 0.0)));
    assert(!history.push(sample(19, 3.0, 0.0, 0.0)));
    assert(history.push(sample(30, 3.0, 0.0, 0.0)));
    assert(!history.find_interpolated(10).has_value());
    assert(history.find_interpolated(20).has_value());
    assert(history.find_interpolated(30).has_value());
}

void test_history_rejects_invalid_capacity() {
    bool threw = false;
    try {
        mosas::runtime::ImuAngularVelocityHistory history(0);
        (void)history;
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    assert(threw);
}

}  // namespace

int main() {
    test_history_interpolates_without_extrapolation();
    test_history_rejects_rollback_and_overwrites_oldest_slot();
    test_history_rejects_invalid_capacity();
    return 0;
}
