#include <mosas/runtime/runtime_types.hpp>

#include <cassert>
#include <cmath>

int main() {
    mosas::runtime::ImuStateSnapshot snapshot{};
    snapshot.pitch_rad = 0.25;
    snapshot.attitude_valid = true;
    snapshot.launched = true;
    snapshot.launch_timestamp_ns = 123456789;
    snapshot.launch_generation = 1;
    snapshot.body_to_navigation = {
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0};

    assert(snapshot.attitude_valid);
    assert(snapshot.launched);
    assert(std::abs(snapshot.pitch_rad - 0.25) < 1e-12);
    assert(snapshot.launch_timestamp_ns == 123456789);
    assert(snapshot.launch_generation == 1);
    return 0;
}
