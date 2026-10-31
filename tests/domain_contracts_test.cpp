#include <mosas/domain/contracts.hpp>

#include "test_support.hpp"

#include <array>
#include <type_traits>

using CameraAcceleration =
    mosas::domain::Vector3<mosas::domain::CameraFrameTag,
                           mosas::domain::MetersPerSecondSquared>;
using DartAcceleration =
    mosas::domain::Vector3<mosas::domain::DartFrameTag,
                           mosas::domain::MetersPerSecondSquared>;
using CameraAngularRate =
    mosas::domain::Vector3<mosas::domain::CameraFrameTag,
                           mosas::domain::RadiansPerSecond>;

static_assert(!std::is_convertible_v<CameraAcceleration, DartAcceleration>);
static_assert(!std::is_convertible_v<CameraAcceleration, CameraAngularRate>);

int main() {
    using namespace mosas::domain;
    const Rotation3<DartFrameTag, CameraFrameTag> camera_to_dart(
        std::array<double, 9>{0.0, -1.0, 0.0,
                              1.0,  0.0, 0.0,
                              0.0,  0.0, 1.0});
    const CameraAcceleration camera_x{1.0, 0.0, 0.0};
    const auto dart_vector = camera_to_dart.apply(camera_x);
    CHECK_NEAR(dart_vector.x, 0.0, 1e-12);
    CHECK_NEAR(dart_vector.y, 1.0, 1e-12);
    CHECK_NEAR(dart_vector.z, 0.0, 1e-12);

    SensorTime unknown_time{};
    CHECK(!unknown_time.is_valid());
    CHECK(unknown_time.uncertainty_ns == -1);

    GuidanceCommand command{};
    CHECK(!command.valid);
    command.valid = true;
    command.overload_g = {0.0, 0.0, 0.0};
    CHECK(command.valid);
    CHECK(command.overload_g.x == 0.0);

    TargetObservation observation{};
    CHECK(!observation.valid);
    observation.valid = true;
    observation.reason = ObservationReason::found;
    observation.center = {320.0, 240.0};
    CHECK(observation.valid);
    CHECK(observation.reason == ObservationReason::found);
    return 0;
}
