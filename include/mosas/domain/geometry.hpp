#ifndef MOSAS_DOMAIN_GEOMETRY_HPP
#define MOSAS_DOMAIN_GEOMETRY_HPP

#include <array>

namespace mosas::domain {

struct CameraFrameTag {};
struct DartFrameTag {};
struct NavigationFrameTag {};
struct BallisticFrameTag {};

struct MetersPerSecond {};
struct MetersPerSecondSquared {};
struct RadiansPerSecond {};
struct Radian {};
struct GravityG {};
struct Dimensionless {};
struct PerSecond {};

template <class Unit>
struct Quantity {
    double value = 0.0;
};

template <class Frame, class Unit>
struct Vector3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

template <class ToFrame, class FromFrame>
class Rotation3 {
public:
    constexpr Rotation3() noexcept
        : matrix_{1.0, 0.0, 0.0,
                  0.0, 1.0, 0.0,
                  0.0, 0.0, 1.0} {}

    explicit constexpr Rotation3(std::array<double, 9> matrix) noexcept
        : matrix_(matrix) {}

    constexpr const std::array<double, 9>& matrix() const noexcept {
        return matrix_;
    }

    template <class Unit>
    Vector3<ToFrame, Unit> apply(
        const Vector3<FromFrame, Unit>& value) const noexcept {
        return {
            matrix_[0] * value.x + matrix_[1] * value.y + matrix_[2] * value.z,
            matrix_[3] * value.x + matrix_[4] * value.y + matrix_[5] * value.z,
            matrix_[6] * value.x + matrix_[7] * value.y + matrix_[8] * value.z,
        };
    }

private:
    std::array<double, 9> matrix_;
};

using VelocityDartMps = Vector3<DartFrameTag, MetersPerSecond>;
using AccelerationDartMps2 =
    Vector3<DartFrameTag, MetersPerSecondSquared>;
using AccelerationBallisticMps2 =
    Vector3<BallisticFrameTag, MetersPerSecondSquared>;
using AngularVelocityDartRadPerSec =
    Vector3<DartFrameTag, RadiansPerSecond>;
using DartOverloadG = Vector3<DartFrameTag, GravityG>;
using DirectionDartUnit = Vector3<DartFrameTag, Dimensionless>;
using DirectionRateDartPerSec = Vector3<DartFrameTag, PerSecond>;

}  // namespace mosas::domain

#endif  // MOSAS_DOMAIN_GEOMETRY_HPP
