#pragma once
#include <mosas/runtime/vector3.hpp>
#include <functional>

/** @brief Optional snapshot of values already computed by PNG.
 * @note LOS elevation is in rad, rates in rad/s, overloads in g, and
 * navigation_acceleration in m/s². The raw and after_overload_limit vectors
 * use ballistic axes. Raw values precede body-channel selection; after_overload_limit
 * is the selected body command converted back to ballistic axes, before the acceleration cap.
 * Final navigation_acceleration and body_overload reflect channel selection and both caps.
 * A reused snapshot is reset on entry; valid remains false on rejected input.
 */
struct PngCalculationTrace {
    bool valid=false;
    bool overload_limited=false;
    bool acceleration_limited=false;
    double inertial_los_elevation_rad=0;
    double measured_elevation_rate_rad_s=0;
    double parabolic_elevation_rate_rad_s=0;
    double compensated_elevation_rate_rad_s=0;
    Vector3 raw_navigation_overload{0,0,0};
    Vector3 after_overload_limit{0,0,0};
    Vector3 navigation_acceleration{0,0,0};
    Vector3 body_overload{0,0,0};
    double raw_command_overload=0;
};

namespace mosas::domain {struct GuidanceInput;}
namespace mosas::guidance {
/** @brief Read-only diagnostic consumer, disabled by default; never a feedback port.
 * The simulation assembler may install a bounded audit sink. Callback failures
 * are runtime faults, not permission to change or retry the numeric command.
 */
using PngTraceObserver=std::function<void(const domain::GuidanceInput&,const PngCalculationTrace&)>;
}
