#include <mosas/guidance/ballistic_png_guidance.hpp>
#include "test_support.hpp"
#include <cmath>
int main() {
    using namespace mosas;
    guidance::PngCalculationTrace trace;
    guidance::BallisticPngGuidance law({},[&](const domain::GuidanceInput&,const guidance::PngCalculationTrace& value){trace=value;});
    domain::GuidanceInput input;
    input.vector_los={true,{0.8,0.6,0},{-0.12,0.16,0.08}};
    input.speed_mps.value=50;
    CHECK(law.calculate(input).valid);
    CHECK_NEAR(trace.inertial_los_elevation_rad,std::asin(0.6),1e-12);
    CHECK_NEAR(trace.measured_elevation_rate_rad_s,0.2,1e-12);
    CHECK_NEAR(trace.measured_lateral_rate_rad_s,0.08,1e-12);
    CHECK_NEAR(trace.parabolic_elevation_rate_rad_s,-9.80665*0.8/100,1e-12);
    CHECK_NEAR(trace.compensated_elevation_rate_rad_s,0.2+9.80665*0.8/100,1e-12);
    CHECK_NEAR(trace.requested_overload.y,3*50/9.80665*(0.2+9.80665*0.8/100),1e-12);
    input.vector_los.valid=false;
    CHECK(!law.calculate(input).valid);
    CHECK(!trace.valid && trace.requested_overload.y==0 && trace.requested_overload.z==0);
    return 0;
}
