#include <mosas/guidance/png_calculation_trace.hpp>
#include <mosas/guidance/default_algorithm_adapters.hpp>
#include "test_support.hpp"
#include "body_vector_test_support.hpp"
#include <array>
#include <cstring>
#include <limits>

namespace {
/** @brief Compare computed fields only; struct padding is not a numeric contract. */
bool identical(const PngGuidanceOutput& a,const PngGuidanceOutput& b){
    const std::array<double,11> x={a.navigation_acceleration.x,a.navigation_acceleration.y,a.navigation_acceleration.z,
        a.body_acceleration.x,a.body_acceleration.y,a.body_acceleration.z,
        a.body_overload.x,a.body_overload.y,a.body_overload.z,a.command_overload,a.command_phase};
    const std::array<double,11> y={b.navigation_acceleration.x,b.navigation_acceleration.y,b.navigation_acceleration.z,
        b.body_acceleration.x,b.body_acceleration.y,b.body_acceleration.z,
        b.body_overload.x,b.body_overload.y,b.body_overload.z,b.command_overload,b.command_phase};
    return a.valid==b.valid&&std::memcmp(x.data(),y.data(),sizeof(x))==0;
}
}
int main(){
    const LineOfSight los{true,0,0};
    for(double pitch:{-.7,0.,.6})for(double roll:{0.,.4})for(double speed:{1.,16.})
    for(double y:{-.2,0.,.2})for(double z:{-.2,0.,.2})for(double cap:{.3,1.})
    for(double acceleration:{2.,std::numeric_limits<double>::infinity()}){
        PngGuidanceConfig config;config.maximum_command_overload=cap;config.maximum_command_acceleration=acceleration;
        PngCalculationTrace trace;
        const auto plain=PngGuidance::calculate(los,{true,y,z},speed,mosas::test::runtime_test_rotation(pitch,roll),config);
        const auto observed=PngGuidance::calculate(los,{true,y,z},speed,mosas::test::runtime_test_rotation(pitch,roll),config,&trace);
        CHECK(identical(plain,observed));CHECK(trace.valid==observed.valid);
    }
    PngGuidanceConfig config;config.maximum_command_overload=.3;config.maximum_command_acceleration=9.80665;
    PngCalculationTrace trace;
    auto output=PngGuidance::calculate(los,{true,0,0},16,mosas::test::runtime_test_rotation(35*std::acos(-1.)/180),config,&trace);
    CHECK(trace.valid&&trace.overload_limited&&!trace.acceleration_limited);
    CHECK_NEAR(trace.raw_navigation_overload.y,1.5*std::cos(35*std::acos(-1.)/180),1e-15);
    CHECK_NEAR(trace.inertial_los_elevation_rad,35*std::acos(-1.)/180,1e-15);
    CHECK_NEAR(trace.measured_elevation_rate_rad_s,0,0);
    CHECK_NEAR(trace.parabolic_elevation_rate_rad_s,
               -config.gravity*std::cos(trace.inertial_los_elevation_rad)/32,1e-15);
    CHECK_NEAR(trace.compensated_elevation_rate_rad_s,-trace.parabolic_elevation_rate_rad_s,0);
    CHECK_NEAR(trace.after_overload_limit.y,.3,1e-15);
    CHECK_NEAR(trace.navigation_acceleration.y,output.navigation_acceleration.y,0);
    config.maximum_command_acceleration=.1;
    output=PngGuidance::calculate(los,{true,0,0},16,mosas::test::runtime_test_rotation(0,.4),config,&trace);
    CHECK(trace.acceleration_limited);CHECK_NEAR(trace.navigation_acceleration.y,.1,1e-15);
    output=PngGuidance::calculate(los,{true,std::numeric_limits<double>::quiet_NaN(),0},16,mosas::test::runtime_test_rotation(),config,&trace);
    CHECK(!output.valid&&!trace.valid);CHECK_NEAR(trace.raw_navigation_overload.y,0,0);
    CHECK_NEAR(trace.inertial_los_elevation_rad,0,0);
    CHECK_NEAR(trace.measured_elevation_rate_rad_s,0,0);
    CHECK_NEAR(trace.parabolic_elevation_rate_rad_s,0,0);
    CHECK_NEAR(trace.compensated_elevation_rate_rad_s,0,0);

    std::uint64_t frame=0;bool callback_valid=false;
    mosas::guidance::ExistingPngGuidanceLawAdapter adapter(config,
        [&](const mosas::domain::GuidanceInput& input,const PngCalculationTrace& value){
            frame=input.source_frame_id;callback_valid=value.valid;
        });
    mosas::domain::GuidanceInput input;
    input.source_frame_id=71;input.line_of_sight={true,{0},{0}};
    input.angular_velocity={true,{0},{0}};input.attitude.valid=true;input.speed_mps.value=16;
    CHECK(adapter.calculate(input).valid);CHECK(callback_valid&&frame==71);
    return 0;
}
