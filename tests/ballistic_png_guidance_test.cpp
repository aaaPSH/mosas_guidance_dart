#include <mosas/guidance/ballistic_png_guidance.hpp>
#include "test_support.hpp"
#include <cmath>
#include <limits>
int main() {
    using namespace mosas;
    guidance::BallisticPngConfig config;
    config.navigation_constant_y=7; config.navigation_constant_z=11;
    guidance::PngCalculationTrace observed;
    std::uint64_t frame=0;
    guidance::BallisticPngGuidance law(config,[&](const domain::GuidanceInput& input,const guidance::PngCalculationTrace& trace){
        frame=input.source_frame_id; observed=trace;
    });
    domain::GuidanceInput input;
    input.source_frame_id=91; input.time.monotonic_ns=12;
    input.speed_mps.value=50; input.vector_los={true,{1,0,0},{0,0.02,-0.03}};
    const auto output=law.calculate(input);
    CHECK(output.valid && output.command.valid && frame==91 && observed.valid);
    CHECK(output.command.source_frame_id==91 && output.command.source_time.monotonic_ns==12);
    CHECK_NEAR(output.command.overload_g.y,7*(50*0.02/9.80665+0.5),1e-12);
    CHECK_NEAR(output.command.overload_g.z,-11*50*0.03/9.80665,1e-12);
    CHECK(output.command.overload_g.y>1); // 计算阶段不能偷偷执行输出限幅。
    CHECK_NEAR(output.diagnostics.requested_overload.y,observed.requested_overload.y,1e-12);
    input.vector_los={true,{0,1,0},{0.1,0,0}};
    const auto pole=law.calculate(input);
    CHECK(!pole.valid && !observed.valid);
    CHECK(pole.command.invalid_reason==domain::GuidanceInvalidReason::invalid_rate);
    CHECK(pole.command.overload_g.y==0 && pole.command.overload_g.z==0);
    input.vector_los.valid=false;
    CHECK(law.calculate(input).invalid_reason==domain::GuidanceInvalidReason::invalid_los);
    config.navigation_constant_z=std::numeric_limits<double>::infinity();
    CHECK(!guidance::BallisticPngGuidance::valid_config(config));
    bool rejected=false;
    try {guidance::BallisticPngGuidance invalid(config);}catch(const std::invalid_argument&){rejected=true;}
    CHECK(rejected);
    return 0;
}
