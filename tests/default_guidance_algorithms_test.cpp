#include <mosas/app/default_guidance_algorithms.hpp>
#include <mosas/app/app_config.hpp>
#include <mosas/guidance/inertial_angle_los_filter.hpp>
#include <mosas/guidance/pinhole_los_estimator.hpp>
#include "command_output_test_support.hpp"
#include "test_support.hpp"
#include <filesystem>
#include <fstream>
#include <sstream>
#include <unistd.h>

namespace {
struct Fixture {
    std::filesystem::path path=std::filesystem::temp_directory_path()/ ("mosas-config-v2-"+std::to_string(::getpid())+".conf");
    std::string source;
    Fixture(){std::ifstream input(std::filesystem::path(MOSAS_SOURCE_DIR)/"config/mosas_guidance.conf");std::ostringstream out;out<<input.rdbuf();source=out.str();}
    ~Fixture(){std::filesystem::remove(path);}
    bool load(const std::string& keys,mosas::app::AppConfig& config,std::string& error) {
        auto text=source;std::istringstream input(keys);std::string line,additional;
        while(std::getline(input,line)) {
            if(line.empty())continue;
            const auto key=line.substr(0,line.find('='));const auto at=text.find("\n"+key+"=");
            if(at==std::string::npos)additional+=line+'\n';
            else {const auto end=text.find('\n',at+1);text.replace(at+1,end-at-1,line);}
        }
        {std::ofstream output(path);output<<text<<"\n[guidance]\n"<<additional;}
        return mosas::app::AppConfigLoader::load(path,&config,&error);
    }
};
mosas::domain::CommandOutputContext output_context() {
    using namespace mosas;
    domain::CommandOutputContext c;
    c.source_frame_id=91;c.source_time=test::command_test_time(1'000'000);
    c.request_valid=c.launched=c.target_found=c.output_enabled=true;
    c.latest_state=test::command_test_state(2'000'000);
    c.emit_time_ns=2'000'001;c.maximum_attitude_age_ns=50'000'000;c.clock_domain=domain::ClockDomain::monotonic;
    return c;
}
}
int main() {
    using namespace mosas;
    Fixture file;app::AppConfig config;std::string error;
    const auto root=std::filesystem::path(MOSAS_SOURCE_DIR)/"config";
    CHECK(app::AppConfigLoader::load(root/"mosas_guidance.conf",&config,&error));
    CHECK(!config.enable_timing_statistics && config.sync_calibration.directory.empty());
    CHECK_NEAR(config.algorithms.attitude.gyro_stationary_threshold_rad_s,0.05,1e-12);
    CHECK_NEAR(config.algorithms.attitude.accel_filter_alpha,0.02,1e-12);
    CHECK(config.algorithms.attitude.stationary_confirm_ns==200'000'000);
    CHECK_NEAR(config.algorithms.los_filter.sigma_a,8.0,1e-12);
    CHECK_NEAR(config.algorithms.los_filter.sigma_a_pitch,10.0,1e-12);
    CHECK_NEAR(config.algorithms.los_filter.sigma_a_yaw,3.5,1e-12);
    CHECK_NEAR(config.algorithms.los_filter.sigma_q,0.0035,1e-12);
    CHECK_NEAR(config.algorithms.los_filter.initial_covariance,1.0,1e-12);
    CHECK(config.algorithms.los_filter.enable_filter);
    CHECK(config.algorithms.los_filter.enable_lever_arm_compensation);
    CHECK_NEAR(config.algorithms.los_filter.maximum_gap_seconds,0.2,1e-12);
    CHECK_NEAR(config.algorithms.los_filter.camera_lever_arm_x,0.10,1e-12);
    CHECK_NEAR(config.algorithms.los_filter.estimated_target_distance,15.0,1e-12);
    CHECK_NEAR(config.algorithms.command_mapping.maximum_command_overload,0.3,1e-12);
    CHECK_NEAR(config.algorithms.command_mapping.maximum_command_acceleration,9.80665,1e-12);
    CHECK(config.camera.fps==120 && config.command.send_rate_hz==100 && config.command.command_timeout_ms==100);
    CHECK(config.algorithms.vision_config.initial_roi.width==640);
    CHECK_NEAR(config.guidance.camera_intrinsics.fx,486.244430986463,1e-9);
    app::AppConfig calibration;
    CHECK(app::AppConfigLoader::load(root/"mosas_sync_calibration.conf",&calibration,&error));
    CHECK(calibration.enable_timing_statistics);
    CHECK_NEAR(calibration.algorithms.attitude.gyro_stationary_threshold_rad_s,0.05,1e-12);
    CHECK_NEAR(calibration.algorithms.attitude.accel_filter_alpha,0.02,1e-12);
    CHECK(calibration.algorithms.attitude.stationary_confirm_ns==200'000'000);
    CHECK(calibration.sync_calibration.directory==root.parent_path()/"data/sync_capture_01");
    CHECK(calibration.sync_calibration.seconds==45 && calibration.sync_calibration.fps==30);
    CHECK(calibration.sync_calibration.max_bytes==2048ULL*1024*1024);
    CHECK_NEAR(calibration.algorithms.los_filter.sigma_a,8.0,1e-12);
    CHECK_NEAR(calibration.algorithms.los_filter.sigma_a_pitch,10.0,1e-12);
    CHECK_NEAR(calibration.algorithms.los_filter.sigma_a_yaw,3.5,1e-12);
    CHECK_NEAR(calibration.algorithms.los_filter.sigma_q,0.0035,1e-12);
    CHECK_NEAR(calibration.algorithms.los_filter.camera_lever_arm_x,0.18,1e-12);
    CHECK(calibration.algorithms.los_filter.enable_lever_arm_compensation);

    CHECK(file.load("png_navigation_constant_y=7\npng_navigation_constant_z=11\npng_maximum_command_overload=10\npng_maximum_command_acceleration=inf\n",config,error));
    bool observed=false;
    auto deps=app::make_default_guidance_algorithms(config.algorithms,
        [&](const domain::GuidanceInput& input,const guidance::PngCalculationTrace& trace){observed=input.source_frame_id==91 && trace.valid;});
    CHECK(deps.vision_detector && deps.flight_state_estimator && deps.los_estimator && deps.los_rate_filter && deps.guidance_law && deps.command_mapper && deps.monotonic_clock);
    CHECK(dynamic_cast<guidance::PinholeLosEstimator*>(deps.los_estimator.get()));
    CHECK(dynamic_cast<guidance::InertialAngleLosFilter*>(deps.los_rate_filter.get()));
    CHECK(dynamic_cast<guidance::BallisticPngGuidance*>(deps.guidance_law.get()));
    domain::GuidanceInput input;
    input.source_frame_id=91;input.time=test::command_test_time(1'000'000);
    input.speed_mps.value=50;input.vector_los={true,{1,0,0},{0,0.02,-0.03}};
    auto raw=deps.guidance_law->calculate(input);
    CHECK(raw.valid && observed);
    CHECK_NEAR(raw.command.overload_g.y,7*(50*0.02/9.80665+0.5),1e-12);
    CHECK_NEAR(raw.command.overload_g.z,-11*50*0.03/9.80665,1e-12);
    CHECK(deps.command_mapper->map(raw.command.overload_g,output_context()).command.valid);

    for(const std::string channel:{"both","pitch","yaw"}) {
        CHECK(file.load("control_channel="+channel+"\n",config,error));
        auto selected=app::make_default_guidance_algorithms(config.algorithms);
        const auto mapped=selected.command_mapper->map({0,2,3},output_context());
        CHECK(mapped.command.valid);
        CHECK_NEAR(mapped.trace.command_overload.value,0.3,1e-12);
        if(channel=="pitch") CHECK(mapped.command.overload_g.z==0);
        if(channel=="yaw") CHECK(mapped.command.overload_g.y==0);
    }
    for(const auto* zero:{"png_maximum_command_overload=0\n","png_maximum_command_acceleration=0\n"}) {
        CHECK(file.load(zero,config,error));
        auto limited=app::make_default_guidance_algorithms(config.algorithms);
        const auto actual=limited.command_mapper->map({0,8,9},output_context());
        CHECK(actual.command.valid);
        CHECK(actual.command.overload_g.y==0 && actual.command.overload_g.z==0);
    }
    CHECK(file.load("los_kalman_sigma_a=6\nlos_kalman_sigma_a_pitch=0\nlos_kalman_sigma_a_yaw=2\n"
                    "los_kalman_sigma_q=0.01\nlos_initial_covariance=0\nlos_enable_filter=false\n"
                    "los_enable_lever_arm_compensation=false\n"
                    "los_maximum_gap_ms=150\nlos_camera_lever_arm_x=-0.12\nlos_estimated_target_distance=20\n",config,error));
    CHECK_NEAR(config.algorithms.los_filter.sigma_a,6.0,1e-12);
    CHECK_NEAR(config.algorithms.los_filter.sigma_a_pitch,0.0,1e-12);
    CHECK_NEAR(config.algorithms.los_filter.sigma_a_yaw,2.0,1e-12);
    CHECK_NEAR(config.algorithms.los_filter.sigma_q,0.01,1e-12);
    CHECK_NEAR(config.algorithms.los_filter.initial_covariance,0.0,1e-12);
    CHECK(!config.algorithms.los_filter.enable_filter);
    CHECK(!config.algorithms.los_filter.enable_lever_arm_compensation);
    CHECK_NEAR(config.algorithms.los_filter.maximum_gap_seconds,0.15,1e-12);
    CHECK_NEAR(config.algorithms.los_filter.camera_lever_arm_x,-0.12,1e-12);
    CHECK_NEAR(config.algorithms.los_filter.estimated_target_distance,20.0,1e-12);
    for(const auto* deleted:{"guidance_algorithm=png\n","body_vector_los_sigma_a=4\n","body_vector_navigation_constant=3\n",
            "inertial_los_sigma_a=4\n","inertial_los_sigma_l=0.0005\n","inertial_los_initial_rate_covariance=1\n",
            "inertial_los_maximum_gap_ms=200\n","inertial_los_camera_lever_arm_x=0.18\n","inertial_los_estimated_target_distance=15\n",
            "los_process_noise=4\n","ballistic_png_navigation_constant=3\n"}) {
        CHECK(!file.load(deleted,config,error));
        CHECK(error.find(std::string(deleted).substr(0,std::string(deleted).find('=')))!=std::string::npos);
    }
    for(const auto* invalid:{"control_channel=Pitch\n","png_navigation_constant_y=0\n","png_navigation_constant_z=nan\n",
            "los_kalman_sigma_a=0\n","los_kalman_sigma_a_pitch=-1\n","los_kalman_sigma_a_yaw=nan\n",
            "los_kalman_sigma_q=0\n","los_initial_covariance=-1\n","los_maximum_gap_ms=0\n",
            "los_enable_filter=maybe\n","los_enable_lever_arm_compensation=maybe\n",
            "los_camera_lever_arm_x=inf\n","los_estimated_target_distance=0\n",
            "png_maximum_command_overload=-1\n","png_maximum_command_acceleration=-1\n","png_gravity=0\n",
            "camera_to_body_m00=2\n","launch_speed_mps=0\n","launch_detection_mode=unknown\n",
            "launch_release_confirm_ms=0\n","launch_release_timeout_ms=10\n","attitude_maximum_gap_ms=0\n",
            "attitude_accel_filter_alpha=0\n","attitude_accel_filter_alpha=nan\n","attitude_accel_filter_alpha=1.1\n",
            "attitude_stationary_confirm_ms=0\n","attitude_stationary_confirm_ms=199\n",
            "launch_release_acceleration_max_mps2=9.6\n","send_rate_hz=0\n","send_rate_hz=1001\n","command_timeout_ms=0\n"})
        CHECK(!file.load(invalid,config,error));
    CHECK(file.load("attitude_accel_filter_alpha=0.1\nattitude_stationary_confirm_ms=300\n",config,error));
    CHECK_NEAR(config.algorithms.attitude.accel_filter_alpha,0.1,1e-12);
    CHECK(config.algorithms.attitude.stationary_confirm_ns==300'000'000);
    for(const std::string mode:{"acceleration","two_stage"}) {
        CHECK(file.load("launch_detection_mode="+mode+"\n",config,error));
        auto algorithms=app::make_default_guidance_algorithms(config.algorithms);
        domain::ImuKinematicSample imu;
        imu.acceleration={0,9.80665,0};
        for(std::int64_t ms=0;ms<=200;ms+=5) {
            imu.time.monotonic_ns=ms*1'000'000;
            CHECK(algorithms.flight_state_estimator->update(imu).attitude.valid==(ms==200));
        }
        imu.time.monotonic_ns=205'000'000;imu.acceleration={30,0,0};algorithms.flight_state_estimator->update(imu);
        imu.time.monotonic_ns=225'000'000;
        CHECK((algorithms.flight_state_estimator->update(imu).launch.phase==domain::LaunchPhase::launched)==(mode=="acceleration"));
        imu.time.monotonic_ns=230'000'000;imu.acceleration={1,0,0};algorithms.flight_state_estimator->update(imu);
        imu.time.monotonic_ns=240'000'000;
        CHECK(algorithms.flight_state_estimator->update(imu).launch.phase==domain::LaunchPhase::launched);
    }
    CHECK(file.load("png_gravity=10\npng_maximum_command_overload=10\npng_maximum_command_acceleration=inf\n",config,error));
    config.algorithms.ballistic_png.gravity=5;config.algorithms.command_mapping.gravity=99;
    auto unified=app::make_default_guidance_algorithms(config.algorithms);
    input.vector_los.inertial_direction_rate={0,0.2,0};
    raw=unified.guidance_law->calculate(input);
    CHECK_NEAR(raw.command.overload_g.y,4.5,1e-12);
    CHECK_NEAR(unified.command_mapper->map(raw.command.overload_g,output_context()).trace.body_acceleration.y,45,1e-12);
    return 0;
}
