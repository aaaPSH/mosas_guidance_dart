#include <mosas/app/simulation_contract.hpp>
#include "test_support.hpp"
#include <limits>

bool rejects(const mosas::app::AppConfig& app, const dart_sim_transport::ServerSnapshot& snapshot,
             const std::string& field) {
    try {mosas::app::validate_simulation_metadata(app,snapshot);}
    catch(const std::runtime_error& e){return std::string(e.what()).find(field)!=std::string::npos;}
    return false;
}
int main() {
    using namespace mosas; using namespace dart_sim_transport;
    app::AppConfig original;std::string error;
    CHECK(app::AppConfigLoader::load(std::filesystem::path(MOSAS_SOURCE_DIR)/"config/mosas_guidance.conf",&original,&error));
    CameraInfo camera;camera.width=640;camera.height=480;camera.stride=1920;camera.camera_hz=120;camera.imu_hz=200;
    camera.fx=486.244430986463;camera.fy=486.544705399672;camera.cx=308.810501121211;camera.cy=244.693205492819;
    ServerSnapshot good{101,0,Session{0,Phase::ready,3},camera};
    app::validate_simulation_metadata(original,good);
    auto bad=good;bad.camera.width=320;CHECK(rejects(original,bad,"width"));
    bad=good;bad.camera.height=240;CHECK(rejects(original,bad,"height"));
    for(const auto pair : {std::make_pair(&CameraInfo::camera_hz,"camera_hz"),
                          std::make_pair(&CameraInfo::imu_hz,"imu_hz"),
                          std::make_pair(&CameraInfo::fx,"fx"),std::make_pair(&CameraInfo::fy,"fy"),
                          std::make_pair(&CameraInfo::cx,"cx"),std::make_pair(&CameraInfo::cy,"cy")}) {
        bad=good;bad.camera.*pair.first+=1;CHECK(rejects(original,bad,pair.second));
        bad=good;bad.camera.*pair.first=std::numeric_limits<double>::quiet_NaN();CHECK(rejects(original,bad,pair.second));
    }
    bad=good;bad.camera.fx+=0.0001;app::validate_simulation_metadata(original,bad);
    bad=good;bad.camera.fx+=0.001;CHECK(rejects(original,bad,"fx"));
    bad=good;bad.camera.R_S_from_C={0,-1,0,1,0,0,0,0,1};CHECK(rejects(original,bad,"rotation"));
    bad=good;bad.session.g0_mps2=10;CHECK(rejects(original,bad,"gravity"));
    for(int i=0;i<3;++i){bad=good;bad.session.world_gravity_mps2[i]+=1;CHECK(rejects(original,bad,"gravity"));}
    bad=good;bad.camera.stride+=1;CHECK(rejects(original,bad,"stride"));
    return 0;
}
