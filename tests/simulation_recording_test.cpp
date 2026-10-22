#include <mosas/app/simulation_runner.hpp>
#include <dart_sim_transport/endpoint.hpp>

#include "test_support.hpp"

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <opencv2/videoio.hpp>
#include <stdexcept>
#include <thread>
#include <unistd.h>

using namespace std::chrono_literals;
using namespace mosas;
using namespace dart_sim_transport;

namespace {
struct Fixture {
    std::filesystem::path root;
    Fixture() {
        char name[]="/tmp/mosas-recording-XXXXXX";
        const char* directory=mkdtemp(name);
        if(!directory)throw std::runtime_error("cannot create recording fixture");
        root=directory;
    }
    ~Fixture() {std::error_code ignored;std::filesystem::remove_all(root,ignored);}
};

template<class Predicate>
bool advance(app::SimulationRunner& runner,Predicate predicate) {
    const auto deadline=std::chrono::steady_clock::now()+3s;
    while(!predicate() && std::chrono::steady_clock::now()<deadline) {
        runner.step();std::this_thread::sleep_for(2ms);
    }
    return predicate();
}

std::vector<std::filesystem::path> files_in(const std::filesystem::path& root,const std::string& extension) {
    std::vector<std::filesystem::path> files;
    if(std::filesystem::exists(root))
        for(const auto& entry:std::filesystem::recursive_directory_iterator(root))
            if(entry.is_regular_file() && entry.path().extension()==extension)files.push_back(entry.path());
    return files;
}
std::vector<std::string> lines_in(const std::filesystem::path& path) {
    std::ifstream input(path);std::vector<std::string> lines;std::string line;
    while(std::getline(input,line))lines.push_back(line);
    return lines;
}
std::string read_bytes(const std::filesystem::path& path) {
    std::ifstream input(path,std::ios::binary);
    return {std::istreambuf_iterator<char>(input),std::istreambuf_iterator<char>()};
}
bool contains(const std::vector<std::string>& log,const std::string& text) {
    return std::any_of(log.begin(),log.end(),[&](const auto& line){return line.find(text)!=std::string::npos;});
}

app::SimulationConfig configuration(const Fixture& fixture,const app::AppConfig& original) {
    app::SimulationConfig config;config.app=original;
    config.app_config_path=std::filesystem::path(MOSAS_SOURCE_DIR)/"config/mosas_guidance.conf";
    config.socket_directory=(fixture.root/"model").string();
    config.app.output.recording_path=(fixture.root/"recordings"/
        std::filesystem::path(original.output.recording_path).filename()).string();
    return config;
}
ServerSnapshot metadata(const app::AppConfig& original,std::uint64_t id) {
    CameraInfo camera;
    camera.width=original.camera.width;camera.height=original.camera.height;camera.stride=camera.width*3;
    camera.pixel_format=PixelFormat::rgb8;
    camera.camera_hz=original.camera.fps;camera.imu_hz=original.imu.sample_rate_hz;
    const auto& k=original.guidance.camera_intrinsics;
    camera.fx=k.fx;camera.fy=k.fy;camera.cx=k.cx;camera.cy=k.cy;
    camera.R_S_from_C={1,0,0,0,1,0,0,0,1};
    Session session{0,Phase::ready,3};session.g0_mps2=standard_gravity;
    session.world_gravity_mps2={0,0,-standard_gravity};
    return {id,0,session,camera};
}
Image image(const CameraInfo& camera,std::uint8_t level) {
    return {camera.width,camera.height,camera.stride,PixelFormat::rgb8,
            Bytes(static_cast<std::size_t>(camera.stride)*camera.height,level)};
}

/** @brief Real Socket sources and original algorithms naturally trigger and flush two partial match windows. */
int match_partial_windows_survive_two_sessions(const app::AppConfig& original) {
    Fixture fixture;auto config=configuration(fixture,original);
    CHECK(config.app.output.mode==output::OutputMode::match);
    CHECK(config.app.output.match_window_duration_ms==4000);
    auto snapshot=metadata(original,601);Endpoint server(config.socket_directory,snapshot);
    const auto recordings=fixture.root/"recordings";
    std::vector<std::string> log;app::SimulationRunner runner(config,[&](const auto& line){log.push_back(line);});
    std::filesystem::path previous_directory;
    std::vector<std::pair<std::filesystem::path,std::string>> previous_files;
    for(int shot=0;shot<2;++shot) {
        if(shot){snapshot.session_id=602;server.set_snapshot(snapshot);}
        CHECK(advance(runner,[&]{return runner.running() && runner.session_id()==snapshot.session_id;}));
        // Stationary history, a 100 ms acceleration ramp, then low-specific-force flight.
        // Neither protocol flight phase nor estimator launch state is injected.
        for(std::int64_t time=0;time<=200'000'000;time+=5'000'000) {
            const bool accelerating=time>50'000'000 && time<=150'000'000;
            const bool released=time>150'000'000;
            CHECK(server.submit(Imu{3,{accelerating?original.algorithms.attitude.launch_speed_mps/0.1:0,
                                      0,released?0:standard_gravity},{}},time));
            if(time==180'000'000) {
                // Wait before advancing simulation time beyond the 10 ms sync
                // budget or overwriting the transport's intentional latest-only image slot.
                CHECK(server.submit(image(snapshot.camera,220),175'000'000));
                CHECK(advance(runner,[&]{return runner.timing().output.samples>=2;}));
            }
        }
        CHECK(server.submit(image(snapshot.camera,240),195'000'000));
        CHECK(advance(runner,[&]{return runner.timing().output.samples>=4;}));
        CHECK(runner.running());
        const auto diagnostics=runner.output_diagnostics();
        CHECK(diagnostics.video.accepted_count==2 && diagnostics.video.healthy);
        CHECK(diagnostics.telemetry.accepted_count==2 && diagnostics.telemetry.healthy);
        CHECK(diagnostics.imu_recording.accepted_count==41 && diagnostics.imu_recording.healthy);
        CHECK(diagnostics.video.dropped_count==0 && diagnostics.telemetry.dropped_count==0);
        CHECK(diagnostics.imu_recording.dropped_count==0);
        const auto launch_ns = original.algorithms.attitude.launch_detection_mode ==
            runtime::LaunchDetectionMode::two_stage ? "165000000" : "75000000";
        CHECK(contains(log, std::string("MOSAS launch_detected_ns=") + launch_ns));
        CHECK(files_in(recordings,".avi").size()==static_cast<std::size_t>(shot));
        runner.stop(); // Less than 4 seconds: joins workers, encodes partial window, flushes CSVs.
        CHECK(!runner.running() && runner.last_error().empty());
        if (original.algorithms.algorithm == app::GuidanceAlgorithm::png) {
            CHECK(std::count_if(log.begin(),log.end(),[](const auto& line) {
                return line.find("pre_acquisition preset_submitted=2 ")==0;
            })==shot+1);
        } else {
            CHECK(!contains(log,"pre_acquisition preset_submitted="));
        }
        const auto videos=files_in(recordings,".avi");
        CHECK(videos.size()==static_cast<std::size_t>(shot+1));
        const auto selected=std::find_if(videos.begin(),videos.end(),[&](const auto& p){return p.parent_path()!=previous_directory;});
        CHECK(selected!=videos.end());const auto video=*selected;
        CHECK(video.filename().string().find("mosas_guidance_")==0);
        cv::VideoCapture capture(video.string());cv::Mat decoded;
        CHECK_NEAR(capture.get(cv::CAP_PROP_FPS),120,0.001);
        CHECK(capture.read(decoded));CHECK(decoded.cols==640 && decoded.rows==480);
        CHECK(decoded.at<cv::Vec3b>(100,500)[0]>200);
        const auto first_pixel=decoded.at<cv::Vec3b>(100,500)[0];
        CHECK(capture.read(decoded));CHECK(decoded.cols==640 && decoded.rows==480);
        CHECK(decoded.at<cv::Vec3b>(100,500)[0]>first_pixel+10);
        CHECK(!capture.read(decoded));capture.release();
        auto telemetry=video;telemetry.replace_extension(".csv");
        const auto rows=lines_in(telemetry);
        CHECK(rows.size()==3);CHECK(rows[0].find("frame_index,frame_timestamp_ns,")==0);
        CHECK(rows[1].find("0,175000000,")==0);CHECK(rows[2].find("1,195000000,")==0);
        std::vector<std::filesystem::path> imu_files;
        for(const auto& path:files_in(video.parent_path(),".csv"))
            if(path.filename().string().find("_imu_")!=std::string::npos)imu_files.push_back(path);
        CHECK(imu_files.size()==1);
        const auto samples=lines_in(imu_files[0]);
        CHECK(samples.size()==42);CHECK(samples[0].find("sample_index,timestamp_ns,")==0);
        CHECK(samples[1].find("0,0,")==0);CHECK(samples.back().find("40,200000000,")==0);
        if(shot) {
            CHECK(video.parent_path()!=previous_directory);
            for(const auto& saved:previous_files)CHECK(read_bytes(saved.first)==saved.second);
        } else {
            for(const auto& entry:std::filesystem::directory_iterator(video.parent_path()))
                if(entry.is_regular_file())previous_files.emplace_back(entry.path(),read_bytes(entry.path()));
        }
        previous_directory=video.parent_path();
    }
    CHECK(contains(log,"output video accepted=2 dropped=0 failures=0"));
    CHECK(contains(log,"output telemetry accepted=2 dropped=0 failures=0"));
    CHECK(contains(log,"output imu accepted=41 dropped=0 failures=0"));
    return 0;
}

/** @brief Match ignores prelaunch video; a loaded debug fixture records it continuously. */
int prelaunch_and_debug_follow_original_policy(const app::AppConfig& original) {
    for(bool debug:{false,true}) {
        Fixture fixture;auto app=original;
        if(debug) {
            auto text=read_bytes(std::filesystem::path(MOSAS_SOURCE_DIR)/"config/mosas_guidance.conf");
            const auto pos=text.find("mode=match");CHECK(pos!=std::string::npos);
            text.replace(pos,std::string("mode=match").size(),"mode=debug");
            const auto path=fixture.root/"debug.conf";{std::ofstream file(path);file<<text;}
            std::string error;CHECK(app::AppConfigLoader::load(path,&app,&error));
        }
        auto config=configuration(fixture,app);auto snapshot=metadata(app,debug?702:701);
        Endpoint server(config.socket_directory,snapshot);std::vector<std::string> log;
        app::SimulationRunner runner(config,[&](const auto& line){log.push_back(line);});
        CHECK(advance(runner,[&]{return runner.running();}));
        for(std::int64_t time=0;time<=20'000'000;time+=5'000'000)
            CHECK(server.submit(Imu{3,{0,0,standard_gravity},{}},time));
        CHECK(server.submit(image(snapshot.camera,220),15'000'000));
        CHECK(advance(runner,[&]{return runner.timing().output.samples>=2;}));
        runner.stop();CHECK(runner.last_error().empty());
        CHECK(!contains(log,"MOSAS launch_detected_ns="));
        if (app.algorithms.algorithm == app::GuidanceAlgorithm::png) {
            CHECK(contains(log,"pre_acquisition preset_submitted=0 first_preset_ns=-1"));
        } else {
            CHECK(!contains(log,"pre_acquisition preset_submitted="));
        }
        const auto videos=files_in(fixture.root/"recordings",".avi");
        CHECK(videos.size()==static_cast<std::size_t>(debug));
        std::size_t imu_files=0;
        for(const auto& path:files_in(fixture.root/"recordings",".csv"))
            if(path.filename().string().find("_imu_")!=std::string::npos) {
                ++imu_files;CHECK(lines_in(path).size()==6);
            }
        CHECK(imu_files==1);
        if(debug) {
            cv::VideoCapture capture(videos[0].string());cv::Mat frame;
            CHECK(capture.read(frame));CHECK(frame.cols==640 && frame.rows==480);
            CHECK_NEAR(capture.get(cv::CAP_PROP_FPS),120,0.001);CHECK(!capture.read(frame));
            auto telemetry=videos[0];telemetry.replace_extension(".csv");
            const auto rows=lines_in(telemetry);CHECK(rows.size()==2);
            CHECK(rows[1].find("0,15000000,")==0);
        }
    }
    return 0;
}

/** @brief Invalid output parents and excessive original-policy windows fail before any ready claim. */
int recording_errors_are_visible(const app::AppConfig& original) {
    for(bool oversized:{false,true}) {
        Fixture fixture;auto config=configuration(fixture,original);
        if(oversized)config.app.output.match_window_duration_ms=1'000'000;
        else {
            const auto blocker=fixture.root/"not_a_directory";
            {std::ofstream file(blocker);file<<"block recording parent";}
            config.app.output.recording_path=(blocker/"mosas_guidance.avi").string();
        }
        Endpoint server(config.socket_directory,metadata(original,oversized?802:801));
        std::vector<std::string> log;app::SimulationRunner runner(config,[&](const auto& line){log.push_back(line);});
        CHECK(advance(runner,[&]{return !runner.last_error().empty();}));
        CHECK(!runner.running());CHECK(!contains(log,"MOSAS simulation ready"));
        CHECK(contains(log,"simulation stopped:"));
        if(oversized)CHECK(runner.last_error().find("memory limit")!=std::string::npos);
        else CHECK(runner.last_error().find("directory")!=std::string::npos);
        runner.stop();
    }
    return 0;
}
} // namespace

int main() {
    app::AppConfig original;std::string error;
    const auto path=std::filesystem::path(MOSAS_SOURCE_DIR)/"config/mosas_guidance.conf";
    const auto before=read_bytes(path);
    CHECK(app::AppConfigLoader::load(path,&original,&error));
    for (const auto algorithm : {app::GuidanceAlgorithm::body_vector_pn,
                                 app::GuidanceAlgorithm::png}) {
        for (const auto mode : {runtime::LaunchDetectionMode::acceleration,
                                runtime::LaunchDetectionMode::two_stage}) {
            auto selected = original;
            selected.algorithms.algorithm = algorithm;
            selected.algorithms.attitude.launch_detection_mode = mode;
            CHECK(match_partial_windows_survive_two_sessions(selected) == 0);
        }
    }
    CHECK(prelaunch_and_debug_follow_original_policy(original)==0);
    CHECK(recording_errors_are_visible(original)==0);
    CHECK(read_bytes(path)==before);
    return 0;
}
