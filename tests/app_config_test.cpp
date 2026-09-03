#include <mosas/app/app_config.hpp>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

namespace {

void test_check(bool condition, const char* expression, const char* file,
                int line) {
    if (!condition) {
        throw std::runtime_error(std::string(file) + ":" +
                                 std::to_string(line) +
                                 ": check failed: " + expression);
    }
}

#define TEST_CHECK(condition) \
    test_check(static_cast<bool>(condition), #condition, __FILE__, __LINE__)

class TemporaryConfigFile {
public:
    explicit TemporaryConfigFile(const std::string& contents) {
        path_ = std::filesystem::temp_directory_path() /
                "mosas_app_config_test.conf";
        std::ofstream output(path_);
        if (!output) {
            throw std::runtime_error("无法创建临时配置文件");
        }
        output << contents;
    }

    ~TemporaryConfigFile() {
        std::error_code error;
        std::filesystem::remove(path_, error);
    }

    const std::filesystem::path& path() const { return path_; }

private:
    std::filesystem::path path_;
};

void test_loads_typed_configuration() {
    TemporaryConfigFile file(
        "[camera]\n"
        "device=camera0\n"
        "width=800\n"
        "height=600\n"
        "fps=25\n"
        "mode=hardware_trigger\n"
        "auto_exposure=false\n"
        "exposure_us=10000\n"
        "gain=0\n"
        "undistort=true\n"
        "distortion_k1=0.1\n"
        "distortion_k2=-0.02\n"
        "distortion_p1=0.001\n"
        "distortion_p2=-0.001\n"
        "distortion_k3=0.003\n"
        "matrix_m00=500\n"
        "matrix_m01=0\n"
        "matrix_m02=320\n"
        "matrix_m10=0\n"
        "matrix_m11=500\n"
        "matrix_m12=240\n"
        "matrix_m20=0\n"
        "matrix_m21=0\n"
        "matrix_m22=1\n"
        "[imu]\n"
        "mode=simulated\n"
        "sample_rate_hz=100\n"
        "[command]\n"
        "mode=log\n"
        "[guidance]\n"
        "launch_speed_mps=120\n"
        "max_imu_age_ms=25\n"
        "capture_queue_capacity=2\n"
        "output_queue_capacity=3\n"
        "png_navigation_constant_y=2.5\n"
        "png_navigation_constant_z=4.5\n"
        "[wireless]\n"
        "enable_wireless_stream=false\n"
        "enable_recording=true\n"
        "recording_path=recordings/output.avi\n");

    mosas::app::AppConfig config;
    std::string error;
    TEST_CHECK(mosas::app::AppConfigLoader::load(file.path(), &config, &error));
    TEST_CHECK(error.empty());
    TEST_CHECK(config.camera.device == "camera0");
    TEST_CHECK(config.camera.width == 800);
    TEST_CHECK(config.camera.height == 600);
    TEST_CHECK(config.camera.fps == 25);
    TEST_CHECK(config.camera.capture_mode ==
               mosas::runtime::CameraCaptureMode::hardware_trigger);
    TEST_CHECK(!config.camera.auto_exposure);
    TEST_CHECK(config.camera.exposure_us == 10000.0);
    TEST_CHECK(config.camera.gain == 0.0);
    TEST_CHECK(config.camera.undistort);
    TEST_CHECK(config.camera.distortion.k1 == 0.1);
    TEST_CHECK(config.camera.distortion.k2 == -0.02);
    TEST_CHECK(config.camera.distortion.p1 == 0.001);
    TEST_CHECK(config.camera.distortion.p2 == -0.001);
    TEST_CHECK(config.camera.distortion.k3 == 0.003);
    TEST_CHECK(config.camera.camera_matrix.m00 == 500.0);
    TEST_CHECK(config.camera.camera_matrix.m02 == 320.0);
    TEST_CHECK(config.camera.camera_matrix.m11 == 500.0);
    TEST_CHECK(config.camera.camera_matrix.m12 == 240.0);
    TEST_CHECK(config.guidance.camera_intrinsics.fx == 500.0);
    TEST_CHECK(config.guidance.camera_intrinsics.fy == 500.0);
    TEST_CHECK(config.guidance.camera_intrinsics.cx == 320.0);
    TEST_CHECK(config.guidance.camera_intrinsics.cy == 240.0);
    TEST_CHECK(config.imu.sample_rate_hz == 100.0);
    TEST_CHECK(config.guidance.launch_speed_mps == 120.0);
    TEST_CHECK(config.guidance.png_guidance.navigation_constant_y == 2.5);
    TEST_CHECK(config.guidance.png_guidance.navigation_constant_z == 4.5);
    TEST_CHECK(config.guidance.max_imu_age_ns == 25'000'000);
    TEST_CHECK(config.guidance.capture_queue_capacity == 2);
    TEST_CHECK(config.guidance.output_queue_capacity == 3);
    TEST_CHECK(!config.wireless.enable_wireless_stream);
    TEST_CHECK(config.wireless.enable_recording);
    TEST_CHECK(config.wireless.recording_path ==
               file.path().parent_path() / "recordings/output.avi");
}

void test_applies_defaults_for_omitted_values() {
    TemporaryConfigFile file("# 只覆盖一个字段\n[camera]\nwidth=640\n");

    mosas::app::AppConfig config;
    std::string error;
    TEST_CHECK(mosas::app::AppConfigLoader::load(file.path(), &config, &error));
    TEST_CHECK(config.camera.width == 640);
    TEST_CHECK(config.camera.height == 480);
    TEST_CHECK(config.camera.fps == 30);
    TEST_CHECK(config.camera.auto_exposure);
    TEST_CHECK(config.camera.exposure_us == 10000.0);
    TEST_CHECK(config.camera.gain == 0.0);
    TEST_CHECK(!config.camera.undistort);
    TEST_CHECK(config.imu.sample_rate_hz == 200.0);
    TEST_CHECK(config.guidance.vision_config.initial_roi.width == 320);
    TEST_CHECK(config.guidance.png_guidance.navigation_constant_y == 3.0);
    TEST_CHECK(config.guidance.png_guidance.navigation_constant_z == 3.0);
    TEST_CHECK(config.wireless.stream.rtsp_url ==
               "rtsp://127.0.0.1:8554/mosas");
}

void test_rejects_invalid_configuration_with_line_number() {
    TemporaryConfigFile file(
        "[camera]\n"
        "width=not-a-number\n");

    mosas::app::AppConfig config;
    std::string error;
    TEST_CHECK(!mosas::app::AppConfigLoader::load(file.path(), &config, &error));
    TEST_CHECK(error.find(":2:") != std::string::npos);
}

void expect_invalid(const std::string& contents) {
    TemporaryConfigFile file(contents);
    mosas::app::AppConfig config;
    std::string error;
    TEST_CHECK(!mosas::app::AppConfigLoader::load(file.path(), &config, &error));
    TEST_CHECK(!error.empty());
}

void test_rejects_strict_syntax_and_ranges() {
    expect_invalid("[]\nwidth=640\n");
    expect_invalid("[camera\nwidth=640\n");
    expect_invalid("[camera]\nunknown=value\n");
    expect_invalid("[camera]\nwidth=640\nwidth=800\n");
    expect_invalid("[camera]\nwidth\n");
    expect_invalid("[camera]\nwidth=\n");
    expect_invalid("[wireless]\nenable_recording=maybe\n");
    expect_invalid("[camera]\nmode=triggered\n");
    expect_invalid("[camera]\nwidth=641\n");
    expect_invalid("[guidance]\ncapture_queue_capacity=0\n");
    expect_invalid("[guidance]\npng_navigation_constant_y=0\n");
    expect_invalid("[guidance]\npng_navigation_constant_z=0\n");
    expect_invalid("[camera]\nexposure_us=0\n");
    expect_invalid("[camera]\ngain=-1\n");
    expect_invalid("[camera]\ndistortion_k1=nan\n");
}

}  // namespace

int main() {
    test_loads_typed_configuration();
    test_applies_defaults_for_omitted_values();
    test_rejects_invalid_configuration_with_line_number();
    test_rejects_strict_syntax_and_ranges();
    return 0;
}
