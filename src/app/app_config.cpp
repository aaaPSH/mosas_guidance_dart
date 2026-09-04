#include <mosas/app/app_config.hpp>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <set>
#include <sstream>
#include <string_view>
#include <utility>

namespace mosas::app {
namespace {

std::string trim(std::string value) {
    const auto first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
        return {};
    }
    const auto last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1);
}

bool fail(const std::filesystem::path& path, std::size_t line,
          const std::string& message, std::string* error) {
    if (error != nullptr) {
        std::ostringstream stream;
        stream << path << ":" << line << ": " << message;
        *error = stream.str();
    }
    return false;
}

bool fail_global(const std::filesystem::path& path, const std::string& message,
                 std::string* error) {
    if (error != nullptr) {
        *error = path.string() + ": " + message;
    }
    return false;
}

bool parse_bool(const std::string& text, bool* value) {
    if (text == "true" || text == "1") {
        *value = true;
        return true;
    }
    if (text == "false" || text == "0") {
        *value = false;
        return true;
    }
    return false;
}

bool parse_int(const std::string& text, int* value) {
    if (text.empty()) {
        return false;
    }
    errno = 0;
    char* end = nullptr;
    const long parsed = std::strtol(text.c_str(), &end, 10);
    if (errno != 0 || end == text.c_str() || *end != '\0' ||
        parsed < std::numeric_limits<int>::min() ||
        parsed > std::numeric_limits<int>::max()) {
        return false;
    }
    *value = static_cast<int>(parsed);
    return true;
}

bool parse_size(const std::string& text, std::size_t* value) {
    if (text.empty() || text.front() == '-') {
        return false;
    }
    errno = 0;
    char* end = nullptr;
    const unsigned long long parsed = std::strtoull(text.c_str(), &end, 10);
    if (errno != 0 || end == text.c_str() || *end != '\0' ||
        parsed > std::numeric_limits<std::size_t>::max()) {
        return false;
    }
    *value = static_cast<std::size_t>(parsed);
    return true;
}

bool parse_double(const std::string& text, double* value) {
    if (text.empty()) {
        return false;
    }
    errno = 0;
    char* end = nullptr;
    const double parsed = std::strtod(text.c_str(), &end);
    if (errno != 0 || end == text.c_str() || *end != '\0' ||
        !std::isfinite(parsed)) {
        return false;
    }
    *value = parsed;
    return true;
}

bool parse_uint8(const std::string& text, uint8_t* value) {
    int parsed = 0;
    if (!parse_int(text, &parsed) || parsed < 0 || parsed > 255) {
        return false;
    }
    *value = static_cast<uint8_t>(parsed);
    return true;
}

bool parse_timestamp_ms(const std::string& text, runtime::TimestampNs* value) {
    int parsed = 0;
    if (!parse_int(text, &parsed) || parsed <= 0) {
        return false;
    }
    *value = static_cast<runtime::TimestampNs>(parsed) * 1'000'000;
    return true;
}

bool parse_mode(const std::string& text, runtime::CameraCaptureMode* value) {
    if (text == "free_run") {
        *value = runtime::CameraCaptureMode::free_run;
        return true;
    }
    if (text == "hardware_trigger") {
        *value = runtime::CameraCaptureMode::hardware_trigger;
        return true;
    }
    return false;
}

bool parse_key(const std::filesystem::path& path, std::size_t line,
               const std::string& section, const std::string& key,
               const std::string& value, AppConfig* config,
               std::set<std::string>* seen, std::string* error) {
    const std::string qualified_key = section + "." + key;
    if (!seen->insert(qualified_key).second) {
        return fail(path, line, "配置键重复: " + qualified_key, error);
    }

    auto invalid = [&] {
        return fail(path, line, "配置值无效: " + qualified_key + "=" + value,
                    error);
    };
    int integer = 0;
    double decimal = 0.0;

    if (section == "camera") {
        if (key == "device") {
            config->camera.device = value;
        } else if (key == "width" && parse_int(value, &integer)) {
            config->camera.width = integer;
        } else if (key == "height" && parse_int(value, &integer)) {
            config->camera.height = integer;
        } else if (key == "fps" && parse_int(value, &integer)) {
            config->camera.fps = integer;
        } else if (key == "pixel_format") {
            config->camera.pixel_format = value;
        } else if (key == "auto_exposure" &&
                   parse_bool(value, &config->camera.auto_exposure)) {
        } else if (key == "exposure_us" &&
                   parse_double(value, &decimal)) {
            config->camera.exposure_us = decimal;
        } else if (key == "gain" && parse_double(value, &decimal)) {
            config->camera.gain = decimal;
        } else if (key == "undistort" &&
                   parse_bool(value, &config->camera.undistort)) {
        } else if (key == "distortion_k1" && parse_double(value, &decimal)) {
            config->camera.distortion.k1 = decimal;
        } else if (key == "distortion_k2" && parse_double(value, &decimal)) {
            config->camera.distortion.k2 = decimal;
        } else if (key == "distortion_p1" && parse_double(value, &decimal)) {
            config->camera.distortion.p1 = decimal;
        } else if (key == "distortion_p2" && parse_double(value, &decimal)) {
            config->camera.distortion.p2 = decimal;
        } else if (key == "distortion_k3" && parse_double(value, &decimal)) {
            config->camera.distortion.k3 = decimal;
        } else if (key == "mode" && parse_mode(value, &config->camera.capture_mode)) {
        } else if (key.rfind("matrix_m", 0) == 0 && key.size() == 10) {
            const int row = key[8] - '0';
            const int column = key[9] - '0';
            if (row >= 0 && row < 3 && column >= 0 && column < 3 &&
                parse_double(value, &decimal)) {
                double* matrix[] = {
                    &config->camera.camera_matrix.m00,
                    &config->camera.camera_matrix.m01,
                    &config->camera.camera_matrix.m02,
                    &config->camera.camera_matrix.m10,
                    &config->camera.camera_matrix.m11,
                    &config->camera.camera_matrix.m12,
                    &config->camera.camera_matrix.m20,
                    &config->camera.camera_matrix.m21,
                    &config->camera.camera_matrix.m22,
                };
                *matrix[row * 3 + column] = decimal;
                config->camera.camera_matrix_configured = true;
            } else {
                return invalid();
            }
        } else {
            return key == "device" ? invalid()
                                    : fail(path, line, "未知配置键或值无效: " +
                                                  qualified_key,
                                           error);
        }
        return true;
    }

    if (section == "imu") {
        if (key == "mode" && value == "simulated") {
            config->imu.mode = ImuMode::simulated;
        } else if (key == "skip_self_check") {
            return parse_bool(value, &config->imu.skip_self_check) || invalid();
        } else if (key == "sample_rate_hz" && parse_double(value, &decimal)) {
            config->imu.sample_rate_hz = decimal;
        } else {
            return invalid();
        }
        return true;
    }

    if (section == "command") {
        if (key == "mode" && value == "log") {
            config->command.mode = CommandMode::log;
        } else {
            return invalid();
        }
        return true;
    }

    if (section == "guidance") {
        auto set_int = [&](int* destination) {
            if (!parse_int(value, &integer)) return false;
            *destination = integer;
            return true;
        };
        auto set_size = [&](std::size_t* destination) {
            if (!parse_size(value, destination)) return false;
            return true;
        };
        auto set_double = [&](double* destination) {
            if (!parse_double(value, &decimal)) return false;
            *destination = decimal;
            return true;
        };
        if (key == "launch_speed_mps") return set_double(&config->guidance.launch_speed_mps) || invalid();
        if (key == "history_capacity") return set_size(&config->guidance.history_capacity) || invalid();
        if (key == "max_imu_age_ms") return parse_timestamp_ms(value, &config->guidance.max_imu_age_ns) || invalid();
        if (key == "capture_queue_capacity") return set_size(&config->guidance.capture_queue_capacity) || invalid();
        if (key == "output_queue_capacity") return set_size(&config->guidance.output_queue_capacity) || invalid();
        if (key == "initial_roi_x") return set_int(&config->guidance.vision_config.initial_roi.x) || invalid();
        if (key == "initial_roi_y") return set_int(&config->guidance.vision_config.initial_roi.y) || invalid();
        if (key == "initial_roi_width") return set_int(&config->guidance.vision_config.initial_roi.width) || invalid();
        if (key == "initial_roi_height") return set_int(&config->guidance.vision_config.initial_roi.height) || invalid();
        if (key == "h_min") return parse_uint8(value, &config->guidance.vision_config.threshold.h_min) || invalid();
        if (key == "h_max") return parse_uint8(value, &config->guidance.vision_config.threshold.h_max) || invalid();
        if (key == "s_min") return parse_uint8(value, &config->guidance.vision_config.threshold.s_min) || invalid();
        if (key == "s_max") return parse_uint8(value, &config->guidance.vision_config.threshold.s_max) || invalid();
        if (key == "v_min") return parse_uint8(value, &config->guidance.vision_config.threshold.v_min) || invalid();
        if (key == "v_max") return parse_uint8(value, &config->guidance.vision_config.threshold.v_max) || invalid();
        if (key == "found_range_x") return set_int(&config->guidance.vision_config.found_range_x) || invalid();
        if (key == "found_range_y") return set_int(&config->guidance.vision_config.found_range_y) || invalid();
        if (key == "min_blob_area") return set_int(&config->guidance.vision_config.min_blob_area) || invalid();
        if (key == "min_aspect_ratio") return set_double(&config->guidance.vision_config.min_aspect_ratio) || invalid();
        if (key == "min_fill_ratio") return set_double(&config->guidance.vision_config.min_fill_ratio) || invalid();
        if (key == "camera_fx") return set_double(&config->guidance.camera_intrinsics.fx) || invalid();
        if (key == "camera_fy") return set_double(&config->guidance.camera_intrinsics.fy) || invalid();
        if (key == "camera_cx") return set_double(&config->guidance.camera_intrinsics.cx) || invalid();
        if (key == "camera_cy") return set_double(&config->guidance.camera_intrinsics.cy) || invalid();
        if (key == "los_process_noise") return set_double(&config->guidance.los_rate_filter.process_noise) || invalid();
        if (key == "los_measurement_noise") return set_double(&config->guidance.los_rate_filter.measurement_noise) || invalid();
        if (key == "los_initial_covariance") return set_double(&config->guidance.los_rate_filter.initial_covariance) || invalid();
        if (key == "los_enable_filter") return parse_bool(value, &config->guidance.los_rate_filter.enable_filter) || invalid();
        if (key == "png_navigation_constant_y") return set_double(&config->guidance.png_guidance.navigation_constant_y) || invalid();
        if (key == "png_navigation_constant_z") return set_double(&config->guidance.png_guidance.navigation_constant_z) || invalid();
        if (key == "png_gravity") return set_double(&config->guidance.png_guidance.gravity) || invalid();
        if (key == "phase_min_static_samples") return set_int(&config->guidance.phase_detector.min_static_samples) || invalid();
        if (key == "phase_min_static_duration_ms") return parse_timestamp_ms(value, &config->guidance.phase_detector.min_static_duration_ns) || invalid();
        if (key == "phase_max_static_axis_variation") return set_double(&config->guidance.phase_detector.max_static_axis_variation) || invalid();
        if (key == "phase_launch_direction") return set_int(&config->guidance.phase_detector.launch_direction) || invalid();
        if (key == "phase_ejection_start_threshold") return set_double(&config->guidance.phase_detector.ejection_start_threshold) || invalid();
        if (key == "phase_ejection_confirm_duration_ms") return parse_timestamp_ms(value, &config->guidance.phase_detector.ejection_confirm_duration_ns) || invalid();
        if (key == "phase_free_flight_release_threshold") return set_double(&config->guidance.phase_detector.free_flight_release_threshold) || invalid();
        if (key == "phase_free_flight_confirm_duration_ms") return parse_timestamp_ms(value, &config->guidance.phase_detector.free_flight_confirm_duration_ns) || invalid();
        if (key.rfind("camera_to_body_m", 0) == 0 && key.size() == 18) {
            const int row = key[16] - '0';
            const int column = key[17] - '0';
            if (row >= 0 && row < 3 && column >= 0 && column < 3 &&
                parse_double(value, &decimal)) {
                double* matrix[] = {
                    &config->guidance.camera_to_body.m00,
                    &config->guidance.camera_to_body.m01,
                    &config->guidance.camera_to_body.m02,
                    &config->guidance.camera_to_body.m10,
                    &config->guidance.camera_to_body.m11,
                    &config->guidance.camera_to_body.m12,
                    &config->guidance.camera_to_body.m20,
                    &config->guidance.camera_to_body.m21,
                    &config->guidance.camera_to_body.m22,
                };
                *matrix[row * 3 + column] = decimal;
                return true;
            }
        }
        return invalid();
    }

    if (section == "wireless") {
        auto set_int = [&](int* destination) {
            if (!parse_int(value, &integer)) return false;
            *destination = integer;
            return true;
        };
        auto set_double = [&](double* destination) {
            if (!parse_double(value, &decimal)) return false;
            *destination = decimal;
            return true;
        };
        if (key == "enable_wireless_stream") return parse_bool(value, &config->wireless.enable_wireless_stream) || invalid();
        if (key == "enable_recording") return parse_bool(value, &config->wireless.enable_recording) || invalid();
        if (key == "recording_path") { config->wireless.recording_path = value; return true; }
        if (key == "recording_fps") return set_double(&config->wireless.recording_fps) || invalid();
        if (key == "draw_visualization") return parse_bool(value, &config->wireless.draw_visualization) || invalid();
        if (key == "rtsp_url") { config->wireless.stream.rtsp_url = value; return true; }
        if (key == "rtsp_transport") { config->wireless.stream.rtsp_transport = value; return true; }
        if (key == "width") return set_int(&config->wireless.stream.width) || invalid();
        if (key == "height") return set_int(&config->wireless.stream.height) || invalid();
        if (key == "fps") return set_int(&config->wireless.stream.fps) || invalid();
        if (key == "bitrate") return set_int(&config->wireless.stream.bitrate) || invalid();
        if (key == "gop_size") return set_int(&config->wireless.stream.gop_size) || invalid();
        if (key == "rtsp_timeout_ms") {
            int timeout_ms = 0;
            if (!parse_int(value, &timeout_ms) || timeout_ms <= 0) return invalid();
            config->wireless.stream.rtsp_timeout_us =
                static_cast<std::int64_t>(timeout_ms) * 1'000;
            return true;
        }
        if (key == "encoder_name") { config->wireless.stream.encoder_name = value; return true; }
        return invalid();
    }

    return fail(path, line, "未知配置节: " + section, error);
}

bool validate(const std::filesystem::path& path, const AppConfig& config,
              std::string* error) {
    const auto& matrix = config.camera.camera_matrix;
    if (config.camera.device.empty() || config.camera.width <= 0 ||
        config.camera.height <= 0 || config.camera.width % 2 != 0 ||
        config.camera.height % 2 != 0 || config.camera.fps <= 0 ||
        (config.camera.pixel_format != "auto" &&
         config.camera.pixel_format.size() != 4) ||
        !std::isfinite(config.camera.exposure_us) ||
        config.camera.exposure_us <= 0.0 || !std::isfinite(config.camera.gain) ||
        config.camera.gain < 0.0 || !std::isfinite(matrix.m00) ||
        !std::isfinite(matrix.m01) || !std::isfinite(matrix.m02) ||
        !std::isfinite(matrix.m10) || !std::isfinite(matrix.m11) ||
        !std::isfinite(matrix.m12) || !std::isfinite(matrix.m20) ||
        !std::isfinite(matrix.m21) || !std::isfinite(matrix.m22) ||
        matrix.m00 <= 0.0 || matrix.m11 <= 0.0 || matrix.m22 == 0.0) {
        return fail_global(path, "摄像头参数无效", error);
    }
    const auto& distortion = config.camera.distortion;
    if (!std::isfinite(distortion.k1) || !std::isfinite(distortion.k2) ||
        !std::isfinite(distortion.p1) || !std::isfinite(distortion.p2) ||
        !std::isfinite(distortion.k3)) {
        return fail_global(path, "畸变参数无效", error);
    }
    if (config.imu.mode != ImuMode::simulated ||
        !std::isfinite(config.imu.sample_rate_hz) ||
        config.imu.sample_rate_hz <= 0.0) {
        return fail_global(path, "IMU 参数无效", error);
    }
    const auto& vision = config.guidance.vision_config;
    if (config.guidance.launch_speed_mps <= 0.0 ||
        config.guidance.history_capacity == 0 ||
        config.guidance.max_imu_age_ns <= 0 ||
        config.guidance.capture_queue_capacity == 0 ||
        config.guidance.output_queue_capacity == 0 ||
        vision.initial_roi.width <= 0 || vision.initial_roi.height <= 0 ||
        vision.found_range_x <= 0 || vision.found_range_y <= 0 ||
        vision.min_blob_area <= 0 || vision.min_aspect_ratio <= 0.0 ||
        vision.min_fill_ratio <= 0.0 ||
        config.guidance.camera_intrinsics.fx <= 0.0 ||
        config.guidance.camera_intrinsics.fy <= 0.0 ||
        config.guidance.los_rate_filter.measurement_noise <= 0.0 ||
        config.guidance.png_guidance.navigation_constant_y <= 0.0 ||
        config.guidance.png_guidance.navigation_constant_z <= 0.0 ||
        config.guidance.png_guidance.gravity <= 0.0) {
        return fail_global(path, "制导参数无效", error);
    }
    if (config.wireless.enable_recording &&
        config.wireless.recording_path.empty()) {
        return fail_global(path, "启用录像时 recording_path 不能为空", error);
    }
    if (config.wireless.recording_fps <= 0.0 ||
        config.wireless.stream.width <= 0 || config.wireless.stream.height <= 0 ||
        config.wireless.stream.width % 2 != 0 ||
        config.wireless.stream.height % 2 != 0 ||
        config.wireless.stream.fps <= 0 || config.wireless.stream.bitrate <= 0 ||
        config.wireless.stream.gop_size <= 0 ||
        config.wireless.stream.rtsp_timeout_us <= 0) {
        return fail_global(path, "无线图传参数无效", error);
    }
    return true;
}

}  // namespace

bool AppConfigLoader::load(const std::filesystem::path& path, AppConfig* config,
                           std::string* error) {
    if (config == nullptr) {
        return fail_global(path, "配置输出指针为空", error);
    }
    std::ifstream input(path);
    if (!input) {
        return fail_global(path, "无法读取配置文件", error);
    }

    AppConfig loaded;
    std::string section;
    std::set<std::string> seen;
    std::string line_text;
    std::size_t line_number = 0;
    while (std::getline(input, line_text)) {
        ++line_number;
        const std::string line = trim(line_text);
        if (line.empty() || line.front() == '#') {
            continue;
        }
        if (line.front() == '[' || line.back() == ']') {
            if (line.size() < 3 || line.front() != '[' || line.back() != ']') {
                return fail(path, line_number, "配置节格式无效", error);
            }
            section = trim(line.substr(1, line.size() - 2));
            if (section.empty() ||
                (section != "camera" && section != "imu" &&
                 section != "command" && section != "guidance" &&
                 section != "wireless")) {
                return fail(path, line_number, "未知配置节: " + section, error);
            }
            continue;
        }
        const auto separator = line.find('=');
        if (section.empty()) {
            return fail(path, line_number, "配置键必须位于配置节中", error);
        }
        if (separator == std::string::npos) {
            return fail(path, line_number, "配置行缺少 '='", error);
        }
        const std::string key = trim(line.substr(0, separator));
        const std::string value = trim(line.substr(separator + 1));
        if (key.empty() || value.empty()) {
            return fail(path, line_number, "配置键和值不能为空", error);
        }
        if (!parse_key(path, line_number, section, key, value, &loaded, &seen,
                       error)) {
            return false;
        }
    }

    if (!input.eof() && input.fail()) {
        return fail_global(path, "读取配置文件时发生错误", error);
    }
    if (loaded.camera.camera_matrix_configured) {
        loaded.guidance.camera_intrinsics.fx = loaded.camera.camera_matrix.m00;
        loaded.guidance.camera_intrinsics.fy = loaded.camera.camera_matrix.m11;
        loaded.guidance.camera_intrinsics.cx = loaded.camera.camera_matrix.m02;
        loaded.guidance.camera_intrinsics.cy = loaded.camera.camera_matrix.m12;
    } else {
        loaded.camera.camera_matrix.m00 = loaded.guidance.camera_intrinsics.fx;
        loaded.camera.camera_matrix.m02 = loaded.guidance.camera_intrinsics.cx;
        loaded.camera.camera_matrix.m11 = loaded.guidance.camera_intrinsics.fy;
        loaded.camera.camera_matrix.m12 = loaded.guidance.camera_intrinsics.cy;
    }
    if (!validate(path, loaded, error)) {
        return false;
    }
    if (!loaded.wireless.recording_path.empty()) {
        const std::filesystem::path absolute_path =
            std::filesystem::absolute(path).parent_path() /
            loaded.wireless.recording_path;
        loaded.wireless.recording_path = absolute_path.lexically_normal().string();
    }
    loaded.guidance.camera_capture =
        runtime::CameraCaptureConfig{loaded.camera.capture_mode};
    *config = std::move(loaded);
    if (error != nullptr) {
        error->clear();
    }
    return true;
}

}  // namespace mosas::app
