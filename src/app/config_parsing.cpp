#include "config_parsing.hpp"

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

namespace mosas::app::config_parsing {

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

bool parse_uint32(const std::string& text, uint32_t* value) {
    if (text.empty() || text.front() == '-') {
        return false;
    }
    errno = 0;
    char* end = nullptr;
    const unsigned long long parsed = std::strtoull(text.c_str(), &end, 10);
    if (errno != 0 || end == text.c_str() || *end != '\0' ||
        parsed > std::numeric_limits<uint32_t>::max()) {
        return false;
    }
    *value = static_cast<uint32_t>(parsed);
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

bool parse_stop_bits(const std::string& text,
                     serial_package::StopBits* value) {
    if (text == "1" || text == "one") {
        *value = serial_package::StopBits::one;
        return true;
    }
    if (text == "2" || text == "two") {
        *value = serial_package::StopBits::two;
        return true;
    }
    return false;
}

bool parse_parity(const std::string& text, serial_package::Parity* value) {
    if (text == "none") {
        *value = serial_package::Parity::none;
        return true;
    }
    if (text == "even") {
        *value = serial_package::Parity::even;
        return true;
    }
    if (text == "odd") {
        *value = serial_package::Parity::odd;
        return true;
    }
    return false;
}

bool parse_flow_control(const std::string& text,
                        serial_package::FlowControl* value) {
    if (text == "none") {
        *value = serial_package::FlowControl::none;
        return true;
    }
    if (text == "software") {
        *value = serial_package::FlowControl::software;
        return true;
    }
    if (text == "hardware") {
        *value = serial_package::FlowControl::hardware;
        return true;
    }
    return false;
}

bool parse_timestamp_ms(const std::string& text, runtime::TimestampNs* value) {
    int parsed = 0;
    if (!parse_int(text, &parsed) || parsed <= 0) {
        return false;
    }
    *value = static_cast<runtime::TimestampNs>(parsed) * 1'000'000;
    return true;
}

bool parse_timestamp_ns(const std::string& text, runtime::TimestampNs* value) {
    if (text.empty()) {
        return false;
    }
    errno = 0;
    char* end = nullptr;
    const long long parsed = std::strtoll(text.c_str(), &end, 10);
    if (errno != 0 || end == text.c_str() || *end != '\0') {
        return false;
    }
    *value = static_cast<runtime::TimestampNs>(parsed);
    return true;
}

bool parse_camera_timestamp_domain(
    const std::string& text, runtime::CameraTimestampDomain* value) {
    if (text == "unknown") {
        *value = runtime::CameraTimestampDomain::unknown;
        return true;
    }
    if (text == "monotonic") {
        *value = runtime::CameraTimestampDomain::monotonic;
        return true;
    }
    if (text == "realtime") {
        *value = runtime::CameraTimestampDomain::realtime;
        return true;
    }
    return false;
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

bool parse_output_mode(const std::string& text, output::OutputMode* value) {
    if (text == "rtsp") {
        *value = output::OutputMode::rtsp;
        return true;
    }
    if (text == "debug") {
        *value = output::OutputMode::debug;
        return true;
    }
    if (text == "match") {
        *value = output::OutputMode::match;
        return true;
    }
    return false;
}

bool valid_pixel_format(const std::string& value) {
    return value == "auto" || value == "MJPEG" || value == "MJPG" ||
           value == "YUYV";
}

bool valid_dart_from_camera(
    const domain::Rotation3<domain::DartFrameTag,
                            domain::CameraFrameTag>& rotation) {
    const auto& matrix = rotation.matrix();
    for (const double value : matrix) {
        if (!std::isfinite(value)) {
            return false;
        }
    }
    const auto dot_columns = [&matrix](std::size_t lhs, std::size_t rhs) {
        return matrix[lhs] * matrix[rhs] +
               matrix[3 + lhs] * matrix[3 + rhs] +
               matrix[6 + lhs] * matrix[6 + rhs];
    };
    const auto column_norm = [&dot_columns](std::size_t column) {
        return std::sqrt(dot_columns(column, column));
    };
    const double determinant =
        matrix[0] * (matrix[4] * matrix[8] - matrix[5] * matrix[7]) -
        matrix[1] * (matrix[3] * matrix[8] - matrix[5] * matrix[6]) +
        matrix[2] * (matrix[3] * matrix[7] - matrix[4] * matrix[6]);
    return std::abs(column_norm(0) - 1.0) <= 1e-6 &&
           std::abs(column_norm(1) - 1.0) <= 1e-6 &&
           std::abs(column_norm(2) - 1.0) <= 1e-6 &&
           std::abs(dot_columns(0, 1)) <= 1e-6 &&
           std::abs(dot_columns(0, 2)) <= 1e-6 &&
           std::abs(dot_columns(1, 2)) <= 1e-6 &&
           std::abs(determinant - 1.0) <= 1e-6;
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
        } else if (key == "device_id" &&
                   parse_uint32(value, &config->camera.device_id)) {
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
        } else if (key == "callback_queue_capacity" &&
                   parse_size(value, &config->camera.callback_queue_capacity)) {
        } else if (key == "frame_timestamp_domain" &&
                   parse_camera_timestamp_domain(
                       value, &config->camera.frame_timestamp_domain)) {
        } else if (key == "camera_timestamp_bias_ns" &&
                   parse_timestamp_ns(value,
                                      &config->camera.camera_timestamp_bias_ns)) {
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
        } else if (key == "mode" && value == "serial") {
            config->imu.mode = ImuMode::serial;
        } else if (key == "sample_rate_hz" && parse_double(value, &decimal)) {
            config->imu.sample_rate_hz = decimal;
        } else if (key == "measurement_latency_ns") {
            return parse_timestamp_ns(value,
                                      &config->imu.measurement_latency_ns) ||
                   invalid();
        } else if (key == "receive_realtime_priority") {
            return parse_int(value,
                             &config->imu.receive_thread.realtime_priority) ||
                   invalid();
        } else if (key == "require_realtime_priority") {
            return parse_bool(
                       value, &config->imu.receive_thread.require_realtime_priority) ||
                   invalid();
        } else {
            return invalid();
        }
        return true;
    }

    if (section == "command") {
        if (key == "mode" && value == "log") {
            config->command.mode = CommandMode::log;
        } else if (key == "mode" && value == "serial") {
            config->command.mode = CommandMode::serial;
        } else {
            return invalid();
        }
        return true;
    }

    if (section == "serial") {
        if (key == "device") {
            config->serial.device = value;
        } else if (key == "baud_rate" &&
                   parse_uint32(value, &config->serial.baud_rate)) {
        } else if (key == "data_bits" &&
                   parse_uint8(value, &config->serial.data_bits)) {
        } else if (key == "stop_bits" &&
                   parse_stop_bits(value, &config->serial.stop_bits)) {
        } else if (key == "parity" &&
                   parse_parity(value, &config->serial.parity)) {
        } else if (key == "flow_control" &&
                   parse_flow_control(value, &config->serial.flow_control)) {
        } else if (key == "read_timeout_ms" &&
                   parse_int(value, &config->serial.read_timeout_ms)) {
        } else if (key == "write_timeout_ms" &&
                   parse_int(value, &config->serial.write_timeout_ms)) {
        } else if (key == "enable_interval_statistics") {
            return parse_bool(value, &config->enable_serial_interval_statistics) ||
                   invalid();
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
        if (key == "launch_speed_mps") return set_double(&config->algorithms.attitude.launch_speed_mps) || invalid();
        if (key == "history_capacity") return set_size(&config->guidance.history_capacity) || invalid();
        if (key == "max_imu_age_ms") return parse_timestamp_ms(value, &config->guidance.max_imu_age_ns) || invalid();
        if (key == "max_camera_future_ms") return parse_timestamp_ms(value, &config->guidance.max_camera_future_ns) || invalid();
        if (key == "max_camera_age_ms") return parse_timestamp_ms(value, &config->guidance.max_camera_age_ns) || invalid();
        if (key == "capture_queue_capacity") return set_size(&config->guidance.capture_queue_capacity) || invalid();
        if (key == "output_queue_capacity") return set_size(&config->guidance.output_queue_capacity) || invalid();
        auto& algorithms = config->algorithms;
        if (key == "guidance_algorithm") {
            if (value == "png") algorithms.algorithm = GuidanceAlgorithm::png;
            else if (value == "body_vector_pn") algorithms.algorithm = GuidanceAlgorithm::body_vector_pn;
            else return invalid();
            return true;
        }
        if (key == "body_vector_navigation_constant") return set_double(&algorithms.body_vector_pn.navigation_constant) || invalid();
        if (key == "body_vector_los_sigma_a") return set_double(&algorithms.body_vector_filter.sigma_a) || invalid();
        if (key == "body_vector_los_sigma_l") return set_double(&algorithms.body_vector_filter.sigma_l) || invalid();
        if (key == "body_vector_los_initial_rate_covariance") return set_double(&algorithms.body_vector_filter.initial_rate_covariance) || invalid();
        if (key == "body_vector_los_maximum_gap_ms") {
            if (!parse_double(value, &decimal)) return invalid();
            algorithms.body_vector_filter.maximum_gap_seconds = decimal * 0.001;
            return true;
        }
        if (key == "body_vector_maximum_command_overload") return set_double(&algorithms.body_vector_pn.maximum_command_overload) || invalid();
        if (key == "body_vector_maximum_command_acceleration") {
            if (value == "inf") {
                algorithms.body_vector_pn.maximum_command_acceleration = std::numeric_limits<double>::infinity();
                return true;
            }
            return set_double(&algorithms.body_vector_pn.maximum_command_acceleration) || invalid();
        }
        if (key == "initial_roi_x") return set_int(&algorithms.vision_config.initial_roi.x) || invalid();
        if (key == "initial_roi_y") return set_int(&algorithms.vision_config.initial_roi.y) || invalid();
        if (key == "initial_roi_width") return set_int(&algorithms.vision_config.initial_roi.width) || invalid();
        if (key == "initial_roi_height") return set_int(&algorithms.vision_config.initial_roi.height) || invalid();
        if (key == "h_min") return parse_uint8(value, &algorithms.vision_config.threshold.h_min) || invalid();
        if (key == "h_max") return parse_uint8(value, &algorithms.vision_config.threshold.h_max) || invalid();
        if (key == "s_min") return parse_uint8(value, &algorithms.vision_config.threshold.s_min) || invalid();
        if (key == "s_max") return parse_uint8(value, &algorithms.vision_config.threshold.s_max) || invalid();
        if (key == "v_min") return parse_uint8(value, &algorithms.vision_config.threshold.v_min) || invalid();
        if (key == "v_max") return parse_uint8(value, &algorithms.vision_config.threshold.v_max) || invalid();
        if (key == "found_range_x") return set_int(&algorithms.vision_config.found_range_x) || invalid();
        if (key == "found_range_y") return set_int(&algorithms.vision_config.found_range_y) || invalid();
        if (key == "min_blob_area") return set_int(&algorithms.vision_config.min_blob_area) || invalid();
        if (key == "min_aspect_ratio") return set_double(&algorithms.vision_config.min_aspect_ratio) || invalid();
        if (key == "min_fill_ratio") return set_double(&algorithms.vision_config.min_fill_ratio) || invalid();
        if (key == "camera_fx") return set_double(&config->guidance.camera_intrinsics.fx) || invalid();
        if (key == "camera_fy") return set_double(&config->guidance.camera_intrinsics.fy) || invalid();
        if (key == "camera_cx") return set_double(&config->guidance.camera_intrinsics.cx) || invalid();
        if (key == "camera_cy") return set_double(&config->guidance.camera_intrinsics.cy) || invalid();
        if (key == "los_kalman_sigma_a" || key == "los_process_noise") return set_double(&algorithms.los_rate_filter.sigma_a) || invalid();
        if (key == "los_kalman_sigma_q" || key == "los_measurement_noise") return set_double(&algorithms.los_rate_filter.sigma_q) || invalid();
        if (key == "los_kalman_sigma_a_pitch") return set_double(&algorithms.los_rate_filter.sigma_a_pitch) || invalid();
        if (key == "los_kalman_sigma_a_yaw") return set_double(&algorithms.los_rate_filter.sigma_a_yaw) || invalid();
        if (key == "los_initial_covariance") return set_double(&algorithms.los_rate_filter.initial_covariance) || invalid();
        if (key == "los_enable_filter") return parse_bool(value, &algorithms.los_rate_filter.enable_filter) || invalid();
        if (key == "attitude_stationary_acceleration_tolerance_mps2") return set_double(&algorithms.attitude.stationary_acceleration_tolerance_mps2) || invalid();
        if (key == "attitude_gyro_stationary_threshold_rad_s") return set_double(&algorithms.attitude.gyro_stationary_threshold_rad_s) || invalid();
        if (key == "attitude_gyro_bias_filter_alpha") return set_double(&algorithms.attitude.gyro_bias_filter_alpha) || invalid();
        if (key == "launch_ax_threshold_mps2") return set_double(&algorithms.attitude.launch_ax_threshold_mps2) || invalid();
        if (key == "launch_confirm_ms") return parse_timestamp_ms(value, &algorithms.attitude.launch_confirm_ns) || invalid();
        if (key == "accelerometer_specific_force") return parse_bool(value, &algorithms.attitude.accelerometer_specific_force) || invalid();
        if (key == "png_navigation_constant_y") return set_double(&algorithms.png_guidance.navigation_constant_y) || invalid();
        if (key == "png_navigation_constant_z") return set_double(&algorithms.png_guidance.navigation_constant_z) || invalid();
        if (key == "png_gravity") {
            if (!set_double(&algorithms.png_guidance.gravity)) {
                return invalid();
            }
            algorithms.attitude.gravity_mps2 = algorithms.png_guidance.gravity;
            algorithms.body_vector_pn.gravity = algorithms.png_guidance.gravity;
            return true;
        }
        if (key == "png_maximum_command_overload") return set_double(&algorithms.png_guidance.maximum_command_overload) || invalid();
        if (key == "png_maximum_command_acceleration") return set_double(&algorithms.png_guidance.maximum_command_acceleration) || invalid();
        if (key.rfind("camera_to_body_m", 0) == 0 && key.size() == 18) {
            const int row = key[16] - '0';
            const int column = key[17] - '0';
            if (row >= 0 && row < 3 && column >= 0 && column < 3 &&
                parse_double(value, &decimal)) {
                auto matrix = config->guidance.dart_from_camera.matrix();
                matrix[static_cast<std::size_t>(row * 3 + column)] = decimal;
                config->guidance.dart_from_camera =
                    domain::Rotation3<domain::DartFrameTag,
                                     domain::CameraFrameTag>(matrix);
                return true;
            }
        }
        return invalid();
    }

    if (section == "output") {
        if (key == "mode" && parse_output_mode(value, &config->output.mode)) {
        } else if (key == "recording_path") {
            config->output.recording_path = value;
        } else if (key == "draw_visualization") {
            return parse_bool(value, &config->output.draw_visualization) ||
                   invalid();
        } else if (key == "match_window_duration_ms" &&
                   parse_int(value, &config->output.match_window_duration_ms)) {
        } else {
            return invalid();
        }
        return true;
    }

    if (section == "wireless") {
        auto set_int = [&](int* destination) {
            if (!parse_int(value, &integer)) return false;
            *destination = integer;
            return true;
        };
        if (key == "rtsp_url") { config->wireless.rtsp_url = value; return true; }
        if (key == "rtsp_transport") { config->wireless.rtsp_transport = value; return true; }
        if (key == "width") return set_int(&config->wireless.width) || invalid();
        if (key == "height") return set_int(&config->wireless.height) || invalid();
        if (key == "fps") return set_int(&config->wireless.fps) || invalid();
        if (key == "bitrate") return set_int(&config->wireless.bitrate) || invalid();
        if (key == "gop_size") return set_int(&config->wireless.gop_size) || invalid();
        if (key == "rtsp_timeout_ms") {
            int timeout_ms = 0;
            if (!parse_int(value, &timeout_ms) || timeout_ms <= 0) return invalid();
            config->wireless.rtsp_timeout_us =
                static_cast<std::int64_t>(timeout_ms) * 1'000;
            return true;
        }
        if (key == "encoder_name") { config->wireless.encoder_name = value; return true; }
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
        !valid_pixel_format(config.camera.pixel_format) ||
        !std::isfinite(config.camera.exposure_us) ||
        config.camera.exposure_us <= 0.0 || !std::isfinite(config.camera.gain) ||
        config.camera.gain < 0.0 || config.camera.callback_queue_capacity == 0 ||
        config.camera.frame_timestamp_domain ==
            runtime::CameraTimestampDomain::unknown ||
        !std::isfinite(matrix.m00) ||
        !std::isfinite(matrix.m01) || !std::isfinite(matrix.m02) ||
        !std::isfinite(matrix.m10) || !std::isfinite(matrix.m11) ||
        !std::isfinite(matrix.m12) || !std::isfinite(matrix.m20) ||
        !std::isfinite(matrix.m21) || !std::isfinite(matrix.m22) ||
        matrix.m00 <= 0.0 || matrix.m11 <= 0.0 || matrix.m22 == 0.0) {
        return fail_global(path, "摄像头参数无效", error);
    }
    if (!std::isfinite(config.imu.sample_rate_hz) ||
        config.imu.sample_rate_hz <= 0.0 ||
        config.imu.measurement_latency_ns < 0 ||
        config.imu.receive_thread.read_chunk_size == 0 ||
        config.imu.receive_thread.realtime_priority < 0 ||
        config.imu.receive_thread.realtime_priority > 99) {
        return fail_global(path, "IMU 参数无效", error);
    }
    const bool serial_required =
        config.imu.mode == ImuMode::serial ||
        config.command.mode == CommandMode::serial;
    if (serial_required &&
        (config.serial.device.empty() || config.serial.baud_rate == 0 ||
         config.serial.data_bits < 5 || config.serial.data_bits > 8 ||
         config.serial.read_timeout_ms < 0 ||
         config.serial.write_timeout_ms < 0)) {
        return fail_global(path, "串口参数无效", error);
    }
    if (config.imu.mode == ImuMode::serial &&
        config.serial.read_timeout_ms <= 0) {
        return fail_global(path, "串口 IMU 模式要求 read_timeout_ms 大于 0",
                           error);
    }
    if (!validate_guidance(path, config, error)) {
        return false;
    }
    if (config.output.mode != output::OutputMode::rtsp &&
        config.output.recording_path.empty()) {
        return fail_global(path, "debug/match 模式要求 output.recording_path 不能为空",
                           error);
    }
    if (config.output.mode == output::OutputMode::match &&
        !output::match_window_fits_memory(
            config.camera.width, config.camera.height, config.camera.fps,
            config.output.match_window_duration_ms)) {
        return fail_global(path, "match 窗口时长无效或超过图像缓存内存上限（output.match_window_duration_ms）", error);
    }
    if (config.output.mode == output::OutputMode::rtsp &&
        (config.wireless.width <= 0 || config.wireless.height <= 0 ||
         config.wireless.width % 2 != 0 || config.wireless.height % 2 != 0 ||
         config.wireless.fps <= 0 || config.wireless.bitrate <= 0 ||
         config.wireless.gop_size <= 0 || config.wireless.rtsp_timeout_us <= 0)) {
        return fail_global(path, "无线图传参数无效", error);
    }
    return true;
}

bool validate_guidance(const std::filesystem::path& path, const AppConfig& config,
                       std::string* error) {
    const auto& vision = config.algorithms.vision_config;
    const auto& los_filter = config.algorithms.los_rate_filter;
    const auto& attitude = config.algorithms.attitude;
    const auto& png = config.algorithms.png_guidance;
    auto vector_pn = config.algorithms.body_vector_pn;
    vector_pn.gravity = attitude.gravity_mps2;
    if (!guidance::BodyVectorLosFilter::valid_config(config.algorithms.body_vector_filter) ||
        !guidance::BodyVectorPnGuidance::valid_config(vector_pn)) {
        return fail_global(path, "镖体系矢量 PN 参数无效", error);
    }
    const auto& intrinsics = config.guidance.camera_intrinsics;
    if (!std::isfinite(attitude.launch_speed_mps) ||
        attitude.launch_speed_mps <= 0.0 ||
        config.guidance.history_capacity == 0 ||
        config.guidance.max_imu_age_ns <= 0 ||
        config.guidance.max_camera_future_ns < 0 ||
        config.guidance.max_camera_age_ns <= 0 ||
        config.guidance.capture_queue_capacity == 0 ||
        config.guidance.output_queue_capacity == 0 ||
        config.guidance.info_log_queue_capacity == 0 ||
        vision.initial_roi.width <= 0 || vision.initial_roi.height <= 0 ||
        vision.found_range_x <= 0 || vision.found_range_y <= 0 ||
        vision.min_blob_area <= 0 || vision.min_aspect_ratio <= 0.0 ||
        vision.min_fill_ratio <= 0.0 || !std::isfinite(intrinsics.fx) ||
        !std::isfinite(intrinsics.fy) || !std::isfinite(intrinsics.cx) ||
        !std::isfinite(intrinsics.cy) || intrinsics.fx <= 0.0 ||
        intrinsics.fy <= 0.0 ||
        !valid_dart_from_camera(config.guidance.dart_from_camera) ||
        los_filter.sigma_a <= 0.0 || los_filter.sigma_q <= 0.0 ||
        los_filter.sigma_a_pitch < 0.0 || los_filter.sigma_a_yaw < 0.0 ||
        los_filter.initial_covariance < 0.0 ||
        attitude.gravity_mps2 <= 0.0 ||
        attitude.stationary_acceleration_tolerance_mps2 <= 0.0 ||
        attitude.gyro_stationary_threshold_rad_s <= 0.0 ||
        attitude.gyro_bias_filter_alpha <= 0.0 ||
        attitude.gyro_bias_filter_alpha > 1.0 ||
        attitude.launch_ax_threshold_mps2 <= 0.0 ||
        attitude.launch_confirm_ns <= 0 ||
        png.navigation_constant_y <= 0.0 ||
        png.navigation_constant_z <= 0.0 || png.gravity <= 0.0 ||
        png.maximum_command_overload < 0.0 ||
        std::isnan(png.maximum_command_acceleration) ||
        png.maximum_command_acceleration < 0.0) {
        return fail_global(path, "制导参数无效", error);
    }
    return true;
}

bool load_hardware_config(const std::filesystem::path& path, AppConfig* config,
                          std::string* error) {
    if (config == nullptr) {
        return fail_global(path, "配置输出指针为空", error);
    }
    AppConfig loaded;
    std::set<std::string> seen;
    if (!read_ini(path, {"camera", "imu", "command", "serial", "guidance", "output", "wireless"},
                  [&](std::size_t line, const std::string& section,
                      const std::string& key, const std::string& value) {
                      return parse_key(path, line, section, key, value, &loaded, &seen, error);
                  }, error)) {
        return false;
    }
    if (seen.find("output.mode") == seen.end()) {
        return fail_global(path, "必须显式配置 output.mode=rtsp|debug|match",
                           error);
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
    if (!loaded.output.recording_path.empty()) {
        const std::filesystem::path absolute_path =
            std::filesystem::absolute(path).parent_path() /
            loaded.output.recording_path;
        loaded.output.recording_path = absolute_path.lexically_normal().string();
    }
    loaded.guidance.camera_capture =
        runtime::CameraCaptureConfig{loaded.camera.capture_mode};
    *config = std::move(loaded);
    if (error != nullptr) {
        error->clear();
    }
    return true;
}

bool read_ini(const std::filesystem::path& path, const std::set<std::string>& sections,
              const IniVisitor& visitor, std::string* error) {
    std::ifstream input(path);
    if (!input) return fail_global(path, "无法读取配置文件", error);
    std::string section, text;
    std::size_t number = 0;
    while (std::getline(input, text)) {
        ++number;
        const auto line = trim(text);
        if (line.empty() || line.front() == '#') continue;
        if (line.front() == '[' || line.back() == ']') {
            if (line.size() < 3 || line.front() != '[' || line.back() != ']')
                return fail(path, number, "配置节格式无效", error);
            section = trim(line.substr(1, line.size() - 2));
            if (!sections.count(section)) return fail(path, number, "未知配置节: " + section, error);
            continue;
        }
        if (section.empty()) return fail(path, number, "配置键必须位于配置节中", error);
        const auto separator = line.find('=');
        if (separator == std::string::npos) return fail(path, number, "配置行缺少 '='", error);
        const auto key = trim(line.substr(0, separator));
        const auto value = trim(line.substr(separator + 1));
        if (key.empty() || value.empty()) return fail(path, number, "配置键和值不能为空", error);
        if (!visitor(number, section, key, value)) return false;
    }
    if (!input.eof() && input.fail()) return fail_global(path, "读取配置文件时发生错误", error);
    return true;
}

}  // namespace mosas::app::config_parsing
