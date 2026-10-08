#include <mosas/output/telemetry_csv_recorder.hpp>

#include <filesystem>
#include <fstream>
#include <locale>
#include <sstream>
#include <system_error>
#include <utility>

namespace mosas::output {
namespace {

constexpr char kFlightDataHeader[] =
    "frame_index,frame_timestamp_ns,frame_received_timestamp_ns,"
    "imu_timestamp_ns,vision_found,target_x,target_y,target_width,"
    "target_height,target_area,target_center_x,target_center_y,"
    "imu_acceleration_x_mps2,imu_acceleration_y_mps2,"
    "imu_acceleration_z_mps2,imu_angular_velocity_x_rad_s,"
    "imu_angular_velocity_y_rad_s,imu_angular_velocity_z_rad_s,"
    "servo_control_p_normalized,servo_control_y_normalized,"
    "imu_pitch_rad,imu_roll_rad,imu_attitude_valid,imu_launched,los_valid,"
    "los_q_y_rad,los_q_z_rad,los_rate_valid,los_rate_q_y_rad_s,"
    "los_rate_q_z_rad_s,guidance_valid,"
    "body_overload_x_g,body_overload_y_g,body_overload_z_g,"
    "command_overload_g,command_phase_rad,source_frame_id\n";

}  // namespace

TelemetryCsvRecorder::TelemetryCsvRecorder(
    std::shared_ptr<LocalRecordingSession> session)
    : session_(std::move(session)) {
    if (session_ == nullptr) {
        set_error("local recording session is null");
    } else if (!session_->valid()) {
        set_error(session_->configuration_error());
    }
}

TelemetryCsvRecorder::~TelemetryCsvRecorder() { stop(); }

bool TelemetryCsvRecorder::start() {
    if (session_ == nullptr || !session_->valid()) {
        if (session_ != nullptr) {
            set_error(session_->configuration_error());
        }
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(error_mutex_);
        last_error_.clear();
    }
    failed_ = false;
    stopped_ = false;
    cancelled_.store(false, std::memory_order_release);
    started_ = true;
    return true;
}

void TelemetryCsvRecorder::reset() noexcept {
    stop();
    if (session_ != nullptr) {
        session_->reset();
    }
    release_recording();
    buffered_rows_.clear();
    recording_frame_index_ = 0;
    recording_generation_ = 0;
    launch_window_active_ = false;
    started_ = false;
    stopped_ = false;
    cancelled_.store(false, std::memory_order_release);
    failed_ = false;
    {
        std::lock_guard<std::mutex> lock(error_mutex_);
        last_error_.clear();
    }
}

bool TelemetryCsvRecorder::record(const runtime::ProcessedFrame& frame) {
    if (cancelled_.load(std::memory_order_acquire)) {
        return true;
    }
    if (!started_ || stopped_ || failed_) {
        if (!started_ && !stopped_) {
            set_error("telemetry CSV recorder is not started");
        }
        return stopped_;
    }
    if (session_->config().mode == LocalRecordingMode::continuous) {
        return record_continuous(frame);
    }
    return record_launch_window(frame);
}

bool TelemetryCsvRecorder::record_continuous(
    const runtime::ProcessedFrame& frame) {
    if (!writer_.is_open() && !open_recording(0)) {
        return false;
    }
    const std::string row = make_csv_row(recording_frame_index_, frame);
    if (!write_csv_row(row)) {
        return false;
    }
    ++recording_frame_index_;
    return true;
}

bool TelemetryCsvRecorder::record_launch_window(
    const runtime::ProcessedFrame& frame) {
    const LocalWindowDecision decision =
        session_->classify(frame, LocalRecordingSinkKind::telemetry);
    if (decision.action == LocalWindowAction::invalid) {
        set_error(decision.error);
        return false;
    }
    if (decision.action == LocalWindowAction::ignore) {
        return true;
    }

    if (!launch_window_active_) {
        launch_window_active_ = true;
        recording_generation_ = decision.generation;
        recording_frame_index_ = 0;
        buffered_rows_.clear();
    }
    if (decision.action == LocalWindowAction::complete) {
        return persist_buffered_window(true);
    }
    if (buffered_rows_.size() >= session_->max_window_frame_count()) {
        return true;
    }
    buffered_rows_.push_back(make_csv_row(recording_frame_index_, frame));
    ++recording_frame_index_;
    return true;
}

std::string TelemetryCsvRecorder::make_csv_row(
    std::uint64_t frame_index, const runtime::ProcessedFrame& frame) const {
    const auto& vision_detection = frame.vision_detection;
    const auto& flight_state = frame.flight_state;
    const auto& guidance = frame.guidance;
    const auto& overlay = frame.overlay;
    const auto& box = vision_detection.diagnostics.bounding_box;
    const auto& sample = flight_state.imu_sample;
    const auto& attitude = flight_state.attitude;
    const auto& launch = flight_state.launch;
    const auto& line_of_sight_rate = guidance.diagnostics;
    std::ostringstream row;
    row << frame_index << ',' << frame.frame.timestamp_ns << ','
        << frame.frame.received_timestamp_ns << ','
        << flight_state.time.monotonic_ns << ','
        << (vision_detection.observation.valid ? 1 : 0) << ',' << box.x << ','
        << box.y << ',' << box.width << ',' << box.height << ','
        << vision_detection.diagnostics.area_pixels << ','
        << vision_detection.observation.center.u << ','
        << vision_detection.observation.center.v << ','
        << sample.acceleration.x << ',' << sample.acceleration.y << ','
        << sample.acceleration.z << ',' << sample.angular_velocity.x << ','
        << sample.angular_velocity.y << ',' << sample.angular_velocity.z << ','
        << sample.servo_control_p_normalized << ','
        << sample.servo_control_y_normalized << ','
        << attitude.pitch_rad.value << ',' << attitude.roll_rad.value << ','
        << (attitude.valid ? 1 : 0) << ','
        << (launch.phase == domain::LaunchPhase::launched ? 1 : 0) << ','
        << (overlay.line_of_sight_valid ? 1 : 0) << ','
        << overlay.line_of_sight_q_y.value << ','
        << overlay.line_of_sight_q_z.value << ','
        << (overlay.line_of_sight_rate_valid ? 1 : 0) << ','
        << overlay.line_of_sight_rate_q_y.value << ','
        << overlay.line_of_sight_rate_q_z.value << ','
        << (guidance.valid ? 1 : 0) << ','
        << line_of_sight_rate.body_overload.x << ','
        << line_of_sight_rate.body_overload.y << ','
        << line_of_sight_rate.body_overload.z << ','
        << line_of_sight_rate.command_overload.value << ','
        << line_of_sight_rate.command_phase.value << ','
        << frame.frame.source_frame_id << '\n';
    return row.str();
}

bool TelemetryCsvRecorder::open_recording(std::uint64_t generation) {
    const LocalRecordingPaths paths =
        session_->paths_for_generation(generation);
    if (!paths.telemetry_csv.parent_path().empty()) {
        std::error_code directory_error;
        std::filesystem::create_directories(
            paths.telemetry_csv.parent_path(), directory_error);
        if (directory_error) {
            set_error("create telemetry directory failed: " +
                      paths.telemetry_csv.parent_path().string() + " (" +
                      directory_error.message() + ")");
            return false;
        }
    }
    writer_.open(paths.telemetry_csv, std::ios::out | std::ios::trunc);
    if (!writer_.is_open()) {
        set_error("open flight data CSV failed: " +
                  paths.telemetry_csv.string());
        return false;
    }
    writer_.imbue(std::locale::classic());
    writer_ << kFlightDataHeader;
    writer_.flush();
    if (!writer_) {
        release_recording();
        set_error("write flight data CSV header failed: " +
                  paths.telemetry_csv.string());
        return false;
    }
    recording_generation_ = generation;
    recording_frame_index_ = 0;
    return true;
}

bool TelemetryCsvRecorder::write_csv_row(const std::string& row) {
    if (!writer_.is_open()) {
        set_error("telemetry CSV output is not open");
        return false;
    }
    writer_ << row;
    writer_.flush();
    if (!writer_) {
        const LocalRecordingPaths paths =
            session_->paths_for_generation(recording_generation_);
        set_error("write flight data CSV row failed: " +
                  paths.telemetry_csv.string());
        return false;
    }
    return true;
}

bool TelemetryCsvRecorder::finalize_recording(
    std::uint64_t expected_row_count) {
    const LocalRecordingPaths paths =
        session_->paths_for_generation(recording_generation_);
    if (writer_.is_open()) {
        writer_.flush();
        if (!writer_) {
            set_error("flush flight data CSV failed: " +
                      paths.telemetry_csv.string());
        }
        writer_.close();
        if (writer_.fail() && last_error().empty()) {
            set_error("close flight data CSV failed: " +
                      paths.telemetry_csv.string());
        }
    }
    if (!last_error().empty()) {
        return false;
    }

    std::ifstream csv(paths.telemetry_csv);
    std::string header;
    if (!csv.is_open() || !std::getline(csv, header) ||
        header != std::string(kFlightDataHeader,
                              sizeof(kFlightDataHeader) - 2)) {
        set_error("finalized flight data CSV header is invalid: " +
                  paths.telemetry_csv.string());
        return false;
    }
    std::uint64_t row_count = 0;
    std::string row;
    while (std::getline(csv, row)) {
        ++row_count;
    }
    if (csv.bad() || row_count != expected_row_count) {
        set_error("finalized flight data CSV row count mismatch: " +
                  paths.telemetry_csv.string());
        return false;
    }
    return true;
}

bool TelemetryCsvRecorder::persist_buffered_window(bool complete) {
    const std::uint64_t generation = recording_generation_;
    if (buffered_rows_.empty()) {
        launch_window_active_ = false;
        if (complete) {
            session_->mark_completed(LocalRecordingSinkKind::telemetry,
                                     generation);
        }
        return true;
    }
    if (!open_recording(generation)) {
        release_recording();
        buffered_rows_.clear();
        launch_window_active_ = false;
        return false;
    }
    for (const std::string& row : buffered_rows_) {
        if (!write_csv_row(row)) {
            release_recording();
            buffered_rows_.clear();
            launch_window_active_ = false;
            return false;
        }
    }
    const auto expected_row_count = buffered_rows_.size();
    if (!finalize_recording(expected_row_count)) {
        release_recording();
        buffered_rows_.clear();
        launch_window_active_ = false;
        return false;
    }
    buffered_rows_.clear();
    launch_window_active_ = false;
    if (complete) {
        session_->mark_completed(LocalRecordingSinkKind::telemetry, generation);
    }
    return true;
}

void TelemetryCsvRecorder::release_recording() noexcept {
    if (writer_.is_open()) {
        writer_.flush();
        writer_.close();
    }
}

bool TelemetryCsvRecorder::take_rearm_request() noexcept {
    return session_ != nullptr && session_->take_rearm_request();
}

void TelemetryCsvRecorder::cancel() noexcept {
    cancelled_.store(true, std::memory_order_release);
}

void TelemetryCsvRecorder::stop() noexcept {
    if (stopped_) {
        return;
    }
    stopped_ = true;
    if (session_ != nullptr &&
        session_->config().mode == LocalRecordingMode::launch_window &&
        launch_window_active_) {
        try {
            if (!persist_buffered_window(false)) {
                failed_ = true;
            }
        } catch (const std::exception& exception) {
            set_error(exception.what());
            release_recording();
            buffered_rows_.clear();
            launch_window_active_ = false;
        } catch (...) {
            set_error("save partial flight data CSV failed");
            release_recording();
            buffered_rows_.clear();
            launch_window_active_ = false;
        }
    } else if (session_ != nullptr &&
               session_->config().mode == LocalRecordingMode::continuous &&
               writer_.is_open()) {
        try {
            if (!finalize_recording(recording_frame_index_)) {
                failed_ = true;
                release_recording();
            }
        } catch (const std::exception& exception) {
            set_error(exception.what());
            release_recording();
        } catch (...) {
            set_error("finalize continuous flight data CSV failed");
            release_recording();
        }
    }
    release_recording();
}

std::string TelemetryCsvRecorder::last_error() const {
    std::lock_guard<std::mutex> lock(error_mutex_);
    return last_error_;
}

void TelemetryCsvRecorder::set_error(std::string error) noexcept {
    failed_ = true;
    try {
        std::lock_guard<std::mutex> lock(error_mutex_);
        if (last_error_.empty()) {
            last_error_ = std::move(error);
        }
    } catch (...) {
    }
}

}  // namespace mosas::output
