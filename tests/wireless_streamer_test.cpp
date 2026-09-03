#include <cassert>
#include <string>

#include <mosas/wireless/rtsp_frame_sink.hpp>
#include <mosas/wireless/rtsp_streamer.hpp>

#include <opencv2/core.hpp>

namespace {

void test_streamer_rejects_non_rtsp_url() {
    mosas::wireless::RtspStreamConfig config;
    config.rtsp_url = "udp://127.0.0.1:5000";

    mosas::wireless::RtspStreamer streamer(config);
    std::string error;
    assert(!streamer.start(&error));
    assert(error.find("rtsp://") != std::string::npos);
}

void test_streamer_rejects_empty_rtsp_url() {
    mosas::wireless::RtspStreamConfig config;
    config.rtsp_url.clear();

    mosas::wireless::RtspStreamer streamer(config);
    std::string error;
    assert(!streamer.start(&error));
    assert(error.find("RTSP") != std::string::npos);
}

void test_frame_sink_switch_disables_wireless_output() {
    mosas::wireless::RtspFrameSinkConfig config;
    config.enable_wireless_stream = false;
    config.stream.rtsp_url = "invalid://disabled-output";

    mosas::wireless::RtspFrameSink sink(config, cv::Point2d(20.0, 18.0));
    mosas::runtime::CameraFrame frame{
        123,
        cv::Mat(40, 40, CV_8UC3, cv::Scalar(0, 0, 0)),
    };
    const VisionResult vision_result{
        false, {0, 0, 0, 0, 0, 0.0, 0.0}, {0, 0, 40, 40}};
    const mosas::runtime::ImuStateSnapshot imu_state{
        123, mosas::runtime::FlightPhase::free_flight,
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
    };
    const PngGuidanceOutput guidance{
        false, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, 0.0,
        0.0,
    };

    assert(sink.publish(frame, vision_result, imu_state, guidance));
}

}  // namespace

int main() {
    test_streamer_rejects_non_rtsp_url();
    test_streamer_rejects_empty_rtsp_url();
    test_frame_sink_switch_disables_wireless_output();
    return 0;
}
