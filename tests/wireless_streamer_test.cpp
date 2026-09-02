#include <cassert>
#include <string>

#include <mosas/wireless/rtsp_streamer.hpp>

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

}  // namespace

int main() {
    test_streamer_rejects_non_rtsp_url();
    test_streamer_rejects_empty_rtsp_url();
    return 0;
}
