#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include <opencv2/core/mat.hpp>

namespace mosas::wireless {

struct RtspStreamConfig {
    std::string rtsp_url = "rtsp://127.0.0.1:8554/mosas";
    // 使用 TCP 承载 RTP 可以减少接收端防火墙丢包；部分服务端也支持 udp。
    std::string rtsp_transport = "tcp";
    int width = 640;
    int height = 480;
    int fps = 30;
    int bitrate = 2'000'000;
    int gop_size = 30;
    // RTSP 建连和网络 I/O 超时时间，单位为微秒。
    std::int64_t rtsp_timeout_us = 5'000'000;
    // 留空时使用 FFmpeg 找到的第一个 H.264 编码器。
    std::string encoder_name;
};

class RtspStreamer {
public:
    explicit RtspStreamer(RtspStreamConfig config);
    ~RtspStreamer();

    RtspStreamer(const RtspStreamer&) = delete;
    RtspStreamer& operator=(const RtspStreamer&) = delete;

    bool start(std::string* error = nullptr);
    bool send_bgr(const cv::Mat& frame, std::int64_t capture_timestamp_ns,
                  std::string* error = nullptr);
    // 请求中断当前网络 I/O；资源由 stop() 在调用线程安全释放。
    void cancel() noexcept;
    // 清除停止后的取消状态，供下一轮 start() 使用。
    void reset() noexcept;
    void stop() noexcept;
    bool running() const noexcept;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace mosas::wireless
