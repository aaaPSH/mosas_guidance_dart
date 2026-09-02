#include <mosas/wireless/rtsp_streamer.hpp>

#include <algorithm>
#include <cstdint>
#include <string>
#include <utility>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>
#include <libavutil/dict.h>
#include <libavutil/opt.h>
#include <libswscale/swscale.h>
}

namespace mosas::wireless {
namespace {

bool set_error(std::string* error, const std::string& message) {
    if (error != nullptr) {
        *error = message;
    }
    return false;
}

std::string ffmpeg_error(const std::string& operation, int code) {
    char buffer[AV_ERROR_MAX_STRING_SIZE]{};
    av_strerror(code, buffer, sizeof(buffer));
    return operation + ": " + buffer;
}

}  // namespace

class RtspStreamer::Impl {
public:
    explicit Impl(RtspStreamConfig config) : config_(std::move(config)) {}

    ~Impl() { stop(); }

    bool start(std::string* error) {
        if (running()) {
            return set_error(error, "streamer is already running");
        }
        if (!validate_config(error)) {
            return false;
        }

        if (avformat_network_init() < 0) {
            return set_error(error, "initialize FFmpeg network support failed");
        }
        network_initialized_ = true;

        const AVCodec* codec = nullptr;
        if (!config_.encoder_name.empty()) {
            codec = avcodec_find_encoder_by_name(config_.encoder_name.c_str());
            if (codec == nullptr) {
                return fail(error, "H.264 encoder not found: " +
                                       config_.encoder_name);
            }
        } else {
            codec = avcodec_find_encoder(AV_CODEC_ID_H264);
        }
        if (codec == nullptr || codec->id != AV_CODEC_ID_H264) {
            return fail(error, "no H.264 encoder is available in FFmpeg");
        }

        int result = avformat_alloc_output_context2(
            &format_context_, nullptr, "rtsp", config_.rtsp_url.c_str());
        if (result < 0 || format_context_ == nullptr) {
            return fail(error, ffmpeg_error("allocate RTSP output", result));
        }

        stream_ = avformat_new_stream(format_context_, nullptr);
        if (stream_ == nullptr) {
            return fail(error, "create RTSP video stream failed");
        }

        codec_context_ = avcodec_alloc_context3(codec);
        if (codec_context_ == nullptr) {
            return fail(error, "allocate H.264 encoder context failed");
        }
        codec_context_->codec_id = codec->id;
        codec_context_->codec_type = AVMEDIA_TYPE_VIDEO;
        codec_context_->width = config_.width;
        codec_context_->height = config_.height;
        codec_context_->pix_fmt = AV_PIX_FMT_YUV420P;
        codec_context_->time_base = AVRational{1, config_.fps};
        codec_context_->framerate = AVRational{config_.fps, 1};
        codec_context_->bit_rate = config_.bitrate;
        codec_context_->gop_size = config_.gop_size;
        codec_context_->max_b_frames = 0;
        codec_context_->flags |= AV_CODEC_FLAG_LOW_DELAY;

        // libx264 的低延迟参数对硬件编码器无效，因此只针对它设置。
        const std::string codec_name = codec->name != nullptr ? codec->name : "";
        if (codec_name == "libx264" || codec_name == "libx264rgb") {
            av_opt_set(codec_context_->priv_data, "preset", "ultrafast", 0);
            av_opt_set(codec_context_->priv_data, "tune", "zerolatency", 0);
            av_opt_set(codec_context_->priv_data, "profile", "baseline", 0);
            av_opt_set(codec_context_->priv_data, "repeat-headers", "1", 0);
        }

        result = avcodec_open2(codec_context_, codec, nullptr);
        if (result < 0) {
            return fail(error, ffmpeg_error("open H.264 encoder", result));
        }

        result = avcodec_parameters_from_context(stream_->codecpar,
                                                  codec_context_);
        if (result < 0) {
            return fail(error,
                        ffmpeg_error("copy encoder parameters", result));
        }
        stream_->time_base = codec_context_->time_base;

        // RTSP 输出由 FFmpeg 的 RTSP muxer 建立连接，不能像 UDP 文件输出
        // 那样额外调用 avio_open。
        AVDictionary* output_options = nullptr;
        av_dict_set(&output_options, "rtsp_transport",
                    config_.rtsp_transport.c_str(), 0);
        av_dict_set(&output_options, "timeout",
                    std::to_string(config_.rtsp_timeout_us).c_str(), 0);
        result = avformat_write_header(format_context_, &output_options);
        av_dict_free(&output_options);
        if (result < 0) {
            return fail(error, ffmpeg_error("connect to RTSP output", result));
        }
        header_written_ = true;

        frame_ = av_frame_alloc();
        if (frame_ == nullptr) {
            return fail(error, "allocate YUV frame failed");
        }
        frame_->format = codec_context_->pix_fmt;
        frame_->width = config_.width;
        frame_->height = config_.height;
        result = av_frame_get_buffer(frame_, 32);
        if (result < 0) {
            return fail(error, ffmpeg_error("allocate YUV frame buffer", result));
        }

        scaler_ = sws_getContext(
            config_.width, config_.height, AV_PIX_FMT_BGR24, config_.width,
            config_.height, AV_PIX_FMT_YUV420P, SWS_FAST_BILINEAR, nullptr,
            nullptr, nullptr);
        if (scaler_ == nullptr) {
            return fail(error, "create BGR to YUV420P converter failed");
        }

        next_pts_ = 0;
        first_capture_timestamp_ns_ = AV_NOPTS_VALUE;
        return true;
    }

    bool send_bgr(const cv::Mat& input, std::int64_t capture_timestamp_ns,
                  std::string* error) {
        if (!running()) {
            return set_error(error, "streamer is not running");
        }
        if (input.empty() || input.cols != config_.width ||
            input.rows != config_.height || input.type() != CV_8UC3) {
            return set_error(error, "frame must be CV_8UC3 with configured size");
        }

        int result = av_frame_make_writable(frame_);
        if (result < 0) {
            return fail(error, ffmpeg_error("make YUV frame writable", result));
        }

        const std::uint8_t* source[] = {input.ptr<std::uint8_t>(0)};
        const int source_stride[] = {static_cast<int>(input.step[0])};
        sws_scale(scaler_, source, source_stride, 0, config_.height,
                  frame_->data, frame_->linesize);
        if (first_capture_timestamp_ns_ == AV_NOPTS_VALUE) {
            first_capture_timestamp_ns_ = capture_timestamp_ns;
        }
        std::int64_t timestamp_pts = next_pts_;
        if (capture_timestamp_ns >= first_capture_timestamp_ns_) {
            timestamp_pts = av_rescale_q(
                capture_timestamp_ns - first_capture_timestamp_ns_,
                AVRational{1, 1'000'000'000}, codec_context_->time_base);
        }
        frame_->pts = std::max(next_pts_, timestamp_pts);
        next_pts_ = frame_->pts + 1;

        result = avcodec_send_frame(codec_context_, frame_);
        if (result < 0) {
            return fail(error, ffmpeg_error("send frame to H.264 encoder", result));
        }
        return write_encoded_packets(error);
    }

    void stop() noexcept {
        if (stopping_) {
            return;
        }
        stopping_ = true;
        if (header_written_ && codec_context_ != nullptr &&
            format_context_ != nullptr) {
            if (avcodec_send_frame(codec_context_, nullptr) >= 0) {
                write_encoded_packets(nullptr);
            }
            av_write_trailer(format_context_);
        }

        if (scaler_ != nullptr) {
            sws_freeContext(scaler_);
            scaler_ = nullptr;
        }
        if (frame_ != nullptr) {
            av_frame_free(&frame_);
        }
        if (codec_context_ != nullptr) {
            avcodec_free_context(&codec_context_);
        }
        if (format_context_ != nullptr) {
            avformat_free_context(format_context_);
            format_context_ = nullptr;
        }
        stream_ = nullptr;
        header_written_ = false;
        if (network_initialized_) {
            avformat_network_deinit();
            network_initialized_ = false;
        }
        stopping_ = false;
    }

    bool running() const noexcept {
        return format_context_ != nullptr && codec_context_ != nullptr &&
               frame_ != nullptr && scaler_ != nullptr;
    }

private:
    bool validate_config(std::string* error) const {
        if (config_.rtsp_url.rfind("rtsp://", 0) != 0) {
            return set_error(error, "RTSP URL must start with rtsp://");
        }
        if (config_.rtsp_url.size() <= 7 ||
            (config_.rtsp_transport != "tcp" &&
             config_.rtsp_transport != "udp")) {
            return set_error(error, "invalid RTSP URL or transport");
        }
        if (config_.width <= 0 || config_.height <= 0 || config_.fps <= 0 ||
            config_.bitrate <= 0 || config_.gop_size <= 0 ||
            config_.rtsp_timeout_us <= 0 ||
            config_.width % 2 != 0 || config_.height % 2 != 0) {
            return set_error(error, "invalid RTSP stream configuration");
        }
        return true;
    }

    bool fail(std::string* error, const std::string& message) {
        stop();
        return set_error(error, message);
    }

    bool write_encoded_packets(std::string* error) {
        AVPacket* packet = av_packet_alloc();
        if (packet == nullptr) {
            return fail(error, "allocate encoded packet failed");
        }

        bool success = true;
        while (true) {
            const int result = avcodec_receive_packet(codec_context_, packet);
            if (result == AVERROR(EAGAIN) || result == AVERROR_EOF) {
                break;
            }
            if (result < 0) {
                success = fail(error,
                               ffmpeg_error("receive H.264 packet", result));
                break;
            }
            av_packet_rescale_ts(packet, codec_context_->time_base,
                                 stream_->time_base);
            packet->stream_index = stream_->index;
            const int write_result =
                av_interleaved_write_frame(format_context_, packet);
            av_packet_unref(packet);
            if (write_result < 0) {
                success = fail(error,
                               ffmpeg_error("write RTSP packet", write_result));
                break;
            }
        }
        av_packet_free(&packet);
        return success;
    }

    RtspStreamConfig config_;
    AVFormatContext* format_context_ = nullptr;
    AVStream* stream_ = nullptr;
    AVCodecContext* codec_context_ = nullptr;
    AVFrame* frame_ = nullptr;
    SwsContext* scaler_ = nullptr;
    std::int64_t next_pts_ = 0;
    std::int64_t first_capture_timestamp_ns_ = AV_NOPTS_VALUE;
    bool header_written_ = false;
    bool network_initialized_ = false;
    bool stopping_ = false;
};

RtspStreamer::RtspStreamer(RtspStreamConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {}

RtspStreamer::~RtspStreamer() = default;

bool RtspStreamer::start(std::string* error) { return impl_->start(error); }

bool RtspStreamer::send_bgr(const cv::Mat& frame,
                            std::int64_t capture_timestamp_ns,
                            std::string* error) {
    return impl_->send_bgr(frame, capture_timestamp_ns, error);
}

void RtspStreamer::stop() noexcept { impl_->stop(); }

bool RtspStreamer::running() const noexcept { return impl_->running(); }

}  // namespace mosas::wireless
