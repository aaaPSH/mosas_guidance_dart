#include "vision_visualizer.hpp"

#include <algorithm>
#include <cstddef>

namespace {

struct RgbColor {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
};

bool is_valid_frame(const Rgb888Frame& frame) {
    return frame.data != nullptr && frame.width > 0 && frame.height > 0 &&
           frame.stride >= frame.width * 3;
}

VisionRoi clip_roi(const VisionRoi& roi, int width, int height) {
    const int left = std::max(0, std::min(roi.x, width));
    const int top = std::max(0, std::min(roi.y, height));
    const int right = std::max(left, std::min(roi.x + roi.width, width));
    const int bottom = std::max(top, std::min(roi.y + roi.height, height));
    return {left, top, right - left, bottom - top};
}

void set_pixel(Rgb888Frame& frame, int x, int y, const RgbColor& color) {
    if (x < 0 || x >= frame.width || y < 0 || y >= frame.height) {
        return;
    }
    const std::size_t offset = static_cast<std::size_t>(y) * frame.stride +
                                static_cast<std::size_t>(x) * 3U;
    frame.data[offset] = color.red;
    frame.data[offset + 1] = color.green;
    frame.data[offset + 2] = color.blue;
}

void draw_rectangle(Rgb888Frame& frame, const VisionRoi& raw_roi,
                    const RgbColor& color) {
    const VisionRoi roi = clip_roi(raw_roi, frame.width, frame.height);
    if (roi.width <= 0 || roi.height <= 0) {
        return;
    }
    const int right = roi.x + roi.width - 1;
    const int bottom = roi.y + roi.height - 1;
    for (int x = roi.x; x <= right; ++x) {
        set_pixel(frame, x, roi.y, color);
        set_pixel(frame, x, bottom, color);
    }
    for (int y = roi.y; y <= bottom; ++y) {
        set_pixel(frame, roi.x, y, color);
        set_pixel(frame, right, y, color);
    }
}

void draw_cross(Rgb888Frame& frame, int center_x, int center_y,
                const RgbColor& color) {
    for (int offset = -2; offset <= 2; ++offset) {
        set_pixel(frame, center_x + offset, center_y, color);
        set_pixel(frame, center_x, center_y + offset, color);
    }
}

}  // namespace

void VisionVisualizer::draw_result(Rgb888Frame& frame,
                                   const VisionResult& result) {
    if (!is_valid_frame(frame)) {
        return;
    }

    draw_rectangle(frame, result.next_roi, {0, 0, 255});
    if (!result.found) {
        return;
    }

    draw_rectangle(frame,
                   {result.blob.x, result.blob.y, result.blob.width,
                    result.blob.height},
                   {255, 0, 0});
    draw_cross(frame, result.blob.x + result.blob.width / 2,
               result.blob.y + result.blob.height / 2, {0, 255, 0});
}
