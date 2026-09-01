#include "vision_recognizer.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <queue>
#include <vector>

namespace {

constexpr VisionConfig kDefaultConfig{
    {0, 0, 320, 240},
    {35, 85, 80, 255, 100, 255},
    40,
    40,
    8,
    0.5,
    0.35,
};

struct Hsv {
    uint8_t h;
    uint8_t s;
    uint8_t v;
};

bool is_valid_frame(const Rgb888Frame& frame) {
    return frame.data != nullptr && frame.width > 0 && frame.height > 0 &&
           frame.stride >= frame.width * 3;
}

bool is_valid_roi(const VisionRoi& roi) {
    return roi.width > 0 && roi.height > 0;
}

VisionRoi clip_roi(const VisionRoi& roi, int width, int height) {
    const int left = std::max(0, std::min(roi.x, width));
    const int top = std::max(0, std::min(roi.y, height));
    const int right = std::max(left, std::min(roi.x + roi.width, width));
    const int bottom = std::max(top, std::min(roi.y + roi.height, height));
    return {left, top, right - left, bottom - top};
}

Hsv rgb_to_hsv(uint8_t red, uint8_t green, uint8_t blue) {
    const double r = static_cast<double>(red) / 255.0;
    const double g = static_cast<double>(green) / 255.0;
    const double b = static_cast<double>(blue) / 255.0;
    const double max_value = std::max({r, g, b});
    const double min_value = std::min({r, g, b});
    const double delta = max_value - min_value;

    double hue = 0.0;
    if (delta > 0.0) {
        if (max_value == r) {
            hue = 60.0 * std::fmod((g - b) / delta, 6.0);
        } else if (max_value == g) {
            hue = 60.0 * (((b - r) / delta) + 2.0);
        } else {
            hue = 60.0 * (((r - g) / delta) + 4.0);
        }
        if (hue < 0.0) {
            hue += 360.0;
        }
    }

    const double saturation = max_value == 0.0 ? 0.0 : delta / max_value;
    return {
        static_cast<uint8_t>(hue / 2.0),
        static_cast<uint8_t>(saturation * 255.0),
        static_cast<uint8_t>(max_value * 255.0),
    };
}

bool is_target_pixel(const Rgb888Frame& frame, int x, int y,
                     const HsvThreshold& threshold) {
    const std::size_t offset = static_cast<std::size_t>(y) * frame.stride +
                                static_cast<std::size_t>(x) * 3U;
    const Hsv hsv = rgb_to_hsv(frame.data[offset], frame.data[offset + 1],
                               frame.data[offset + 2]);
    return hsv.h >= threshold.h_min && hsv.h <= threshold.h_max &&
           hsv.s >= threshold.s_min && hsv.s <= threshold.s_max &&
           hsv.v >= threshold.v_min && hsv.v <= threshold.v_max;
}

bool is_candidate(const VisionBlob& blob, const VisionConfig& config) {
    if (blob.area < config.min_blob_area || blob.width <= 0 ||
        blob.height <= 0) {
        return false;
    }

    const double width = static_cast<double>(blob.width);
    const double height = static_cast<double>(blob.height);
    const double aspect_ratio = std::min(width / height, height / width);
    const double fill_ratio = static_cast<double>(blob.area) / (width * height);
    return aspect_ratio >= config.min_aspect_ratio &&
           fill_ratio >= config.min_fill_ratio;
}

VisionRoi expanded_roi(const VisionBlob& blob, int range_x, int range_y,
                       int frame_width, int frame_height) {
    return clip_roi({blob.x - range_x, blob.y - range_y,
                     blob.width + 2 * range_x, blob.height + 2 * range_y},
                    frame_width, frame_height);
}

}  // namespace

VisionRecognizer::VisionRecognizer() : config_(kDefaultConfig), current_roi_(kDefaultConfig.initial_roi) {}

VisionRecognizer::VisionRecognizer(const VisionConfig& config)
    : config_(config), current_roi_(config.initial_roi) {}

void VisionRecognizer::reset() {
    current_roi_ = config_.initial_roi;
}

VisionResult VisionRecognizer::process(const Rgb888Frame& frame) {
    VisionResult result{false, {0, 0, 0, 0, 0, 0.0, 0.0}, current_roi_};
    if (!is_valid_frame(frame)) {
        return result;
    }

    const VisionRoi roi = clip_roi(current_roi_, frame.width, frame.height);
    if (!is_valid_roi(roi)) {
        current_roi_ = config_.initial_roi;
        result.next_roi = current_roi_;
        return result;
    }

    const std::size_t pixel_count = static_cast<std::size_t>(frame.width) *
                                     static_cast<std::size_t>(frame.height);
    std::vector<uint8_t> visited(pixel_count, 0);
    std::queue<std::pair<int, int>> pending;

    VisionBlob best_blob{0, 0, 0, 0, 0, 0.0, 0.0};
    for (int y = roi.y; y < roi.y + roi.height; ++y) {
        for (int x = roi.x; x < roi.x + roi.width; ++x) {
            const std::size_t index = static_cast<std::size_t>(y) *
                                      static_cast<std::size_t>(frame.width) +
                                      static_cast<std::size_t>(x);
            if (visited[index] ||
                !is_target_pixel(frame, x, y, config_.threshold)) {
                continue;
            }

            visited[index] = 1;
            pending.push({x, y});
            int area = 0;
            double moment_x = 0.0;
            double moment_y = 0.0;
            int min_x = x;
            int max_x = x;
            int min_y = y;
            int max_y = y;

            while (!pending.empty()) {
                const auto [pixel_x, pixel_y] = pending.front();
                pending.pop();
                ++area;
                moment_x += pixel_x;
                moment_y += pixel_y;
                min_x = std::min(min_x, pixel_x);
                max_x = std::max(max_x, pixel_x);
                min_y = std::min(min_y, pixel_y);
                max_y = std::max(max_y, pixel_y);

                for (int offset_y = -1; offset_y <= 1; ++offset_y) {
                    for (int offset_x = -1; offset_x <= 1; ++offset_x) {
                        if (offset_x == 0 && offset_y == 0) {
                            continue;
                        }
                        const int neighbor_x = pixel_x + offset_x;
                        const int neighbor_y = pixel_y + offset_y;
                        if (neighbor_x < roi.x ||
                            neighbor_x >= roi.x + roi.width ||
                            neighbor_y < roi.y ||
                            neighbor_y >= roi.y + roi.height) {
                            continue;
                        }

                        const std::size_t neighbor_index =
                            static_cast<std::size_t>(neighbor_y) *
                                static_cast<std::size_t>(frame.width) +
                            static_cast<std::size_t>(neighbor_x);
                        if (visited[neighbor_index] ||
                            !is_target_pixel(frame, neighbor_x, neighbor_y,
                                             config_.threshold)) {
                            continue;
                        }
                        visited[neighbor_index] = 1;
                        pending.push({neighbor_x, neighbor_y});
                    }
                }
            }

            const VisionBlob candidate{
                min_x,
                min_y,
                max_x - min_x + 1,
                max_y - min_y + 1,
                area,
                moment_x / static_cast<double>(area),
                moment_y / static_cast<double>(area),
            };
            if (is_candidate(candidate, config_) &&
                candidate.area > best_blob.area) {
                best_blob = candidate;
            }
        }
    }

    if (best_blob.area > 0) {
        result.found = true;
        result.blob = best_blob;
        current_roi_ = expanded_roi(best_blob, config_.found_range_x,
                                     config_.found_range_y, frame.width,
                                     frame.height);
    } else {
        current_roi_ = clip_roi(config_.initial_roi, frame.width, frame.height);
    }
    result.next_roi = current_roi_;
    return result;
}

VisionRoi VisionRecognizer::current_roi() const {
    return current_roi_;
}
