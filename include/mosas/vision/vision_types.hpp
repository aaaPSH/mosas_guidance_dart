#ifndef VISION_TYPES_HPP
#define VISION_TYPES_HPP

#include <cstdint>

struct VisionRoi {
    int x;
    int y;
    int width;
    int height;
};

struct VisionBlob {
    int x;
    int y;
    int width;
    int height;
    int area;
    double center_x;
    double center_y;
};

// 使用 OpenCV 风格的离散 HSV 范围：H 为 0..179，S/V 为 0..255。
struct HsvThreshold {
    uint8_t h_min;
    uint8_t h_max;
    uint8_t s_min;
    uint8_t s_max;
    uint8_t v_min;
    uint8_t v_max;
};

struct VisionConfig {
    VisionRoi initial_roi;
    HsvThreshold threshold;
    int found_range_x;
    int found_range_y;
    int min_blob_area;
    double min_aspect_ratio;
    double min_fill_ratio;
};

struct VisionResult {
    bool found;
    VisionBlob blob;
    VisionRoi next_roi;
};

#endif  // VISION_TYPES_HPP
