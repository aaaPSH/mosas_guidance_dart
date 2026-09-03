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

// 可视化面板使用的引导数据，角度单位为弧度，角速度单位为弧度/秒。
struct VisionOverlayData {
    bool line_of_sight_valid;
    double line_of_sight_q_y_rad;
    double line_of_sight_q_z_rad;
    bool line_of_sight_rate_valid;
    double line_of_sight_rate_q_y_rad_s;
    double line_of_sight_rate_q_z_rad_s;
    bool body_overload_valid;
    double body_overload_x_g;
    double body_overload_y_g;
    double body_overload_z_g;
    bool attitude_valid;
    double attitude_pitch_rad;
    double attitude_yaw_rad;
    double attitude_roll_rad;
    bool velocity_valid;
    double velocity_x_mps;
    double velocity_y_mps;
    double velocity_z_mps;
};

#endif  // VISION_TYPES_HPP
