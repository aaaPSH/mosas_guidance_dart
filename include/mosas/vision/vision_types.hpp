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

/** @brief 可视化快照；LOS 属于导航惯性系，曝光姿态为同一时刻的三轴角；单位 rad、rad/s。 */
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
    bool attitude_valid = false;
    double attitude_pitch_rad = 0.0;
    double attitude_roll_rad = 0.0;
    double attitude_yaw_rad = 0.0;
    bool output_attitude_valid = false;
    double output_attitude_pitch_rad = 0.0;
    double output_attitude_roll_rad = 0.0;
    bool launched = false;
    double processing_fps = 0.0;
};

#endif  // VISION_TYPES_HPP
