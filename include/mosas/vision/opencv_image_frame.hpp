#ifndef MOSAS_VISION_OPENCV_IMAGE_FRAME_HPP
#define MOSAS_VISION_OPENCV_IMAGE_FRAME_HPP

#include <mosas/domain/image_frame.hpp>

#include <memory>

#include <opencv2/core/mat.hpp>

namespace mosas::vision {

cv::Mat make_bgr8_mat_view(const domain::ImageView& image);
cv::Mat make_bgr8_mat_view(domain::ImageFrame& image);

// Transfers the Mat header without copying pixel storage. backing_owner keeps
// external storage alive when the Mat itself does not own its data.
bool assign_bgr8_mat(domain::ImageFrame* output, cv::Mat image,
                     std::shared_ptr<void> backing_owner = {});

}  // namespace mosas::vision

#endif  // MOSAS_VISION_OPENCV_IMAGE_FRAME_HPP
