#include <mosas/vision/opencv_image_frame.hpp>

#include <limits>
#include <utility>

namespace mosas::vision {
namespace {

struct OpenCvFrameOwner {
    std::shared_ptr<void> backing_owner;
    cv::Mat image;
};

std::size_t required_bytes(const cv::Mat& image) {
    const std::size_t row_bytes =
        static_cast<std::size_t>(image.cols) * image.elemSize();
    const std::size_t preceding_rows =
        static_cast<std::size_t>(image.rows - 1);
    if (preceding_rows != 0 &&
        image.step[0] >
            (std::numeric_limits<std::size_t>::max() - row_bytes) /
                preceding_rows) {
        return 0;
    }
    return preceding_rows * image.step[0] + row_bytes;
}

}  // namespace

cv::Mat make_bgr8_mat_view(const domain::ImageView& image) {
    if (!image.is_valid()) {
        return {};
    }
    return cv::Mat(image.height, image.width, CV_8UC3,
                   const_cast<std::uint8_t*>(image.data), image.stride_bytes);
}

cv::Mat make_bgr8_mat_view(domain::ImageFrame& image) {
    if (!image.is_valid()) {
        return {};
    }
    return cv::Mat(image.height, image.width, CV_8UC3, image.data,
                   image.stride_bytes);
}

bool assign_bgr8_mat(domain::ImageFrame* output, cv::Mat image,
                     std::shared_ptr<void> backing_owner) {
    if (output == nullptr || image.empty() || image.type() != CV_8UC3) {
        return false;
    }
    const std::size_t data_size_bytes = required_bytes(image);
    if (data_size_bytes == 0 || image.data == nullptr) {
        return false;
    }

    auto owner = std::make_shared<OpenCvFrameOwner>();
    owner->backing_owner = std::move(backing_owner);
    owner->image = std::move(image);
    output->owner = owner;
    output->data = owner->image.data;
    output->data_size_bytes = data_size_bytes;
    output->width = owner->image.cols;
    output->height = owner->image.rows;
    output->stride_bytes = owner->image.step[0];
    output->format = domain::PixelFormat::bgr8;
    return output->is_valid();
}

}  // namespace mosas::vision
