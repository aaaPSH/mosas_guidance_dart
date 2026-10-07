#include <mosas/vision/opencv_image_frame.hpp>

#include "test_support.hpp"

#include <memory>
#include <vector>

int main() {
    using namespace mosas;

    constexpr int kWidth = 2;
    constexpr int kHeight = 2;
    constexpr std::size_t kStride = 10;
    auto storage = std::make_shared<std::vector<std::uint8_t>>(
        kStride * (kHeight - 1) + kWidth * 3, 0);
    std::weak_ptr<std::vector<std::uint8_t>> storage_lifetime = storage;

    domain::ImageFrame source{};
    source.owner = storage;
    source.data = storage->data();
    source.data_size_bytes = storage->size();
    source.width = kWidth;
    source.height = kHeight;
    source.stride_bytes = kStride;
    source.format = domain::PixelFormat::bgr8;
    CHECK(source.is_valid());

    cv::Mat borrowed = vision::make_bgr8_mat_view(source.view());
    CHECK(!borrowed.empty());
    CHECK(borrowed.type() == CV_8UC3);
    CHECK(borrowed.cols == kWidth);
    CHECK(borrowed.rows == kHeight);
    CHECK(borrowed.step[0] == kStride);
    (*storage)[kStride] = 0x5a;
    CHECK(borrowed.at<cv::Vec3b>(1, 0)[0] == 0x5a);

    domain::ImageFrame adapted{};
    CHECK(vision::assign_bgr8_mat(&adapted, std::move(borrowed), storage));
    CHECK(adapted.is_valid());
    CHECK(adapted.data == source.data);
    CHECK(adapted.stride_bytes == kStride);
    source = {};
    storage.reset();
    CHECK(!storage_lifetime.expired());
    CHECK(vision::make_bgr8_mat_view(adapted).at<cv::Vec3b>(1, 0)[0] == 0x5a);

    domain::ImageView invalid{nullptr, 0, kWidth, kHeight, kStride,
                             domain::PixelFormat::unknown};
    CHECK(vision::make_bgr8_mat_view(invalid).empty());
    CHECK(!vision::assign_bgr8_mat(&adapted, cv::Mat{}));

    adapted = {};
    CHECK(storage_lifetime.expired());
    return 0;
}
