#include <mosas/domain/image_frame.hpp>

#include "test_support.hpp"

#include <memory>
#include <vector>

int main() {
    using mosas::domain::ImageFrame;
    using mosas::domain::ImageView;
    using mosas::domain::PixelFormat;

    auto pixels = std::make_shared<std::vector<std::uint8_t>>(14, 0);
    std::weak_ptr<std::vector<std::uint8_t>> lifetime = pixels;
    ImageFrame frame{};
    frame.owner = pixels;
    frame.data = pixels->data();
    frame.data_size_bytes = pixels->size();
    frame.width = 2;
    frame.height = 2;
    frame.stride_bytes = 8;
    frame.format = PixelFormat::bgr8;
    CHECK(frame.is_valid());

    const ImageView view = frame.view();
    CHECK(view.is_valid());
    CHECK(view.width == 2);
    CHECK(view.height == 2);
    CHECK(view.stride_bytes == 8);
    CHECK(view.data_size_bytes == 14);
    CHECK(view.data == pixels->data());

    ImageFrame queued_frame = frame;
    frame = {};
    pixels.reset();
    CHECK(!lifetime.expired());
    CHECK(queued_frame.is_valid());
    queued_frame = {};
    CHECK(lifetime.expired());

    ImageView short_stride{reinterpret_cast<const std::uint8_t*>(1), 64,
                           2, 2, 5, PixelFormat::bgr8};
    CHECK(!short_stride.is_valid());
    ImageView short_storage{reinterpret_cast<const std::uint8_t*>(1), 13,
                            2, 2, 8, PixelFormat::bgr8};
    CHECK(!short_storage.is_valid());
    ImageView zero_stride{reinterpret_cast<const std::uint8_t*>(1), 64,
                          2, 2, 0, PixelFormat::bgr8};
    CHECK(!zero_stride.is_valid());
    ImageView unknown_format{reinterpret_cast<const std::uint8_t*>(1), 64,
                             2, 2, 8, PixelFormat::unknown};
    CHECK(!unknown_format.is_valid());

    ImageFrame missing_owner{};
    missing_owner.data = reinterpret_cast<std::uint8_t*>(1);
    missing_owner.data_size_bytes = 6;
    missing_owner.width = 2;
    missing_owner.height = 1;
    missing_owner.stride_bytes = 6;
    missing_owner.format = PixelFormat::bgr8;
    CHECK(!missing_owner.is_valid());
    return 0;
}
