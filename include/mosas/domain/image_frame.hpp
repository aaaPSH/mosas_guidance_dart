#ifndef MOSAS_DOMAIN_IMAGE_FRAME_HPP
#define MOSAS_DOMAIN_IMAGE_FRAME_HPP

#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>

namespace mosas::domain {

enum class PixelFormat { unknown, bgr8, rgb8 };

struct ImageView {
    const std::uint8_t* data = nullptr;
    std::size_t data_size_bytes = 0;
    int width = 0;
    int height = 0;
    std::size_t stride_bytes = 0;
    PixelFormat format = PixelFormat::unknown;

    bool is_valid() const noexcept {
        if (data == nullptr || width <= 0 || height <= 0 ||
            (format != PixelFormat::bgr8 && format != PixelFormat::rgb8)) {
            return false;
        }
        const auto width_size = static_cast<std::size_t>(width);
        const auto height_size = static_cast<std::size_t>(height);
        if (width_size > std::numeric_limits<std::size_t>::max() / 3) {
            return false;
        }
        const std::size_t row_bytes = width_size * 3;
        if (stride_bytes < row_bytes ||
            height_size - 1 >
                (std::numeric_limits<std::size_t>::max() - row_bytes) /
                    stride_bytes) {
            return false;
        }
        const std::size_t required_bytes =
            (height_size - 1) * stride_bytes + row_bytes;
        return data_size_bytes >= required_bytes;
    }
};

// Owns the pixel storage without exposing the storage implementation type.
// The pointer and metadata describe the same view for the lifetime of owner.
struct ImageFrame {
    std::shared_ptr<void> owner;
    std::uint8_t* data = nullptr;
    std::size_t data_size_bytes = 0;
    int width = 0;
    int height = 0;
    std::size_t stride_bytes = 0;
    PixelFormat format = PixelFormat::unknown;

    ImageView view() const noexcept {
        return {data, data_size_bytes, width, height, stride_bytes, format};
    }

    bool is_valid() const noexcept {
        return owner != nullptr && view().is_valid();
    }
};

}  // namespace mosas::domain

#endif  // MOSAS_DOMAIN_IMAGE_FRAME_HPP
