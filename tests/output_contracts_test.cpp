#include <mosas/runtime/output_ports.hpp>

#include "test_support.hpp"

#include <memory>
#include <type_traits>
#include <vector>

namespace {

class FakeVideoRecorder final : public mosas::runtime::IVideoRecorder {
public:
    bool start() override {
        started = true;
        return true;
    }

    void reset() noexcept override {}

    bool record(const mosas::runtime::ProcessedFrame& frame) override {
        last_frame_id = frame.frame.source_frame_id;
        last_image_owner = frame.frame.image.owner.get();
        ++recorded_count;
        return true;
    }

    void cancel() noexcept override { cancelled = true; }
    void stop() noexcept override { stopped = true; }
    std::string last_error() const override { return {}; }

    bool started = false;
    bool cancelled = false;
    bool stopped = false;
    std::size_t recorded_count = 0;
    std::uint64_t last_frame_id = 0;
    const void* last_image_owner = nullptr;
};

}  // namespace

int main() {
    using namespace mosas::runtime;

    static_assert(std::is_abstract_v<IVideoRecorder>);
    static_assert(std::is_abstract_v<ITelemetryRecorder>);
    static_assert(std::is_abstract_v<IVisualizationSink>);
    static_assert(std::is_abstract_v<IImuRecorder>);
    static_assert(std::is_abstract_v<IAsyncLogger>);
    // Producers can construct log payloads without the concrete logger header.
    static_assert(std::is_trivially_copyable_v<RuntimeLogEvent>);
    const RuntimeLogEvent event{};
    CHECK(event.kind == RuntimeLogEventKind::diagnostic);
    CHECK(event.text.back() == '\0');

    const RuntimeOutputDiagnostics empty{};
    CHECK(!empty.video.enabled);
    CHECK(empty.video.healthy);
    CHECK(empty.video.accepted_count == 0);
    CHECK(empty.video.dropped_count == 0);
    CHECK(empty.video.failure_count == 0);
    CHECK(empty.video.queue_depth == 0);
    CHECK(empty.video.queue_capacity == 0);
    CHECK(empty.video.last_error.empty());
    CHECK(!empty.telemetry.enabled);
    CHECK(!empty.visualization.enabled);
    CHECK(!empty.imu_recording.enabled);
    CHECK(!empty.logging.enabled);

    auto pixels = std::make_shared<std::vector<std::uint8_t>>(3, 0x52);
    ProcessedFrame processed{};
    processed.frame.source_frame_id = 42;
    processed.frame.image.owner = pixels;
    processed.frame.image.data = pixels->data();
    processed.frame.image.data_size_bytes = pixels->size();
    processed.frame.image.width = 1;
    processed.frame.image.height = 1;
    processed.frame.image.stride_bytes = 3;
    processed.frame.image.format = mosas::domain::PixelFormat::bgr8;

    FakeVideoRecorder recorder;
    CHECK(recorder.start());
    CHECK(recorder.record(processed));
    CHECK(recorder.started);
    CHECK(recorder.recorded_count == 1);
    CHECK(recorder.last_frame_id == 42);
    CHECK(recorder.last_image_owner == processed.frame.image.owner.get());
    recorder.cancel();
    recorder.stop();
    CHECK(recorder.cancelled);
    CHECK(recorder.stopped);
    return 0;
}
