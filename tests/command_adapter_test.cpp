#include "command_adapter.hpp"

#include "test_support.hpp"
#include "command_output_test_support.hpp"
#include <mosas/adapters/host_adapters.hpp>
#include <sstream>

#include <array>
#include <cstdint>

int main() {
    mosas::domain::MappedGuidanceCommand command{};
    command.valid = true;
    command.launched = true;
    command.target_found = true;
    command.overload_g = {1.5, 0.005, -2.5};
    const auto values = mosas::app::detail::make_control_frame_values(command);
    CHECK(values.launched);
    CHECK(values.target_found);
    CHECK_NEAR(values.yaw_overload_g, -2.5, 1e-12);
    CHECK_NEAR(values.pitch_overload_g, 0.005, 1e-12);

    std::array<std::uint8_t, serial_package::kControlFrameSize> frame{};
    CHECK(serial_package::encode_control_frame(values, &frame));
    constexpr std::array<std::uint8_t, serial_package::kControlFrameSize>
        expected_valid_frame{0xAA, 0x55, 0x01, 0x01, 0x06, 0xFF, 0x01,
                             0x00, 0x00, 0x00, 0x76, 0x6F, 0x0D, 0x0A};
    CHECK(frame == expected_valid_frame);

    command.valid = false;
    const auto invalid_values =
        mosas::app::detail::make_control_frame_values(command);
    CHECK(invalid_values.launched);
    CHECK(invalid_values.target_found);
    CHECK_NEAR(invalid_values.yaw_overload_g, 0.0, 1e-12);
    CHECK_NEAR(invalid_values.pitch_overload_g, 0.0, 1e-12);
    CHECK(serial_package::encode_control_frame(invalid_values, &frame));
    constexpr std::array<std::uint8_t, serial_package::kControlFrameSize>
        expected_invalid_frame{0xAA, 0x55, 0x01, 0x01, 0x00, 0x00, 0x00,
                               0x00, 0x00, 0x00, 0x8C, 0xCE, 0x0D, 0x0A};
    CHECK(frame == expected_invalid_frame);

    command.target_found = false;
    CHECK(serial_package::encode_control_frame(
        mosas::app::detail::make_control_frame_values(command), &frame));
    constexpr std::array<std::uint8_t, serial_package::kControlFrameSize>
        expected_lost_frame{0xAA, 0x55, 0x01, 0x00, 0x00, 0x00, 0x00,
                            0x00, 0x00, 0x00, 0xED, 0x76, 0x0D, 0x0A};
    CHECK(frame == expected_lost_frame);

    command.launched = false;
    CHECK(serial_package::encode_control_frame(
        mosas::app::detail::make_control_frame_values(command), &frame));
    constexpr std::array<std::uint8_t, serial_package::kControlFrameSize>
        expected_reset_frame{0xAA, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00,
                             0x00, 0x00, 0x00, 0x3E, 0x31, 0x0D, 0x0A};
    CHECK(frame == expected_reset_frame);

    // 日志边界也必须映射弹道请求，不能直接记录为 body 值。
    std::ostringstream log;
    mosas::app::LoggingCommandSink sink(log);
    CHECK(mosas::test::bind_test_output(sink,[]{return std::optional<mosas::domain::FlightStateSnapshot>(
        mosas::test::command_test_state(200,0,std::acos(-1.0)/2));},[]{return 201;}));
    CHECK(sink.start());
    mosas::domain::GuidanceCommand request;
    request.source_frame_id=91;request.source_time=mosas::test::command_test_time(100);
    request.valid=request.launched=request.target_found=true;request.overload_g={0,0.5,0};
    CHECK(sink.try_submit(request));sink.stop();
    const auto emitted=sink.latest_emission();
    CHECK(emitted && emitted->mapping.command.source_frame_id==91);
    CHECK_NEAR(emitted->mapping.command.overload_g.z,-0.5,1e-12);
    CHECK(emitted->disposition==mosas::domain::CommandEmissionDisposition::log_accepted);
    CHECK(log.str().find("-0.5")!=std::string::npos);
    return 0;
}
