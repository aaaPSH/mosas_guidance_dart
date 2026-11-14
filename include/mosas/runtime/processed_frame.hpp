#ifndef MOSAS_RUNTIME_PROCESSED_FRAME_HPP
#define MOSAS_RUNTIME_PROCESSED_FRAME_HPP

#include <mosas/domain/algorithm_results.hpp>
#include <mosas/domain/command_output.hpp>
#include <mosas/runtime/runtime_types.hpp>

#include <memory>

namespace mosas::runtime {

class CommandEmissionReceipt;

// Each sink receives its own snapshot; image storage remains shared and immutable.
struct ProcessedFrame {
    CameraFrame frame;
    domain::VisionDetection vision_detection{};
    domain::FlightStateSnapshot flight_state{};
    domain::GuidanceOutput guidance{};
    /** @brief 各阶段布尔结果为 false 时的实际原因；尚未执行的阶段保持 not_evaluated。 */
    domain::GuidanceInvalidReason attitude_invalid_reason = domain::GuidanceInvalidReason::not_evaluated;
    domain::GuidanceInvalidReason los_invalid_reason = domain::GuidanceInvalidReason::not_evaluated;
    domain::GuidanceInvalidReason los_rate_invalid_reason = domain::GuidanceInvalidReason::not_evaluated;
    /** @brief 没有同帧输出快照的原因；由提交路径或观测 worker 写入，不推测已发送过载。 */
    domain::GuidanceInvalidReason output_missing_reason = domain::GuidanceInvalidReason::output_not_submitted;
    /** @brief 提交前登记的同帧首次输出凭据；仅观测 worker 有界等待，送入 sink 前清除。 */
    std::shared_ptr<CommandEmissionReceipt> pending_command_emission;
    /** @brief 只附带与当前来源帧、代际及时间匹配的首次真实输出；未发送或等待到期时为空。 */
    std::optional<domain::CommandEmissionSnapshot> command_emission;
    domain::GuidanceOverlay overlay{};
};

}  // namespace mosas::runtime

#endif  // MOSAS_RUNTIME_PROCESSED_FRAME_HPP
