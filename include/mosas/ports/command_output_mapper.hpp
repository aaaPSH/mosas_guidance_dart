#ifndef MOSAS_PORTS_COMMAND_OUTPUT_MAPPER_HPP
#define MOSAS_PORTS_COMMAND_OUTPUT_MAPPER_HPP
#include <mosas/domain/command_output.hpp>
namespace mosas::ports {
/** @brief 设备无关的输出边界映射接口；调用不得等待传感器或执行 I/O。 */
class ICommandOutputMapper {
public:
    virtual ~ICommandOutputMapper() = default;
    /** @return 构造配置可用于安全输出时为 true。 */
    virtual bool valid() const noexcept = 0;
    /** @brief Clear all per-run trigger state without changing configuration.
     * @note Call at a session or generation boundary with output attempts excluded by the caller.
     */
    virtual void reset() noexcept = 0;
    /** @param request 未限幅弹道请求。 @param context 同次输出的完整上下文。
     * @return 无效输入或保护触发时为无效零命令，保留相应原因及来源。
     */
    virtual domain::CommandMappingResult map(const domain::BallisticOverloadG& request,
        const domain::CommandOutputContext& context) const noexcept = 0;
};
}
#endif
