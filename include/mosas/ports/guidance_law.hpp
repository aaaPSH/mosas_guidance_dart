#ifndef MOSAS_PORTS_GUIDANCE_LAW_HPP
#define MOSAS_PORTS_GUIDANCE_LAW_HPP

#include <mosas/domain/algorithm_results.hpp>

namespace mosas::ports {

/// 输入一致的视线、姿态和运动状态并计算制导输出的算法端口。
class IGuidanceLaw {
public:
    virtual ~IGuidanceLaw() = default;

    /**
     * @brief 根据当前帧的有效状态计算制导指令及诊断量。
     * @param input 同步到相机帧时刻的制导输入；角度、角速度和物理量单位由领域类型标注。
     * @return 制导结果；无效输入须通过 valid 和 invalid_reason 表达，不以零指令代替无效状态。
     */
    virtual domain::GuidanceOutput calculate(
        const domain::GuidanceInput& input) = 0;
};

}  // namespace mosas::ports

#endif  // MOSAS_PORTS_GUIDANCE_LAW_HPP
