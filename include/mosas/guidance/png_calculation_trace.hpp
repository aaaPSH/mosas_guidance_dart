#pragma once
#include <mosas/guidance/ballistic_png_math.hpp>
#include <functional>
namespace mosas::domain { struct GuidanceInput; }
namespace mosas::guidance {
/** @brief 曝光时刻计算的只读观察接口，不包含尚未发生的最终输出。
 * @note 观察失败可触发运行故障，但不能修改控制值、重复计算或参与反馈。
 */
using PngTraceObserver = std::function<void(const domain::GuidanceInput&, const PngCalculationTrace&)>;
}
