#ifndef MOSAS_APP_DEFAULT_GUIDANCE_ALGORITHMS_HPP
#define MOSAS_APP_DEFAULT_GUIDANCE_ALGORITHMS_HPP

#include <mosas/app/app_config.hpp>
#include <mosas/guidance/png_calculation_trace.hpp>

namespace mosas::app {

/**
 * @brief 根据配置成对构造 LOS 滤波器与制导律，默认保留原 PNG。
 * @param config 算法选择、视觉/姿态和滤波/制导参数。
 * @param trace_observer 可选 PNG 只读诊断；默认关闭，不装配到矢量 PN。
 * @return 可移交给运行时的独占算法组件。
 */
runtime::RuntimeSupervisorDependencies make_default_guidance_algorithms(
    const GuidanceAlgorithmConfig& config,
    guidance::PngTraceObserver trace_observer = {});

}  // namespace mosas::app

#endif  // MOSAS_APP_DEFAULT_GUIDANCE_ALGORITHMS_HPP
