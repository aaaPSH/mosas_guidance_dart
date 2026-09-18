#ifndef MOSAS_RUNTIME_THREAD_AFFINITY_HPP
#define MOSAS_RUNTIME_THREAD_AFFINITY_HPP

#include <string>

namespace mosas::runtime {

// 将当前线程限制在指定 CPU 核；cpu_affinity 为 -1 时表示不绑定。
// 失败时返回 false，并通过 error 返回可诊断的错误信息。
bool configure_current_thread_affinity(int cpu_affinity,
                                       std::string* error);

}  // namespace mosas::runtime

#endif  // MOSAS_RUNTIME_THREAD_AFFINITY_HPP
