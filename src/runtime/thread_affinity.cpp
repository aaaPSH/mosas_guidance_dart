#include <mosas/runtime/thread_affinity.hpp>

#include <cerrno>
#include <cstring>

#ifdef __linux__
#include <pthread.h>
#include <sched.h>
#endif

namespace mosas::runtime {

bool configure_current_thread_affinity(int cpu_affinity,
                                       std::string* error) {
    if (error != nullptr) {
        error->clear();
    }
    if (cpu_affinity == -1) {
        return true;
    }
    if (cpu_affinity < -1) {
        if (error != nullptr) {
            *error = "CPU 亲和性必须为 -1 或非负 CPU 编号";
        }
        return false;
    }

#ifdef __linux__
    if (cpu_affinity >= CPU_SETSIZE) {
        if (error != nullptr) {
            *error = "CPU 编号超出系统 CPU 集合范围";
        }
        return false;
    }

    cpu_set_t cpu_set;
    CPU_ZERO(&cpu_set);
    CPU_SET(cpu_affinity, &cpu_set);
    const int result = pthread_setaffinity_np(
        pthread_self(), sizeof(cpu_set), &cpu_set);
    if (result != 0) {
        if (error != nullptr) {
            *error = "设置当前线程 CPU 亲和性失败: ";
            *error += std::strerror(result);
        }
        return false;
    }
    return true;
#else
    if (error != nullptr) {
        *error = "当前平台不支持 Linux CPU 亲和性配置";
    }
    return false;
#endif
}

}  // namespace mosas::runtime
