#pragma once
#include <sched.h>
#include <pthread.h>
#include <cstdio>

namespace CpuAffinity {

inline bool bindToCore(int coreId) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(coreId, &cpuset);
    pthread_t thread = pthread_self();
    int ret = pthread_setaffinity_np(thread, sizeof(cpu_set_t), &cpuset);
    if (ret != 0) {
        fprintf(stderr, "[CpuAffinity] Failed to bind to core %d\n", coreId);
        return false;
    }
    return true;
}

inline bool setRtPriority(int priority = 50) {
    struct sched_param param;
    param.sched_priority = priority;
    int ret = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
    if (ret != 0) {
        fprintf(stderr, "[CpuAffinity] Failed to set RT priority (need root?)\n");
        return false;
    }
    return true;
}

} // namespace CpuAffinity
