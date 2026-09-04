#ifndef NPU_CORE_CONFIG_H
#define NPU_CORE_CONFIG_H

#include <algorithm>

inline int normalizeNpuCoreIndex(int core)
{
    return std::clamp(core, 0, 2);
}

#endif // NPU_CORE_CONFIG_H
