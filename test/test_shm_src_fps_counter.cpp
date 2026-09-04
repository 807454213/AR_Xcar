#include "videocapture.h"

#include <cstdint>
#include <cstdlib>
#include <iostream>

namespace {

void expectEqual(uint64_t actual, uint64_t expected, const char* name)
{
    if (actual != expected) {
        std::cerr << name << " expected " << expected
                  << " got " << actual << std::endl;
        std::exit(1);
    }
}

} // namespace

int main()
{
    expectEqual(shmFidAdvanceForFps(100, 0, false), 1, "first valid fid");
    expectEqual(shmFidAdvanceForFps(100, 100, true), 0, "same fid");
    expectEqual(shmFidAdvanceForFps(104, 100, true), 4, "forward skipped fids");
    expectEqual(shmFidAdvanceForFps(1200, 100, true), 1, "implausible fid jump");
    expectEqual(shmFidAdvanceForFps(3, 100, true), 1, "stream reset");
    return 0;
}
