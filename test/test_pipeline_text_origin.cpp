#include "app/pipeline.h"

#include <cstdlib>
#include <iostream>

namespace {

void expectPoint(AppTextOrigin actual, int x, int y, const char* name)
{
    if (actual.x != x || actual.y != y) {
        std::cerr << name << " expected (" << x << "," << y
                  << ") got (" << actual.x << "," << actual.y << ")\n";
        std::exit(1);
    }
}

} // namespace

int main()
{
    expectPoint(appTextOriginInsideFrame(20, 20, 0, 30, 8, 100, 80),
                20, 20, "normal position");
    expectPoint(appTextOriginInsideFrame(92, 20, 58, 30, 8, 100, 80),
                58, 20, "right edge uses fallback side");
    expectPoint(appTextOriginInsideFrame(92, 20, -40, 30, 8, 100, 80),
                2, 20, "fallback clamps left edge");
    expectPoint(appTextOriginInsideFrame(20, 3, 0, 30, 8, 100, 80),
                20, 10, "top edge moves down");
    expectPoint(appTextOriginInsideFrame(90, 90, 30, 30, 8, 100, 80),
                30, 78, "bottom edge clamps baseline");
    return 0;
}
