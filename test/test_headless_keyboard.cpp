#include "app/pipeline.h"

#include <iostream>

int main()
{
    bool ok = true;

    const char headless_key = appSelectKeyboardInput(false, 0, 'F');
    if (headless_key != 'F') {
        std::cout << "FAIL headless expected F got " << (int)headless_key << "\n";
        ok = false;
    }

    const char display_key = appSelectKeyboardInput(true, 'S', 'F');
    if (display_key != 'S') {
        std::cout << "FAIL display expected S got " << (int)display_key << "\n";
        ok = false;
    }

    std::cout << (ok ? "OK" : "FAIL") << " headless keyboard selection\n";
    return ok ? 0 : 2;
}
