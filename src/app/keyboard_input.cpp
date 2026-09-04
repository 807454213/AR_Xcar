#include "app/pipeline.h"

char appSelectKeyboardInput(bool display_enabled, int cv_key, int terminal_key)
{
    const int key = display_enabled ? cv_key : terminal_key;
    if (key < 0)
        return 0;
    return static_cast<char>(key & 0xff);
}
