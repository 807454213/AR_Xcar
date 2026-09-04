#ifndef APP_PIPELINE_H
#define APP_PIPELINE_H

#include <algorithm>

//=============================================================================
// 主流水线入口：初始化硬件代理 / RKNN / PPSeg / 相机，运行主循环并在退出时清理。
// 返回进程退出码（0 正常）。
//=============================================================================
int runPipeline();

char appSelectKeyboardInput(bool display_enabled, int cv_key, int terminal_key);

struct AppTextOrigin {
    int x = 0;
    int y = 0;
};

inline AppTextOrigin appTextOriginInsideFrame(int preferredX,
                                              int preferredY,
                                              int fallbackX,
                                              int textWidth,
                                              int textHeight,
                                              int frameWidth,
                                              int frameHeight,
                                              int margin = 2)
{
    const int safeMargin = std::max(0, margin);
    int x = preferredX;
    if (frameWidth > 0 && textWidth > 0 &&
        x + textWidth > frameWidth - safeMargin) {
        x = fallbackX;
    }
    x = std::max(safeMargin, x);
    if (frameWidth > 0 && textWidth > 0 &&
        x + textWidth > frameWidth - safeMargin) {
        x = std::max(safeMargin, frameWidth - textWidth - safeMargin);
    }

    int y = preferredY;
    if (textHeight > 0)
        y = std::max(textHeight + safeMargin, y);
    if (frameHeight > 0)
        y = std::min(std::max(safeMargin, y), frameHeight - safeMargin);

    return {x, y};
}

#endif // APP_PIPELINE_H
