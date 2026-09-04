# Gold Band Visual Overlay Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a config-controlled main-frame debug overlay for the gold slow/reachable band tuning parameters.

**Architecture:** Add one `tc.goldBandVisualEnabled` boolean to config defaults, JSON loading, JSON writing, and sample configs. Add one local drawing helper in `drive_control.cpp` and call it from the existing `draw_debug` block so the overlay only appears when the app already renders debug visuals.

**Tech Stack:** C++17, OpenCV drawing primitives, existing JSON config helpers, existing CMake test targets.

## Global Constraints

- The flag name is exactly `goldBandVisualEnabled`.
- The default value is `false`.
- The overlay draws only on the main frame, not in BEV.
- The overlay draws only when `appDebugOverlayActive(config().app)` is true.
- The displayed geometry must use the same perspective gold-band calculation path used by runtime gold logic.
- Do not restructure `drive_control.cpp` beyond the local helper needed for this overlay.

---

### Task 1: Config Flag and Tests

**Files:**
- Modify: `include/config.h`
- Modify: `src/io/config.cpp`
- Modify: `configs/config.json`
- Modify: `configs/config2.json`
- Create: `test/test_gold_band_visual_config.cpp`
- Modify: `test/CMakeLists.txt`

**Interfaces:**
- Produces: `config().tc.goldBandVisualEnabled` as a `bool`.
- Consumes: existing `jBool()` and config save format in `src/io/config.cpp`.

- [ ] **Step 1: Write the failing config test**

Create `test/test_gold_band_visual_config.cpp`:

```cpp
#include "config.h"

#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>

static bool writeFile(const char* path, const std::string& text)
{
    std::ofstream out(path);
    if (!out) return false;
    out << text;
    return true;
}

int main()
{
    const bool default_value = config().tc.goldBandVisualEnabled;
    if (default_value != false) {
        std::cerr << "default goldBandVisualEnabled expected false\n";
        return 1;
    }

    const char* path = "/tmp/xcar_gold_band_visual_config.json";
    const std::string json =
        "{\n"
        "  \"tc\": {\n"
        "    \"goldBandVisualEnabled\": true,\n"
        "    \"goldTrackWidthAddInner\": 8,\n"
        "    \"goldTrackWidthAddOuter\": 22,\n"
        "    \"goldReachableWidthAddOuter\": 87\n"
        "  }\n"
        "}\n";

    if (!writeFile(path, json)) {
        std::cerr << "failed to write temp config\n";
        return 2;
    }

    if (!configLoad(path)) {
        std::cerr << "configLoad failed\n";
        return 3;
    }

    if (!config().tc.goldBandVisualEnabled) {
        std::cerr << "goldBandVisualEnabled was not loaded as true\n";
        return 4;
    }

    std::remove(path);
    return 0;
}
```

Add this target to `test/CMakeLists.txt`:

```cmake
add_executable(test_gold_band_visual_config
    ${CMAKE_CURRENT_LIST_DIR}/test_gold_band_visual_config.cpp
    ${ROOT_DIR}/src/io/config.cpp
    ${ROOT_DIR}/src/perception/camera_model.cpp
)
target_link_libraries(test_gold_band_visual_config ${OpenCV_LIBS})
set_target_properties(test_gold_band_visual_config PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
```

- [ ] **Step 2: Run test to verify it fails**

Run:

```bash
cmake -S test -B test/build
cmake --build test/build --target test_gold_band_visual_config -j$(nproc)
./test/build/bin/test_gold_band_visual_config
```

Expected: build fails because `TrackControlParams` has no member named `goldBandVisualEnabled`.

- [ ] **Step 3: Implement config support**

Add to `include/config.h` near the gold width fields:

```cpp
bool goldBandVisualEnabled = false; // 调试叠加：显示金币内扩/外扩/可达带
```

Add to `src/io/config.cpp` after `allowGoldOutsideTrack` is loaded:

```cpp
tc.goldBandVisualEnabled = jBool(tcSec, "goldBandVisualEnabled", tc.goldBandVisualEnabled);
```

Add to `src/io/config.cpp` save output after `allowGoldOutsideTrack`:

```cpp
fprintf(fp, "        \"goldBandVisualEnabled\": %s,\n", tc.goldBandVisualEnabled ? "true" : "false");
```

Add to both config JSON files after `allowGoldOutsideTrack`:

```json
"goldBandVisualEnabled": false,
```

- [ ] **Step 4: Run test to verify it passes**

Run:

```bash
cmake --build test/build --target test_gold_band_visual_config -j$(nproc)
./test/build/bin/test_gold_band_visual_config
```

Expected: command exits 0.

---

### Task 2: Main-Frame Overlay and Pixel Test

**Files:**
- Modify: `src/control/drive_control.cpp`
- Create: `test/test_gold_band_visual_overlay.cpp`
- Modify: `test/CMakeLists.txt`

**Interfaces:**
- Consumes: `config().tc.goldBandVisualEnabled`.
- Consumes: `tc_process(...)` existing frame mutation path.
- Produces: main-frame overlay only when both `draw_debug` and `goldBandVisualEnabled` are true.

- [ ] **Step 1: Write the failing overlay test**

Create `test/test_gold_band_visual_overlay.cpp`:

```cpp
#include "config.h"
#include "trackcontrol.h"
#include "uart.h"

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

static int changedPixels(const cv::Mat& a, const cv::Mat& b)
{
    cv::Mat diff;
    cv::absdiff(a, b, diff);
    std::vector<cv::Mat> ch;
    cv::split(diff, ch);
    cv::Mat mask = ch[0] | ch[1] | ch[2];
    return cv::countNonZero(mask);
}

static void makeTrack(int w, int h, std::vector<int>& mid,
                      std::vector<int>& left, std::vector<int>& right)
{
    mid.assign(h, w / 2);
    left.assign(h, 95);
    right.assign(h, 225);
}

static int runCase(bool enabled)
{
    const int W = 320;
    const int H = 240;
    config().app.runtimeMode = "vision";
    config().app.recordDebug = false;
    config().tc.goldBandVisualEnabled = enabled;
    config().tc.goldTrackWidthAddInner = 8;
    config().tc.goldTrackWidthAddOuter = 22;
    config().tc.goldReachableWidthAddOuter = 87;
    config().tc.goldReachableBypassMinY = 136;
    config().tc.carFrontY = 150;

    Uart::instance().setTransmitEnabled(false);
    tc_init(W, H);

    std::vector<int> mid, left, right;
    makeTrack(W, H, mid, left, right);

    cv::Mat frame(H, W, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat before = frame.clone();
    cv::Mat mask(H, W, CV_8UC1, cv::Scalar(255));
    HardwareProxy hw;
    std::vector<TrackedObject> objs;

    (void)tc_process(mid, left, right, objs, frame, frame, mask, hw);
    return changedPixels(before, frame);
}

int main()
{
    const int off_changed = runCase(false);
    const int on_changed = runCase(true);

    std::cout << "off_changed=" << off_changed
              << " on_changed=" << on_changed << "\n";

    if (off_changed != 0) {
        std::cerr << "overlay disabled should not change an empty debug frame\n";
        return 1;
    }
    if (on_changed <= 0) {
        std::cerr << "overlay enabled should draw gold band pixels\n";
        return 2;
    }
    return 0;
}
```

Add this target to `test/CMakeLists.txt`:

```cmake
add_executable(test_gold_band_visual_overlay
    ${CMAKE_CURRENT_LIST_DIR}/test_gold_band_visual_overlay.cpp
    ${TEST_COMMON_SOURCES}
    ${TEST_CONTROL_SOURCES}
)
target_compile_definitions(test_gold_band_visual_overlay PRIVATE XCAR_TESTING)
target_link_libraries(test_gold_band_visual_overlay ${TEST_LIBS} curl ssl crypto)
set_target_properties(test_gold_band_visual_overlay PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
```

- [ ] **Step 2: Run test to verify it fails**

Run:

```bash
cmake --build test/build --target test_gold_band_visual_overlay -j$(nproc)
./test/build/bin/test_gold_band_visual_overlay
```

Expected: binary runs and exits 2 with `on_changed=0`.

- [ ] **Step 3: Implement overlay drawing**

Add a helper in `src/control/drive_control.cpp` near other gold helpers:

```cpp
static void tc_drawGoldBandVisual(Mat& frame,
                                  const vector<int>& left,
                                  const vector<int>& right,
                                  int y_top,
                                  int y_bottom)
{
    if (frame.empty()) return;
    const auto& TC = config().tc;
    const int h = frame.rows;
    const int w = frame.cols;
    const int y0 = clampInt(std::max(0, y_top), 0, std::max(0, h - 1));
    const int y1 = clampInt(std::min(y_bottom, h - 1), 0, std::max(0, h - 1));
    for (int y = y0; y <= y1; y += 2) {
        if (y < 0 || y >= (int)left.size() || y >= (int)right.size()) continue;
        const int lx = left[y];
        const int rx = right[y];
        if (lx < 0 || rx <= lx) continue;

        int lx_ex = -1, lx_in = -1, rx_in = -1, rx_ex = -1;
        if (!tc_goldBandAtRowPerspective(y, lx, rx,
                                         TC.goldTrackWidthAddInner,
                                         TC.goldTrackWidthAddOuter,
                                         lx_ex, lx_in, rx_in, rx_ex)) {
            continue;
        }

        int lx_reach = -1, lx_dummy = -1, rx_dummy = -1, rx_reach = -1;
        if (!tc_goldBandAtRowPerspective(y, lx, rx, 0,
                                         std::max(TC.goldReachableWidthAddOuter,
                                                  TC.goldTrackWidthAddOuter),
                                         lx_reach, lx_dummy, rx_dummy, rx_reach)) {
            lx_reach = lx_ex;
            rx_reach = rx_ex;
        }

        auto cx = [w](int x) { return clampInt(x, 0, std::max(0, w - 1)); };
        circle(frame, Point(cx(lx), y), 1, Scalar(255, 220, 0), -1, LINE_AA);
        circle(frame, Point(cx(rx), y), 1, Scalar(255, 220, 0), -1, LINE_AA);
        circle(frame, Point(cx(lx_in), y), 1, Scalar(0, 0, 255), -1, LINE_AA);
        circle(frame, Point(cx(rx_in), y), 1, Scalar(0, 0, 255), -1, LINE_AA);
        circle(frame, Point(cx(lx_ex), y), 1, Scalar(0, 140, 255), -1, LINE_AA);
        circle(frame, Point(cx(rx_ex), y), 1, Scalar(0, 140, 255), -1, LINE_AA);
        circle(frame, Point(cx(lx_reach), y), 1, Scalar(255, 0, 0), -1, LINE_AA);
        circle(frame, Point(cx(rx_reach), y), 1, Scalar(255, 0, 0), -1, LINE_AA);
    }

    char info[128];
    snprintf(info, sizeof(info), "GOLD BAND inner=%d outer=%d reach=%d",
             TC.goldTrackWidthAddInner, TC.goldTrackWidthAddOuter,
             TC.goldReachableWidthAddOuter);
    putText(frame, info, Point(4, 72), FONT_HERSHEY_SIMPLEX, 0.38,
            Scalar(255, 255, 255), 1);
}
```

Call it in `tc_process()` debug visualization section:

```cpp
if (draw_debug && TC.goldBandVisualEnabled) {
    tc_drawGoldBandVisual(frame, left_use, right_use, yTop2, yBottom);
}
```

- [ ] **Step 4: Run focused tests**

Run:

```bash
cmake --build test/build --target test_gold_band_visual_overlay test_gold_band_visual_config test_gold_slow_band -j$(nproc)
./test/build/bin/test_gold_band_visual_overlay
./test/build/bin/test_gold_band_visual_config
./test/build/bin/test_gold_slow_band
```

Expected: all commands exit 0.

---

### Task 3: Final Verification

**Files:**
- Verify: all files changed in Tasks 1 and 2.

- [ ] **Step 1: Build main binary**

Run:

```bash
cmake --build build --target main -j$(nproc)
```

Expected: target builds successfully.

- [ ] **Step 2: Check whitespace**

Run:

```bash
git diff --check -- include/config.h src/io/config.cpp src/control/drive_control.cpp configs/config.json configs/config2.json test/test_gold_band_visual_config.cpp test/test_gold_band_visual_overlay.cpp test/CMakeLists.txt
```

Expected: no output.

- [ ] **Step 3: Review diff**

Run:

```bash
git diff -- include/config.h src/io/config.cpp src/control/drive_control.cpp configs/config.json configs/config2.json test/test_gold_band_visual_config.cpp test/test_gold_band_visual_overlay.cpp test/CMakeLists.txt
```

Expected: diff only contains the config flag, overlay helper/call, tests, and CMake target registrations.
