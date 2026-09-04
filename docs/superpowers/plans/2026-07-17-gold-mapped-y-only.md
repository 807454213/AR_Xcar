# Gold Mapped-Y Only Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make gold mapping compute only `mapped_y = goldMappedYK1 * box.y + (5/6) * box.height` while preserving the detector-provided x coordinate.

**Architecture:** Replace the point-returning mapping helper with an integer y helper. Pipeline preprocessing updates only `center_y`, and the control layer combines the preserved `TrackedObject::center_x` with a freshly computed mapped y so direct callers remain correct.

**Tech Stack:** C++17, OpenCV, CMake, JSON configuration, Markdown.

## Global Constraints

- Use exactly `mapped_y = goldMappedYK1 * box.y + (5.0f / 6.0f) * box.height`.
- `goldMappedYK1` is the total positive `box.y` slope; keep the compiled default `0.4f` and preserve the active JSON value `1.4`.
- Keep K2 fixed at exact `5.0f / 6.0f`; do not add a K2 configuration key.
- Never derive mapped x from `box.x` or `box.width`; preserve `TrackedObject::center_x`.
- Continue rounding y with `std::lround` and clamping it to the image range.
- Preserve unrelated working-tree configuration changes.

---

### Task 1: Implement the Positive-Slope Y-Only Mapping

**Files:**
- Modify: `test/test_gold_slow_band.cpp:14-75,1693,1802,2004`
- Modify: `test/test_gold_outside_record_control.cpp:48-73`
- Modify: `test/test_vehicle_gold_source_driven_control.cpp:29-44,90`
- Modify: `include/trackcontrol.h:34-56`
- Modify: `src/control/drive_control.cpp:2978-2981`
- Modify: `include/config.h:163` comment only
- Modify: `configs/config.json:116` using an index-only hunk
- Modify: `Xcar2.md:265,395`

**Interfaces:**
- Replaces: `cv::Point tc_goldMappedPointFromBox(const cv::Rect&, int)`.
- Produces: `int tc_goldMappedYFromBox(const cv::Rect&, int)`.
- Preserves: `void tc_applyGoldMappedCenter(TrackedObject&, int)`, now mutating only `center_y`.

- [ ] **Step 1: Change tests and inverse fixtures to require the y-only API**

Replace `goldMappedYK1FormulaOk()` in `test/test_gold_slow_band.cpp` with:

```cpp
static bool goldMappedYK1FormulaOk()
{
    const float saved = config().tc.goldMappedYK1;

    config().tc.goldMappedYK1 = 1.4f;
    const int y100 =
        tc_goldMappedYFromBox(cv::Rect(40, 100, 20, 30), 240);
    const int y110 =
        tc_goldMappedYFromBox(cv::Rect(40, 110, 20, 30), 240);
    const int upper_clamped =
        tc_goldMappedYFromBox(cv::Rect(40, 200, 20, 30), 240);
    const int lower_clamped =
        tc_goldMappedYFromBox(cv::Rect(40, -100, 20, 0), 240);

    config().tc.goldMappedYK1 = 0.5f;
    const int y100_k05 =
        tc_goldMappedYFromBox(cv::Rect(40, 100, 20, 30), 240);
    const int y110_k05 =
        tc_goldMappedYFromBox(cv::Rect(40, 110, 20, 30), 240);
    const int height_only =
        tc_goldMappedYFromBox(cv::Rect(40, 0, 20, 6), 240);
    config().tc.goldMappedYK1 = saved;

    return y100 == 165 &&
           y110 == 179 &&
           y110 - y100 == 14 &&
           y100_k05 == 75 &&
           y110_k05 == 80 &&
           y110_k05 - y100_k05 == 5 &&
           height_only == 5 &&
           upper_clamped == 239 &&
           lower_clamped == 0;
}
```

Use `K1=1.0f` for control fixtures so integer box rows can reproduce every requested mapped row exactly. Replace `makeGoldAtFoot()` with:

```cpp
static TrackedObject makeGoldAtFoot(int foot_x, int foot_y, int box_size = 16)
{
    constexpr float kFixtureK1 = 1.0f;
    constexpr float kMappedHeightK2 = 5.0f / 6.0f;
    const float box_y =
        ((float)foot_y - kMappedHeightK2 * (float)box_size) / kFixtureK1;
    TrackedObject g;
    g.class_id = GOLD;
    g.score = 0.95f;
    g.box = cv::Rect(foot_x - box_size / 2,
                     (int)std::lround(box_y),
                     box_size, box_size);
    g.center_x = foot_x;
    g.center_y = foot_y;
    g.frame_id = 1;
    return g;
}
```

At the start of `main()`, set `config().tc.goldMappedYK1 = 1.0f`.

Before calling `tc_applyGoldMappedCenter(mapped_gold_center, 240)`, set:

```cpp
mapped_gold_center.center_x = 73;
```

Change the mapped-center assertion to require `center_x == 73` and `center_y == 108`.

In `test/test_gold_outside_record_control.cpp`, use:

```cpp
constexpr float kFixtureK1 = 1.0f;
constexpr float kMappedHeightK2 = 5.0f / 6.0f;
const float boxY =
    ((float)y - kMappedHeightK2 * (float)boxH) / kFixtureK1;
```

and set `config().tc.goldMappedYK1 = 1.0f` in `resetFixture()`.

In `test/test_vehicle_gold_source_driven_control.cpp`, use the same direct inverse:

```cpp
constexpr float kFixtureK1 = 1.0f;
constexpr float kMappedHeightK2 = 5.0f / 6.0f;
const float boxY =
    ((float)mappedY - kMappedHeightK2 * (float)boxH) / kFixtureK1;
```

and set `tc.goldMappedYK1 = 1.0f` in `ElementHarness`.

- [ ] **Step 2: Build to verify RED**

Run:

```bash
cmake --build test/build --target test_gold_slow_band -j2
```

Expected: compilation fails because `tc_goldMappedYFromBox` does not yet exist and the user's partial edit still declares a point-returning helper.

- [ ] **Step 3: Implement the y-only production API**

In `include/trackcontrol.h`, replace the partial helper and apply function with:

```cpp
inline int tc_goldMappedYFromBox(const cv::Rect& box, int img_h)
{
    constexpr float kGoldMappedHeightK2 = 5.0f / 6.0f;
    const float mapped_y =
        config().tc.goldMappedYK1 * (float)box.y +
        kGoldMappedHeightK2 * (float)std::max(0, box.height);
    return std::clamp((int)std::lround(mapped_y),
                      0, std::max(0, img_h - 1));
}

inline void tc_applyGoldMappedCenter(TrackedObject& o, int img_h)
{
    if (o.class_id != GOLD) return;
    o.center_y = tc_goldMappedYFromBox(o.box, img_h);
}
```

In `src/control/drive_control.cpp`, use:

```cpp
static inline Point tcGoldFootPoint(const TrackedObject& g)
{
    return Point(g.center_x, tc_goldMappedYFromBox(g.box, g_img_h));
}
```

Update the `include/config.h` comment without changing the `0.4f` default:

```cpp
float goldMappedYK1 = 0.4f; // mapped_y 中 box.y 的总斜率；box.y 每增 10，mapped_y 增加 10*k1
```

Keep the active JSON line at `"goldMappedYK1": 1.4`.

- [ ] **Step 4: Update operator documentation**

In the configuration table, use:

```markdown
| `goldMappedYK1` | 金币纵向映射中 `box.y` 的总斜率；公式为 `mapped_y = goldMappedYK1 * box.y + (5/6) * box.height`，编译默认 `0.4` |
```

In the gold mapping section, use:

```markdown
- AI 检测返回后先调用 `tc_applyGoldMappedCenter()`：只按 `mapped_y = goldMappedYK1 * box.y + (5/6) * box.height` 计算、四舍五入并限制金币 y；检测器提供的 `center_x` 保持不变。`goldMappedYK1` 是 `box.y` 的总增长斜率，K2 固定为精确 `5/6`。后续区域、可达性、拉线和调试点均使用原 `center_x` 与映射后的 y。
```

- [ ] **Step 5: Run focused tests to verify GREEN**

Run:

```bash
cmake --build test/build --target test_gold_slow_band -j2
test/build/bin/test_gold_slow_band
```

Expected: build and test exit 0; output reports `gold_ai_mapped_center=(73,108)` and no mapping mismatch.

- [ ] **Step 6: Build application and run gold regressions**

Build `main` and these targets:

```bash
cmake --build build --target main -j2
cmake --build test/build --target \
  test_gold_band_visual_config test_config_cleanup test_gold_slow_band \
  test_gold_outside_record_config test_gold_outside_record_control \
  test_gold_band_visual_overlay test_vehicle_gold_source_driven_control -j2
```

Run all seven executables. Expected: all exit 0.

- [ ] **Step 7: Verify scope and commit owned hunks**

Run `git diff --check` and inspect the complete diff. Stage the implementation files and only the `goldMappedYK1: 1.4` hunk from `configs/config.json`; leave `goldTrackWidthAddOuter`, `carLeavingDistM`, and other user tuning unstaged.

Commit with:

```bash
git commit -m "fix: map only gold y coordinate"
```
