# Gold Mapped-Y Direction Fix Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make positive `tc.goldMappedYK1` values apply an upward correction whose magnitude grows with `box.y`.

**Architecture:** Keep the existing `tc_goldMappedPointFromBox()` interface and mapping pipeline. Change only the configurable y-term from addition to subtraction, lock the direction with a focused regression test, and synchronize the configuration comment and operator documentation.

**Tech Stack:** C++17, OpenCV `cv::Rect`/`cv::Point`, CMake, JSON configuration, Markdown.

## Global Constraints

- The formula is exactly `mapped_y = box.y - goldMappedYK1 * box.y + 0.8167 * box.height`.
- Keep `goldMappedYK1` named as-is, with default `0.0f`; preserve the active configured value `0.15`.
- Keep mapped x, `std::lround`, image-bound clamping, and fixed height coefficient `0.8167f` unchanged.
- Preserve unrelated working-tree changes.

---

### Task 1: Correct and Verify the Gold Mapped-Y Direction

**Files:**
- Modify: `test/test_gold_slow_band.cpp:14-35`
- Modify: `include/trackcontrol.h:34-44`
- Modify: `include/config.h:163`
- Modify: `Xcar2.md:265,395`

**Interfaces:**
- Consumes: `cv::Point tc_goldMappedPointFromBox(const cv::Rect& box, int img_h)` and `config().tc.goldMappedYK1`.
- Produces: the same mapping function and configuration key, with positive `goldMappedYK1` values applying an upward y correction.

- [ ] **Step 1: Change the focused test to require an upward correction**

Replace `goldMappedYK1FormulaOk()` in `test/test_gold_slow_band.cpp` with:

```cpp
static bool goldMappedYK1FormulaOk()
{
    const float saved = config().tc.goldMappedYK1;

    config().tc.goldMappedYK1 = 0.0f;
    const cv::Point base =
        tc_goldMappedPointFromBox(cv::Rect(40, 60, 20, 30), 240);
    const cv::Point lower_base =
        tc_goldMappedPointFromBox(cv::Rect(40, 120, 20, 30), 240);

    config().tc.goldMappedYK1 = 0.25f;
    const cv::Point tuned =
        tc_goldMappedPointFromBox(cv::Rect(40, 60, 20, 30), 240);
    const cv::Point lower_tuned =
        tc_goldMappedPointFromBox(cv::Rect(40, 120, 20, 30), 240);

    config().tc.goldMappedYK1 = 0.0f;
    const cv::Point clamped =
        tc_goldMappedPointFromBox(cv::Rect(40, 220, 20, 40), 240);
    config().tc.goldMappedYK1 = saved;

    const int upper_correction = base.y - tuned.y;
    const int lower_correction = lower_base.y - lower_tuned.y;
    return base == cv::Point(50, 85) &&
           tuned == cv::Point(50, 70) &&
           lower_base == cv::Point(50, 145) &&
           lower_tuned == cv::Point(50, 115) &&
           lower_correction > upper_correction &&
           clamped == cv::Point(50, 239);
}
```

- [ ] **Step 2: Build and run the focused test to verify RED**

Run:

```bash
cmake --build test/build --target test_gold_slow_band -j2
test/build/bin/test_gold_slow_band
```

Expected: build succeeds and the executable exits nonzero with `gold mapped-y k1 formula/clamp mismatch`, because the current positive term maps the tuned samples downward.

- [ ] **Step 3: Implement the minimal sign correction**

In `include/trackcontrol.h`, use:

```cpp
const float mapped_y =
    (float)box.y -
    config().tc.goldMappedYK1 * (float)box.y +
    kGoldMappedHeightK2 * (float)std::max(0, box.height);
```

Do not change the x calculation, rounding, clamping, or `kGoldMappedHeightK2`.

- [ ] **Step 4: Update the parameter comment and operator documentation**

Change the `include/config.h` member comment to:

```cpp
float goldMappedYK1 = 0.0f; // mapped_y 中 box.y 的向上修正比例，k2 固定为 0.8167
```

In the `Xcar2.md` configuration table, document:

```markdown
| `goldMappedYK1` | 金币纵向映射中 `box.y` 的向上修正比例，公式为 `mapped_y = box.y - goldMappedYK1 * box.y + 0.8167 * box.height`；默认 `0.0` |
```

In the gold mapping description, use:

```markdown
- AI 检测返回后先调用 `tc_applyGoldMappedCenter()`：横坐标取检测框中心，纵坐标按 `mapped_y = box.y - goldMappedYK1 * box.y + 0.8167 * box.height` 计算、四舍五入并限制在画面高度内，再把映射点写回金币中心；后续区域判断、可达性判断、拉线和调试点均基于该映射点。`goldMappedYK1` 增大时，`box.y` 越大的检测框向上修正越多；默认 `0.0`。
```

Leave `configs/config.json` at the user's active `"goldMappedYK1": 0.15`.

- [ ] **Step 5: Run the focused test to verify GREEN**

Run:

```bash
cmake --build test/build --target test_gold_slow_band -j2
test/build/bin/test_gold_slow_band
```

Expected: build and test both exit 0; the output contains `gold_ai_mapped_center=(60,108)` for the zero-coefficient fixture and no mapping mismatch message.

- [ ] **Step 6: Build the application and run gold regressions**

Run:

```bash
cmake --build build --target main -j2
cmake --build test/build --target \
  test_gold_band_visual_config \
  test_config_cleanup \
  test_gold_slow_band \
  test_gold_outside_record_config \
  test_gold_outside_record_control \
  test_gold_band_visual_overlay \
  test_vehicle_gold_source_driven_control -j2
```

Then run each executable:

```bash
test/build/bin/test_gold_band_visual_config
test/build/bin/test_config_cleanup
test/build/bin/test_gold_slow_band
test/build/bin/test_gold_outside_record_config
test/build/bin/test_gold_outside_record_control
test/build/bin/test_gold_band_visual_overlay
test/build/bin/test_vehicle_gold_source_driven_control
```

Expected: `main` builds and all seven tests exit 0.

- [ ] **Step 7: Verify scope and commit**

Run:

```bash
git diff --check
git diff -- include/trackcontrol.h include/config.h test/test_gold_slow_band.cpp Xcar2.md configs/config.json
git status --short
```

Expected: the implementation diff contains the formula sign, focused test, comment, and documentation changes. `configs/config.json` still contains the user's `0.15` tuning and is not staged by this task.

Stage only the implementation-owned files and commit:

```bash
git add include/trackcontrol.h include/config.h test/test_gold_slow_band.cpp Xcar2.md
git diff --cached --check
git commit -m "fix: reverse gold mapped y correction"
```
