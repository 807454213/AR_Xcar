# Gold Mapped-Y Pivot Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the gold y correction with a fixed pivot mapping whose inverse slope is controlled directly by `goldMappedYK1`.

**Architecture:** Keep `tc_goldMappedPointFromBox()` as the only production mapping entry point. Calibrate it around `(box.y=120, box.height=30) -> mapped_y=150`, update the compiled/configured default to `0.4`, and make test fixtures invert the same formula so their requested mapped positions remain stable.

**Tech Stack:** C++17, OpenCV `cv::Rect`/`cv::Point`, CMake, JSON configuration, Markdown.

## Global Constraints

- Use exactly `mapped_y = 150 + goldMappedYK1 * (120 - box.y) + 0.8167 * (box.height - 30)`.
- Keep `goldMappedYK1` configurable without clamping and change its compiled default to `0.4f`.
- Preserve mapped x, `std::lround`, image-bound clamping, and the fixed `0.8167f` height coefficient.
- Preserve the user's active JSON value `0.4` and all unrelated working-tree changes.
- The calibration is intentionally fixed for the current 320×240 camera image.

---

### Task 1: Implement and Verify the Pivot Mapping

**Files:**
- Modify: `test/test_gold_band_visual_config.cpp:16-31`
- Modify: `test/test_gold_slow_band.cpp:14-67,1684`
- Modify: `test/test_gold_outside_record_control.cpp:48-73`
- Modify: `test/test_vehicle_gold_source_driven_control.cpp:29-43,89`
- Modify: `include/trackcontrol.h:34-49`
- Modify: `include/config.h:163`
- Modify: `configs/config.json:116` using an index-only hunk so unrelated tuning remains unstaged
- Modify: `Xcar2.md:265,395`

**Interfaces:**
- Consumes: `cv::Point tc_goldMappedPointFromBox(const cv::Rect& box, int img_h)` and `TrackControlParams::goldMappedYK1`.
- Produces: the same mapping function/key with a fixed pivot and direct inverse slope.

- [ ] **Step 1: Update focused tests and fixtures before production code**

In `test/test_gold_band_visual_config.cpp`, require the new default:

```cpp
const float default_k1 = config().tc.goldMappedYK1;
if (std::fabs(default_k1 - 0.4f) > 1e-6f) {
    std::cerr << "default goldMappedYK1 expected 0.4\n";
    return 1;
}

const char* missing_path = "/tmp/xcar_gold_mapped_y_config_missing.json";
if (!writeFile(missing_path, "{\n  \"tc\": {}\n}\n") ||
    !configLoad(missing_path) ||
    std::fabs(config().tc.goldMappedYK1 - 0.4f) > 1e-6f) {
    std::cerr << "missing goldMappedYK1 did not preserve default\n";
    return 10;
}
```

Replace `goldMappedYK1FormulaOk()` in `test/test_gold_slow_band.cpp` with:

```cpp
static bool goldMappedYK1FormulaOk()
{
    const float saved = config().tc.goldMappedYK1;

    config().tc.goldMappedYK1 = 0.4f;
    const cv::Point anchor =
        tc_goldMappedPointFromBox(cv::Rect(40, 120, 20, 30), 240);
    const cv::Point y_minus_10 =
        tc_goldMappedPointFromBox(cv::Rect(40, 110, 20, 30), 240);
    const cv::Point y_plus_10 =
        tc_goldMappedPointFromBox(cv::Rect(40, 130, 20, 30), 240);
    const cv::Point taller =
        tc_goldMappedPointFromBox(cv::Rect(40, 120, 20, 40), 240);
    const cv::Point clamped =
        tc_goldMappedPointFromBox(cv::Rect(40, -120, 20, 30), 240);

    config().tc.goldMappedYK1 = 0.5f;
    const cv::Point anchor_k05 =
        tc_goldMappedPointFromBox(cv::Rect(40, 120, 20, 30), 240);
    const cv::Point y_minus_10_k05 =
        tc_goldMappedPointFromBox(cv::Rect(40, 110, 20, 30), 240);
    config().tc.goldMappedYK1 = saved;

    return anchor == cv::Point(50, 150) &&
           y_minus_10 == cv::Point(50, 154) &&
           y_plus_10 == cv::Point(50, 146) &&
           taller == cv::Point(50, 158) &&
           anchor_k05 == cv::Point(50, 150) &&
           y_minus_10_k05 == cv::Point(50, 155) &&
           clamped == cv::Point(50, 239);
}
```

Replace `makeGoldAtFoot()` with an inverse-pivot fixture and set the main fixture coefficient to `0.4f`:

```cpp
static TrackedObject makeGoldAtFoot(int foot_x, int foot_y, int box_size = 16)
{
    constexpr float kFixtureK1 = 0.4f;
    constexpr float kMappedHeightK2 = 0.8167f;
    const float box_y = 120.0f -
        ((float)foot_y - 150.0f -
         kMappedHeightK2 * ((float)box_size - 30.0f)) / kFixtureK1;
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

At the start of `main()`, use:

```cpp
config().tc.goldMappedYK1 = 0.4f;
```

In `test/test_gold_outside_record_control.cpp`, replace the fixture helper and reset coefficient with:

```cpp
static TrackedObject makeGoldAtMappedPoint(int x, int y)
{
    constexpr int boxW = 18;
    constexpr int boxH = 18;
    constexpr float kFixtureK1 = 0.4f;
    constexpr float kMappedHeightK2 = 0.8167f;
    const float boxY = 120.0f -
        ((float)y - 150.0f -
         kMappedHeightK2 * ((float)boxH - 30.0f)) / kFixtureK1;
    TrackedObject gold;
    gold.class_id = GOLD;
    gold.score = 0.99f;
    gold.box = cv::Rect(x - boxW / 2, (int)std::lround(boxY), boxW, boxH);
    gold.center_x = x;
    gold.center_y = y;
    gold.frame_id = 1;
    return gold;
}
```

```cpp
config().tc.goldMappedYK1 = 0.4f;
```

In `test/test_vehicle_gold_source_driven_control.cpp`, use the same inverse calculation:

```cpp
TrackedObject makeOutsideGold()
{
    constexpr int mappedY = 183;
    constexpr int boxH = 16;
    constexpr float kFixtureK1 = 0.4f;
    constexpr float kMappedHeightK2 = 0.8167f;
    const float boxY = 120.0f -
        ((float)mappedY - 150.0f -
         kMappedHeightK2 * ((float)boxH - 30.0f)) / kFixtureK1;
    TrackedObject gold;
    gold.class_id = GOLD;
    gold.score = 0.95f;
    gold.box = cv::Rect(62, (int)std::lround(boxY), 16, boxH);
    gold.center_x = 70;
    gold.center_y = mappedY;
    gold.frame_id = 1;
    return gold;
}
```

Set `tc.goldMappedYK1 = 0.4f` in `ElementHarness`.

- [ ] **Step 2: Build and run focused tests to verify RED**

Run:

```bash
cmake --build test/build --target test_gold_band_visual_config test_gold_slow_band -j2
test/build/bin/test_gold_band_visual_config
test/build/bin/test_gold_slow_band
```

Expected: at least one executable exits nonzero. The config test reports the old compiled default, and the mapping test rejects the uncommitted experimental formula because it clamps almost every point to y 239.

- [ ] **Step 3: Implement the pivot formula and default**

Replace the body calculation in `tc_goldMappedPointFromBox()` with:

```cpp
constexpr float kGoldMappedReferenceBoxY = 120.0f;
constexpr float kGoldMappedReferenceBoxHeight = 30.0f;
constexpr float kGoldMappedReferenceY = 150.0f;
constexpr float kGoldMappedHeightK2 = 0.8167f;
const float mapped_y =
    kGoldMappedReferenceY +
    config().tc.goldMappedYK1 *
        (kGoldMappedReferenceBoxY - (float)box.y) +
    kGoldMappedHeightK2 *
        ((float)std::max(0, box.height) - kGoldMappedReferenceBoxHeight);
```

Remove the experimental coefficient clamp and hard-coded `240.0f` expression. Keep x mapping, rounding, and clamping unchanged.

In `include/config.h`, use:

```cpp
float goldMappedYK1 = 0.4f; // box.y 反向映射斜率；y 每减 10，mapped_y 增加 10*k1
```

Keep the user's active `configs/config.json` line as:

```json
"goldMappedYK1": 0.4,
```

- [ ] **Step 4: Synchronize operator documentation**

In the `Xcar2.md` configuration table, use:

```markdown
| `goldMappedYK1` | 金币纵向映射的反向斜率；公式为 `mapped_y = 150 + goldMappedYK1 * (120 - box.y) + 0.8167 * (box.height - 30)`，默认 `0.4` |
```

In the gold mapping description, use:

```markdown
- AI 检测返回后先调用 `tc_applyGoldMappedCenter()`：横坐标取检测框中心，纵坐标按 `mapped_y = 150 + goldMappedYK1 * (120 - box.y) + 0.8167 * (box.height - 30)` 计算、四舍五入并限制在画面高度内，再写回金币中心。锚点 `(box.y=120, box.height=30)` 固定映射到 `y=150`；默认 `goldMappedYK1=0.4` 时，`box.y` 每减小 10，映射点 y 增加 4。后续区域、可达性、拉线和调试点判断均使用该映射点。
```

- [ ] **Step 5: Run focused tests to verify GREEN**

Run:

```bash
cmake --build test/build --target test_gold_band_visual_config test_gold_slow_band -j2
test/build/bin/test_gold_band_visual_config
test/build/bin/test_gold_slow_band
```

Expected: both executables exit 0, and the slow-band output contains no mapping mismatch message.

- [ ] **Step 6: Build the application and run gold regressions**

Run:

```bash
cmake --build build --target main -j2
cmake --build test/build --target \
  test_gold_band_visual_config test_config_cleanup test_gold_slow_band \
  test_gold_outside_record_config test_gold_outside_record_control \
  test_gold_band_visual_overlay test_vehicle_gold_source_driven_control -j2
```

Run all seven executables. Expected: `main` builds and all tests exit 0.

- [ ] **Step 7: Verify scope and commit only owned hunks**

Run:

```bash
git diff --check
git diff -- include/trackcontrol.h include/config.h configs/config.json \
  test/test_gold_band_visual_config.cpp test/test_gold_slow_band.cpp \
  test/test_gold_outside_record_control.cpp \
  test/test_vehicle_gold_source_driven_control.cpp Xcar2.md
git status --short
```

Stage the implementation files. For `configs/config.json`, stage only the `goldMappedYK1: 0.4` hunk and leave `goldTrackWidthAddOuter`, `carLeavingDistM`, and any other user tuning unstaged. Commit with:

```bash
git commit -m "feat: pivot gold mapped y"
```
