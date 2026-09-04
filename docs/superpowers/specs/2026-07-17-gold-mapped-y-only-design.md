# Gold Mapped-Y Only Design

## Goal

Map only the gold detection's y coordinate with a positive configurable `box.y` slope. Preserve the detector-provided x coordinate without recomputing it from the box.

## Formula

Use:

```text
mapped_y = goldMappedYK1 * box.y + (5 / 6) * box.height
```

`goldMappedYK1` is the total `box.y` slope: increasing `box.y` by 10 increases `mapped_y` by `10 * goldMappedYK1`. `K2` is the fixed exact constant `5.0f / 6.0f` and is not configurable.

The result is rounded with `std::lround` and clamped to `[0, img_h - 1]`. Keep the compiled `goldMappedYK1` default at `0.4f`; preserve the active JSON tuning value `1.4`.

## Y-Only Interface

- Replace `cv::Point tc_goldMappedPointFromBox(...)` with `int tc_goldMappedYFromBox(...)`.
- `tc_applyGoldMappedCenter()` updates only `TrackedObject::center_y`; it must leave `center_x` unchanged.
- The control-layer gold point uses `cv::Point(g.center_x, tc_goldMappedYFromBox(g.box, g_img_h))`. It must not derive x from `box.x` or `box.width`.

This keeps production, tests, and direct control callers consistent even when Pipeline preprocessing was not run.

## Scope

- Complete and replace the user's uncommitted partial y-only edit in `include/trackcontrol.h`.
- Update the control-layer helper, focused mapping tests, inverse test fixtures, and `Xcar2.md`.
- Do not add `goldMappedYK2` or change unrelated gold zone/reachability behavior.
- Preserve unrelated working-tree configuration changes.

## Verification

Focused tests will verify:

1. `goldMappedYK1` directly controls the positive y slope.
2. The height term uses exact `5/6` behavior.
3. Mapped y is clamped to image bounds.
4. Applying the mapping does not change a deliberately non-box-centered `center_x`.
5. Existing gold control scenarios retain their requested mapped points through updated inverse fixtures.

Then rebuild `main` and run the existing gold configuration, slow-band, outside-record, visual-overlay, and source-driven control regressions.
