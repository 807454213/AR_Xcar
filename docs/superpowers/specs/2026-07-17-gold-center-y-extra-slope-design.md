# Gold Center-Y Extra-Slope Design

## Goal

Make the gold mapped y-coordinate move in the same direction as the detected gold center y-coordinate, but slightly faster. Keep the detector-provided x-coordinate unchanged.

This design supersedes the y-formula in `2026-07-17-gold-mapped-y-only-design.md`. The earlier formula treated `box.y` as the source coordinate, but OpenCV `cv::Rect::y` is the detection box's top edge rather than its center.

## Mapping Formula

Derive the source gold y-coordinate from the detection-box center:

```cpp
gold_y = box.y + box.height / 2.0f;
mapped_y = (1.0f + goldMappedYK1) * gold_y
         + (2.0f / 6.0f) * box.height;
```

`goldMappedYK1` is the extra positive growth rate above the source center-y rate. With a fixed box height, increasing `gold_y` by 10 increases `mapped_y` by `10 * (1 + goldMappedYK1)`. For example, `goldMappedYK1 = 0.25` produces an ideal increase of 12.5 pixels before integer rounding.

The height coefficient is the fixed exact constant `2.0f / 6.0f`; it is not configurable. Use a non-negative height, round the final result with `std::lround`, and clamp it to `[0, img_h - 1]`.

Set the compiled default and the active tuning value of `goldMappedYK1` to `0.25`. Preserve all unrelated values in `configs/config.json`.

## Coordinate Handling

- `tc_goldMappedYFromBox()` returns only the mapped integer y-coordinate.
- `tc_applyGoldMappedCenter()` updates only `TrackedObject::center_y` for gold detections.
- `TrackedObject::center_x` remains the detector-provided value.
- Control, reachability, band classification, guidance, and debugging continue to consume the unchanged x-coordinate together with the mapped y-coordinate.

## Tests

Add or update focused tests that prove:

1. The formula uses `box.y + box.height / 2.0f`, not the top edge alone.
2. At fixed height and positive K1, increasing the gold center y increases mapped y faster than one-to-one, allowing for final integer rounding.
3. `goldMappedYK1 = 0.25` places representative mapped points below the corresponding detection-box center and bottom edge.
4. K2 contributes exactly `2/6` of box height.
5. The mapped result clamps to the image bounds.
6. Applying gold mapping preserves `center_x`.
7. Configuration load/save preserves `goldMappedYK1`, and its compiled default is `0.25`.

Run the focused gold tests and the main application build after implementation.
