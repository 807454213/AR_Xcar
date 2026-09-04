# Gold Mapped-Y Pivot Design

## Goal

Map gold detections with a fixed reference point and a configurable inverse y slope: when `box.y` decreases by 10 pixels, `mapped_y` increases by `10 * goldMappedYK1` pixels.

## Formula

Use the fixed 320×240-camera calibration:

```text
mapped_y = 150
         + goldMappedYK1 * (120 - box.y)
         + 0.8167 * (box.height - 30)
```

The reference detection `(box.y=120, box.height=30)` always maps to `mapped_y=150`, independent of `goldMappedYK1`. With the default `goldMappedYK1=0.4`, reducing `box.y` by 10 increases `mapped_y` by 4, while increasing `box.y` by 10 decreases `mapped_y` by 4.

The fixed height coefficient remains `0.8167`: increasing `box.height` by 10 increases `mapped_y` by about 8.17. The mapped x remains the detection-box horizontal center. The final y is rounded with `std::lround` and clamped to the image range.

## Configuration

- Keep the key name `tc.goldMappedYK1`.
- Change its compiled default from `0.0f` to `0.4f`.
- Do not clamp the configured value; it directly controls the inverse y slope.
- Preserve the active JSON value `"goldMappedYK1": 0.4` and all unrelated user tuning.

## Implementation Scope

- Replace the uncommitted experimental hard-coded formula in `include/trackcontrol.h` with the pivot formula.
- Remove the experimental `std::clamp(..., 0.3f, 0.5f)` coefficient restriction.
- Update the configuration default/comment, focused mapping tests, configuration tests, and `Xcar2.md`.
- Do not change gold x mapping, downstream zone/reachability logic, or other control behavior.

## Verification

Focused tests will verify:

1. `(120,30)` maps to y 150 for multiple coefficient values.
2. With `goldMappedYK1=0.4`, y 110 maps to 154 and y 130 maps to 146 for height 30.
3. Changing `goldMappedYK1` changes the inverse movement rate without moving the anchor.
4. The height term and image-bound clamping remain active.
5. Missing JSON configuration retains the new `0.4f` default, while load/save round trips preserve an explicit value.

Then rebuild `main` and run the existing gold configuration, slow-band, outside-record, visual-overlay, and source-driven control regression tests.
