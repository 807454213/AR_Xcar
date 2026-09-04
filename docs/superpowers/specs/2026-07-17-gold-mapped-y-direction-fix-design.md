# Gold Mapped-Y Direction Fix Design

## Goal

Make a positive `tc.goldMappedYK1` move the gold mapped point upward, with a larger upward correction for detection boxes whose `box.y` is larger.

## Mapping Formula

Use:

```text
mapped_y = box.y - goldMappedYK1 * box.y + 0.8167 * box.height
```

The mapped x remains the horizontal center of the detection box. The mapped y is rounded with `std::lround` and clamped to the image range exactly as before.

`goldMappedYK1=0.0` preserves the base formula. A positive value produces an upward correction of `goldMappedYK1 * box.y`; therefore the correction grows as the box moves toward the bottom of the image. The active configured value `0.15` remains positive and does not need migration.

## Scope

- Change only the sign of the configurable `box.y` correction in `tc_goldMappedPointFromBox()`.
- Update the configuration comment and `Xcar2.md` formula/parameter explanation.
- Keep the key name, default value, fixed height coefficient `0.8167`, x mapping, rounding, and clamping unchanged.
- Preserve unrelated working-tree changes, including the user's active `configs/config.json` tuning.

## Verification

The focused mapping test will verify:

1. `goldMappedYK1=0.0` retains the existing base mapped y.
2. A positive coefficient moves the mapped y upward.
3. With equal box heights, a larger `box.y` receives a larger upward correction.
4. Image-bound clamping remains correct.

Then rebuild `main` and run the existing gold configuration, slow-band, outside-record, visual-overlay, and source-driven control regression tests.
