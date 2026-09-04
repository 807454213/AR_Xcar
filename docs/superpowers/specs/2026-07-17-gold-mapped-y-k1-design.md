# Gold mapped-y K1 configuration design

## Goal

Make the gold detection mapping row manually tunable without changing the existing fixed `k2` value. The camera pose remains fixed, and mapping continues to use only the detection box and image dimensions.

## Mapping formula

Replace the current mapping-row calculation with:

```text
mapped_y = box.y + goldMappedYK1 * box.y + 0.8167 * box.height
```

The result remains rounded to the nearest integer and clamped to `[0, img_h - 1]`. The mapped x coordinate remains the detection-box horizontal center.

## Configuration

Add `float goldMappedYK1 = 0.0f` to `TrackControlParams` and load it from `tc.goldMappedYK1` in `configs/config.json`.

- Default value: `0.0`
- Missing-key behavior: use the compiled default, preserving compatibility with older configuration files.
- `k2` remains a compile-time constant of `0.8167`; no separate `k2` configuration is introduced.

All production and control paths must continue to call the same shared mapping helper so HUD coordinates, gold-zone classification, locking, guidance, and tests use an identical mapped point.

## Scope

The change is limited to gold mapped-y calculation and configuration loading. It does not change perspective lane widening, gold-zone boundaries, reachability policy, UART modes, or state-machine priority.

## Verification

Update/add focused tests covering:

1. `goldMappedYK1=0.0` follows the approved formula with `k2=0.8167`.
2. A nonzero `goldMappedYK1` changes the mapped y by the expected amount.
3. Mapping still clamps at the image bounds.
4. Configuration loading reads `tc.goldMappedYK1`, and an absent key keeps the default.
5. Existing gold-control and gold-visualization tests continue to pass.
