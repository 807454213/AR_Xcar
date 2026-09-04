# Gold Band Visual Overlay Design

## Goal

Add a config-controlled debug overlay that visualizes the three gold-band tuning parameters on the main camera frame:

- `goldTrackWidthAddInner`
- `goldTrackWidthAddOuter`
- `goldReachableWidthAddOuter`

The overlay is for live tuning in `vision` or debug-recording workflows.

## Scope

Add a new `tc` config flag named `goldBandVisualEnabled`, default `false`.

When `goldBandVisualEnabled` is true and the existing debug overlay is active, draw the gold-band helper lines directly on the main frame during `tc_process()`.

The overlay must not draw when the global debug overlay is inactive. This keeps race-mode behavior and non-debug frame handling unchanged.

## Visual Behavior

For each valid track row in the active processing range:

- Draw original left/right track boundary reference points.
- Draw the inner slow-band boundary from `goldTrackWidthAddInner`.
- Draw the outer slow-band boundary from `goldTrackWidthAddOuter`.
- Draw the reachable outer boundary from `goldReachableWidthAddOuter`.
- Use distinct colors for the three generated boundary classes.

Add a compact HUD line such as:

```text
GOLD BAND inner=8 outer=22 reach=87
```

The overlay should use the same perspective row scaling already used by gold-band classification, so the displayed lines match runtime decisions.

## Architecture

Keep the implementation inside the existing track-control debug drawing path:

- `include/config.h`: add the boolean field.
- `src/io/config.cpp`: load and write the boolean in the `tc` section.
- `configs/config.json` and `configs/config2.json`: include the new flag near the three gold-band width parameters.
- `src/control/drive_control.cpp`: add a small helper that draws the band lines from existing `left_use` and `right_use` vectors using the same band calculation helpers as runtime gold logic.

No new UI framework, BEV rendering path, or keyboard runtime toggle is required.

## Testing

Use TDD with focused tests:

1. Config test proves `goldBandVisualEnabled` can be read from JSON and defaults to false.
2. Drawing test proves a frame remains unchanged when the flag is false and changes when the flag is true under debug overlay conditions.

Run the focused test target and build the main binary before reporting completion.
