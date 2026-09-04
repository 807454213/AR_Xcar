# Person Band Visual Design

## Goal

Add a debug-only visualization for the pedestrian track expansion band, matching the existing gold band overlay workflow.

## Design

Introduce `tc.personBandVisualEnabled`, defaulting to `false`. When both `app.debugOverlay` and this flag are true, `tc_process` draws pedestrian band markers over the camera frame using the same row-sampled overlay style as the gold band visualization.

The overlay uses the existing pedestrian widening math:

- Cyan dots: original PPSeg left/right track edges.
- Orange dots: outward pedestrian expansion edges from `personTrackWidthAdd`.
- Yellow dots: inward pedestrian band edges from `personTrackWidthInward`.

The overlay is purely visual. It must not change pedestrian FSM state, guidance, UART commands, or race-mode behavior.

## Files

- `include/config.h`: add the flag to `TrackControlParams`.
- `src/io/config.cpp`: load and write the flag.
- `configs/config.json`: include the new flag, default `false`.
- `src/control/drive_control.cpp`: draw the overlay near the gold band overlay.
- `test/test_person_band_visual_overlay.cpp`: verify the debug and flag gates.

## Testing

Build and run `test_person_band_visual_overlay`. The test compares frames with the flag off/on and confirms:

- `debugOverlay=false` suppresses drawing.
- `debugOverlay=true` draws pixels when `personBandVisualEnabled=true`.
- The overlay does not draw white text into the HUD band.
