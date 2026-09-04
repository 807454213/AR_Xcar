# Gold Slow Fixed Error Y Design

## Goal

Change gold slow / follow-gold error sampling from a moving, weighted row based on the deepest gold point to a fixed configured row.

## Scope

Add a new track-control config parameter:

```json
"goldSlowErrorCalcY": 170
```

When the controller is in `FOLLOW_GOLD` / gold slow behavior:

- `r.dynamic_error_y` is set to `goldSlowErrorCalcY`.
- Final gold error is calculated from a single interpolated point on `r.guidance_curve` at `goldSlowErrorCalcY`.
- `goldErrorCalcBand` is not used for gold slow error calculation.
- `goldErrorFixedYMin` remains in config for compatibility, but no longer affects gold slow error-row selection.

## Non-Goals

- Do not change gold target selection.
- Do not change gold lock / lost-hold behavior.
- Do not change gold guidance curve generation except as required to keep the fixed error row inside the generated curve range.
- Do not remove existing config keys.

## Architecture

The existing follow-gold path computes:

1. Gold target points.
2. Dynamic work-zone bounds.
3. Guidance curve through the gold points.
4. Final error from the guidance curve.

The new behavior changes only the error-row selection and final gold error calculation:

- The work-zone bounds still cover the relevant gold path points so the curve can pass through gold targets.
- The fixed error row is included in the work zone.
- The final error uses `interpY(guidance_curve, goldSlowErrorCalcY) - image_center_x`.

## Config

Add `goldSlowErrorCalcY` to:

- `include/config.h`
- `src/io/config.cpp` load/save
- `configs/config.json`
- `configs/config2.json`

Default value is `170`.

## Testing

Update `test_gold_slow_band` so cases that previously expected gold slow to use the deepest mapped gold y now expect `dynamic_error_y == 170`.

Add or adjust an assertion that compares `final_error` to a single-row interpolation at y=170, not to `goldErrorCalcBand` weighted error.
