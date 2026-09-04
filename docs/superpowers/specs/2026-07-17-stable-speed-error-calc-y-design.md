# Stable-Speed Error Calculation Row Design

## Goal

Give `STABLE_SPEED` its own configurable error-sampling row so an in-track gold target cannot move the row used to calculate steering error.

## Configuration Contract

Add `tc.stableSpeedErrorCalcY` as an integer `TrackControlParams` field. Its compiled default is `175`, matching the existing compiled `errorCalcY` default. The active `configs/config.json` value was initially approved as `140` and then tuned by the user to `135`; preserve the newer value.

The normal config loader and saver must read and write the key. When an older configuration omits the key, inherit that file's loaded `errorCalcY` value so existing presets retain their previous stable-speed behavior.

## Control Behavior

The top-level drive state is selected before the dynamic error row is calculated. When that selected state is `DriveState::StableSpeed`:

- Set `dynamic_error_y` to `stableSpeedErrorCalcY`, clamped to the image range.
- Do not let an in-track gold candidate replace that row with its mapped y.
- Keep the existing guidance source. In-track gold may still shape the guidance curve, and the final error is sampled from that curve at the fixed stable-speed row.
- Do not apply the sign OCR error-row offset because `stableSpeedErrorCalcY` is the complete row for this state.

All other states retain the existing `errorCalcY`, sign offset, gold dynamic-row, and avoidance behavior.

## Scope

- Modify the config model and JSON load/save path.
- Add `stableSpeedErrorCalcY` only to the active `configs/config.json`; preserve its current tuned value `135` and all unrelated tuning edits.
- Add a focused regression to the existing gold/state control test.
- Update the operator configuration documentation.
- Do not change drive-state priority, stable-speed entry timing, guidance generation, motion mode 8, or gold eligibility.

## Verification

The regression enters `STABLE_SPEED` with an eligible in-track gold whose mapped y differs from both configuration rows. It verifies that `dynamic_error_y` equals `stableSpeedErrorCalcY`, that gold guidance remains active, and that `final_error` is the guidance-curve value at that fixed row. Existing gold control and config round-trip tests must remain green.
