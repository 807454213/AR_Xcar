# Fork Left Sustain Repair Design

## Context

The supplied `/home/orangepi/xcar_shm_test/shm_20260715_214*.png` fork frames currently enter `ForkEntry` for the early part of the sequence, but `test_fork_entry_left` reports only 14/19 frames with the centerline on the left branch. The failures occur after the strict entry pull drops out while the car is still in the fork area. The centerline then follows the wider/right-side merged region instead of staying on the left branch.

The requested scope is only `ForkScanBias::Left`. Right-branch symmetry is out of scope.

## Design

Add a narrow "left sustain repair" path in the perception layer after the existing `detectAndApplyForkEntryPull()` attempt fails. It should activate only when all of these are true:

- `getForkScanBias() == ForkScanBias::Left`
- sign geometry blocking is not active
- the current or recent fork state indicates the vehicle is still in a fork context, such as stable `ForkEntry`, an existing entry patch hold, or fork-width/entry metrics that still indicate a fork
- the raw boundary has enough near-field left/right edge rows to fit a stable line

The repair will use the visible near-field left lane edge plus a repaired right edge fitted from fork gap/inner-edge geometry and the lower-right track boundary. It will rewrite `bd.left`, `bd.right`, `bd.mid`, `selectedLeft`, and `selectedRight` for the active ROI rows, then save a normal fork-entry patch hold so later frames can reuse the repaired boundary while the fork width remains active.

## Data Flow

`imgprocessTrackPpSegRaw()` already builds a PPSeg mask and raw `TrackBoundary`, classifies fork phase, tries entry pull, and then optionally applies patch hold or exit repair. The new repair stays in this same section:

1. Try the existing entry pull.
2. If it fails and the bias is left, try left sustain repair.
3. If that succeeds, mark `ForkEntryState.active`, rebuild mids, and save patch hold.
4. Continue through the existing road-mode and centerline computation.

## Testing

Add or extend a fork regression test that runs exactly the 19 supplied frames under `ForkScanBias::Left`. The expected result is that each frame's computed midline at `tc.errorCalcY` is closer to the left branch than the right branch, raising the current result from 14/19 to 19/19.

Existing fork tests and the sign gate behavior must still pass.
