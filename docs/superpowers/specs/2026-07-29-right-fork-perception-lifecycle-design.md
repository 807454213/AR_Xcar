# Right Fork Perception Lifecycle Design

## Status

Approved in discussion on 2026-07-29.

## Context

The vehicle can encounter the same fork geometry in three materially different
situations:

1. It approaches the fork entrance and stays on the main road.
2. It stays on the main road and later passes the branch exit merge.
3. It turns into the right branch and later merges back into the main road.

Commit `9855d7a` improved case 3 by treating `ForkScanBias::Right` as right-branch
exit context and requiring several near-merge left-jump candidates. That context
is too broad. A locked or stale Right bias can allow left-boundary exit repair
while the vehicle is still at the entrance, and the candidate counter is not
updated on every frame that should interrupt the sequence.

This design replaces the broad Right-bias gate with a perception-local journey
lifecycle. It refines the context portion of
`2026-07-29-right-fork-exit-left-repair-design.md`; the existing left-boundary
repair primitive and its anchor/track-width safety checks remain in use.

## Reference Data

The design uses these recordings as chronological ground truth:

- `/home/orangepi/Videos/OneCycle.mp4`
  - `6.6-9.8s`: vehicle remains on the main road and passes the branch exit.
  - `21.5-23.8s`: vehicle reaches the fork entrance and continues straight.
- `/home/orangepi/Videos/RightFork.mp4`
  - `2.4-5.0s`: vehicle reaches the same entrance and turns right.
  - `9.6-12.8s`: vehicle leaves the right branch and merges into the main road.

The recordings are slower than normal tuning runs. The expected maximum runtime
speed is approximately two to three times the recording speed.

The existing PNG fixtures remain the focused positive and negative samples:

- 18 `shm_20260729_14*.png` frames: right-branch exit merge.
- 12 `shm_20260717_21*.png` and `shm_20260717_22*.png` frames: main-road pass at
  the branch exit.
- 5 `shm_20260717_2313*.png` frames: fork entrance.

## Goals

- Permit left-boundary exit repair only after perception has confirmed that the
  vehicle actually entered the right branch.
- Trigger the existing left repair near the right-branch exit merge on at least
  the currently repaired 13 of 18 positive PNGs after valid journey pre-roll.
- Reject the new left repair at the fork entrance and while passing the branch
  exit on the main road.
- Preserve the existing main-road right-boundary exit handling.
- Remain effective when only one third of the slow recording's frames are
  available.
- Keep the change entirely within perception behavior.

## Non-Goals

- Do not modify `src/control/drive_control.cpp`, control priority, steering, or
  speed policy.
- Do not modify SIGN recognition, SIGN priority, UART messages, or TC264
  protocol.
- Do not infer a right turn when no SIGN decision exists. No SIGN continues to
  mean the existing Left/straight default; a SIGN decision continues to take
  priority.
- Do not replace PPSeg, add a model, use map position, or depend on odometry.
- Do not broadly retune generic fork-entry, fork-exit, or curve behavior.
- Do not make a direct Right bias sufficient proof of right-branch occupancy.

## Considered Approaches

### 1. Perception-local journey lifecycle

Observe the existing Right intent, then require the ordered visual events of a
right entrance, rightward branch handoff, branch interior, and left-side exit
merge. This is the selected approach because temporal order separates similar
entrance and exit shapes without changing control behavior.

### 2. Geometry-only exit classifier

Classify each frame from left/right boundary jumps and merge position. This is
simple, but entrance and exit masks can look similar in isolated frames. It is
also sensitive to thresholds and cannot explain whether the vehicle arrived
from the right branch.

### 3. Map, yaw, or odometry gate

Use vehicle pose to decide whether the branch exit is expected. This can be a
future independent safeguard, but it is map-specific, vulnerable to drift, and
outside the requested perception-only scope.

## Core Invariants

1. `ForkScanBias::Right` may start observation but may never directly authorize
   left-boundary exit repair.
2. Only a confirmed right-branch journey may select the right-branch-specific
   left-exit path.
3. Right intent must be followed by visual right-entry commitment and a
   single-lane branch handoff in that order.
4. Once the branch is visually confirmed, clearing the bias to `None` must not
   erase branch occupancy before the exit.
5. Current-frame exit candidacy is updated once per processed perception frame.
   A non-candidate frame always clears the consecutive candidate count.
6. When the journey is not armed, generic main-road fork-exit behavior remains
   unchanged.

## Perception-Local State

Add a private state object in `src/perception/imgprocess.cpp`. It is not consumed
by control code.

```text
Idle
  -> AwaitRightEntry
  -> EnteringRight
  -> InRightBranch
  -> RightExitRepair
  -> Cooldown
  -> Idle
```

The state stores only bounded perception history:

- current phase and phase age;
- whether a valid right-entry pull was observed;
- maximum observed entrance split row and whether it approached the vehicle;
- short right-handoff and single-lane confirmation streaks;
- current exit-candidate streak;
- invalid-frame and normal-lane recovery streaks.

No long boundary vectors or image copies are retained.

## Input Evidence

The lifecycle reuses existing perception outputs:

- `getForkScanBias()`;
- `dualHint`, `stillForkWidthIn`, and `entrySingleLaneNear`;
- `ForkEntryState.active`, `appliedBias`, `splitY`, and `validRows`;
- raw row segments and the bottom-connected/selected track corridor;
- raw single-lane midline displacement (`midDelta`) or equivalent
  `TrackShape::RightCurve` evidence;
- `ForkPhaseMetrics.hasExitBoundary`, `exitIsLeftJump`, `exitTrusted`,
  `exitMergeY`, `exitJumpDL`, and `exitJumpDR`;
- the existing edge-anchor and minimum-track-width checks inside
  `repairForkExitLeftMergeBoundary()`.

Evidence used to advance the lifecycle is captured before right-exit repair
mutates the boundary. A successful right-entry repair may be recorded after the
entry stage and consumed on the next frame.

Expose a read-only `RightForkJourneyPhase` getter in `include/imgprocess.h`,
following the existing `getForkEntryState()` and `getForkExitRepairState()`
debug pattern. Perception tests may inspect it; control code must not consume it.

## State Transitions

### Idle

- Enter `AwaitRightEntry` only when the existing preference is
  `ForkScanBias::Right`.
- All right-branch-specific left repair is disabled.
- Ordinary main-road exit processing is untouched.

### AwaitRightEntry

This phase observes the entrance and cannot perform exit repair.

- Record right-entry evidence only when dual-branch geometry is present and the
  entrance path actually applies `ForkScanBias::Right`.
- A strong observation is one valid right-entry frame whose split row is in the
  existing near-bottom entrance band.
- Otherwise require at most two valid observations; the split row must progress
  toward the vehicle rather than remain far-field noise.
- Enter `EnteringRight` after strong evidence or the two-observation debounce.
- Return to `Idle` immediately if the preference changes to Left.
- Return to `Idle` when entrance geometry resolves without right-entry evidence.

### EnteringRight

This phase distinguishes intent from physical route.

- Continue recording the maximum split row as the fork tip moves down the ROI.
- Confirm a right handoff using one of these image-space signatures:
  - the bottom-connected corridor transfers to the right side of the dual-branch
    split near the vehicle; or
  - a successful right-entry selection is followed immediately by a single-lane
    rightward midline displacement/RightCurve shape.
- The handoff must occur in the entrance-to-single-lane transition window. A
  right curve found later elsewhere on the map cannot retroactively arm it.
- When dual geometry disappears, require a valid near-field single lane and the
  right-handoff evidence. Strong handoff evidence may confirm in one frame;
  ordinary evidence confirms in two valid frames.
- Enter `InRightBranch` only after that confirmation.
- If the fork becomes a stable single straight/main-road lane without the
  right-handoff signature for two valid frames, return to `Idle`. This rule also
  protects the forced or stale Right-bias stress case.
- Bias clearing to `None` is tolerated after right-entry evidence has been
  recorded. A change to Left before commitment cancels the journey.

### InRightBranch

- Keep the phase through ordinary single-lane branch frames even if
  `ForkScanBias` becomes `None`.
- Do not require a long frame streak; the ordered entrance and handoff events
  are the branch-occupancy proof.
- Only this phase may accumulate right-branch left-exit candidates.
- A new dual-branch entrance pattern, prolonged invalid tracking, or the safety
  timeout resets the journey.

### RightExitRepair

A normal candidate requires all of the following:

- the phase was `InRightBranch` or is already `RightExitRepair`;
- `hasExitBoundary`, `exitIsLeftJump`, and `exitTrusted` are true;
- the left jump is dominant and the right boundary remains within the existing
  right-stability limit;
- the merge row is at or below the existing trusted left-merge threshold;
- a read-only preflight of the proposed repair anchor passes the existing edge
  guard and minimum track width.

Normal evidence triggers after two consecutive candidate frames. Strong
evidence may trigger in one frame when:

- the merge is at least the existing deep-merge pad inside the trusted region;
- the left jump exceeds the right jump by the existing dominance margin; and
- the right boundary, anchor, and track width all pass their normal guards.

The candidate update runs on every PPSeg perception frame, even when the
generic `doExit` decision is false. Any non-candidate or invalid frame clears
the candidate streak. An invalid frame does not apply a held repair.

After the streak reaches its threshold, the existing repair primitive
revalidates the anchor and width before mutating the boundary. A rejected repair
clears the streak and leaves the phase at `InRightBranch`; only a successful
repair enters `RightExitRepair`.

After successful repair, remain in this phase only while the merge evidence is
current. Two valid ordinary single-lane recovery frames move to `Cooldown`.

### Cooldown

- Block right-branch-specific left repair.
- Return to `Idle` after two valid ordinary single-lane frames with no exit
  evidence.
- A reset request clears the phase immediately.

## Integration With Existing Exit Logic

Replace the meaning of the current broad helpers, not the generic classifier:

- `forkExitRightBranchContext()` becomes true only for an armed
  `InRightBranch`/`RightExitRepair` journey, never for raw Right bias.
- The locked-Right exception that permits `doExit` is allowed only for that
  armed journey.
- An armed journey counts as runtime exit context after the control-side bias is
  cleared.
- Right intent in `AwaitRightEntry` or `EnteringRight` explicitly blocks the new
  left-exit override.
- With no armed right journey, existing right-boundary merge repair and legacy
  non-right behavior keep their current selection rules.

This keeps the change localized to right-branch context gating and the update
order of its candidate counter. It does not change steering data, control state,
or SIGN decisions.

## Speed and Frame-Drop Behavior

The normal vehicle is expected to move two to three times faster than the
recordings. State advancement therefore follows spatial events, not long fixed
frame windows:

- split row approaching the bottom of the ROI;
- right-side corridor handoff;
- dual geometry becoming a right-turn single lane;
- trusted left merge approaching the vehicle.

Frame counts are limited to one- or two-frame debounce and loose safety caps.
The initial private safety caps are:

- `AwaitRightEntry`: 360 processed frames;
- `EnteringRight`: 180 processed frames;
- `InRightBranch`: 600 processed frames;
- total armed journey: 1200 processed frames.

These caps are leak protection only and must not be used as positive evidence.
They are private perception constants for this change; no config schema change
is part of the design.

## Invalid Data and Reset Behavior

- One or two consecutive invalid tracking frames preserve an already confirmed
  phase but clear the exit-candidate streak.
- The third consecutive invalid frame resets the journey.
- `AwaitRightEntry` resets when Right intent disappears before any entry
  evidence or changes to Left.
- `EnteringRight` resets when the immediate post-fork lane is stable but lacks
  the right-handoff signature.
- `InRightBranch` does not reset merely because bias becomes `None`.
- Existing perception reset entry points, including `resetForkPhaseHunt()`,
  clear the new lifecycle and every streak.
- Phase transitions clear counters owned by the previous phase.
- Timeouts always fail closed: they disable the new left repair and return to
  `Idle`.

## Test Strategy

### Test-driven regression

Before production changes, add or revise tests so the current broad Right gate
fails the new negative assertions. Tests should drive `processFrame()` and
observe both `ForkExitRepairState` and the read-only
`RightForkJourneyPhase` getter.

### Positive right-branch replay

Replay `RightFork.mp4` chronologically:

- no left-exit repair during `2.4-5.0s`;
- confirm the right journey only after entrance handoff to a single lane;
- produce left-exit repair during `9.6-12.8s`;
- preserve anchor edge guards and maximum repaired-midline jump limits.

Run the replay at:

- every frame;
- every second frame with offsets 0 and 1;
- every third frame with offsets 0, 1, and 2.

After chronological entrance/interior pre-roll, replay the 18 positive PNGs.
At least 13 must report active `ForkExitRepairSide::Left`, and repair should
begin on the first geometrically trusted near-merge frame rather than after a
long fixed streak.

### Main-road replay

Replay `OneCycle.mp4` with the same step and offset matrix:

- `6.6-9.8s` must never arm the right-branch lifecycle or add left repair;
- existing main-road right-boundary exit behavior must retain its baseline
  repair count, side, first activation frame, and representative midline values;
- `21.5-23.8s` must not arm the right-branch lifecycle when continuing straight.

Run a bounded stress replay with Right forced through the entrance and immediate
post-entrance handoff. The straight/main-road single lane must reject the
journey rather than advance to `InRightBranch`.

### Focused negative fixtures

- The 12 main-road exit PNGs produce zero new left repairs.
- The 5 entrance PNGs produce zero new left repairs, including locked/stale
  Right stress.
- Directly replaying exit PNGs with Right bias but no entrance/interior history
  produces zero new left repairs.
- An interrupted exit candidate sequence clears its streak instead of combining
  non-consecutive candidates.
- A perception reset between candidate frames clears all lifecycle history.

### Existing regressions

Run the focused perception suite, including:

- `test_fork_scene_samples`;
- `test_fork_exit_stable`;
- `test_fork_entry_left`;
- `test_fork_entry_width`;
- `test_fork_r_frames`;
- `test_right_fork_exit_left_repair`;
- `test_road_straight_curve_samples`.

Run existing SIGN/control tests as isolation checks even though their source is
unchanged.

## Acceptance Criteria

- The new left-boundary exit repair is unreachable from Right bias alone.
- Every positive replay step/offset variant reaches right-exit left repair.
- The pre-rolled 18-image positive sequence keeps at least 13 repaired frames.
- All entrance, main-road exit, direct-exit, stale-Right, and reset negative
  cases produce zero newly introduced left repairs.
- Main-road legacy right-boundary repair metrics match the pre-change baseline.
- Exit candidates never accumulate across a non-candidate frame.
- The implementation diff contains no control, SIGN, UART, or protocol changes.
- All focused perception and isolation regression tests pass.

## Expected File Scope

- Modify `src/perception/imgprocess.cpp`.
- Modify or add perception tests under `test/` and register them in
  `test/CMakeLists.txt`.
- Modify `include/imgprocess.h` only to expose the read-only perception phase
  getter used by tests.
- Do not modify `configs/config.json`.

## Risks and Mitigations

- **Right intent mistaken for physical entry:** require the immediate right-side
  corridor handoff and single-lane right-turn geometry.
- **Fast motion skips an intermediate frame:** accept strong one-frame spatial
  evidence and validate every decimation offset.
- **A later right curve arms stale state:** permit handoff confirmation only in
  the entrance-to-single-lane transition window and apply phase timeouts.
- **Segmentation dropout leaks state or combines candidates:** clear the
  candidate every non-candidate frame and reset after three invalid frames.
- **Main-road behavior changes accidentally:** keep generic exit selection
  outside the lifecycle and compare recorded baseline metrics.
- **State survives into another lap:** reset after recovery, on all perception
  reset entry points, on new fork geometry, and on loose safety timeout.
