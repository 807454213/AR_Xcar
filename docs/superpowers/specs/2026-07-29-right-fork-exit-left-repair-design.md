# Right Fork Exit Left Repair Design

## Context

The vehicle has already entered the right fork branch and later merges back into
the main road. At that merge point the PPSeg mask can show a brief left-edge
jump and intermittent branch fragments. The current `ForkExit` repair sometimes
does not enter the exit repair state on these frames, and in some frames it can
anchor to a near-image-edge segment instead of the real branch exit edge.

The target samples are:

- `test/img/shm_20260729_142154_247.png`
- `test/img/shm_20260729_142201_807.png`
- `test/img/shm_20260729_142207_293.png`
- `test/img/shm_20260729_142210_820.png`
- `test/img/shm_20260729_142214_631.png`
- `test/img/shm_20260729_142217_583.png`
- `test/img/shm_20260729_142220_538.png`
- `test/img/shm_20260729_142224_814.png`
- `test/img/shm_20260729_142228_370.png`
- `test/img/shm_20260729_142231_503.png`
- `test/img/shm_20260729_142233_932.png`
- `test/img/shm_20260729_142236_992.png`
- `test/img/shm_20260729_142239_283.png`
- `test/img/shm_20260729_142247_646.png`
- `test/img/shm_20260729_142251_338.png`
- `test/img/shm_20260729_142257_286.png`
- `test/img/shm_20260729_142301_606.png`
- `test/img/shm_20260729_142308_295.png`

## Goals

- Enter left-boundary `ForkExit` repair reliably for the right-branch exit
  merge frames in the provided sequence.
- Keep repair near the branch exit and main-road merge, not in the far approach
  to the fork.
- Repair from the left track boundary upward, because this right-branch exit
  rejoins the main road through the left-side merge edge.
- Prevent repair anchors from using near-image-edge noise or unrelated outer
  branch fragments.
- Avoid large centerline jumps across adjacent frames in the sequence.
- Preserve existing fork entry, left-branch, sign, UART, and control behavior.

## Non-Goals

- Do not change TC264 protocol, `UartCommander`, or drive-state priority.
- Do not change SIGN decision behavior or fork direction commands.
- Do not replace the PPSeg model or add a new model dependency.
- Do not broadly retune all fork thresholds for unrelated scenes.
- Do not rewrite historical `docs/superpowers/plans` or `specs` files.

## Proposed Approach

Use a narrow enhancement to the existing fork-exit path in
`src/perception/imgprocess.cpp`.

When the current context says the car is in or leaving a right fork branch
(`ForkPhaseHunt::Exit`, an active fork state, or `ForkScanBias::Right`), prefer
left-boundary exit evidence. The existing
`repairForkExitLeftMergeBoundary()` remains the repair primitive, but its entry
conditions become better suited to the right-branch exit merge:

- left-boundary jump evidence can be accepted when the merge row is close enough
  to the vehicle;
- right boundary stability is still required as a guard against ordinary fork
  entry;
- near-image-edge anchors are rejected before fitting the repair line;
- a short left-exit patch hold keeps the previous left repair across brief mask
  dropouts in the merge sequence.

This keeps the behavior local to the exit stage and avoids changing normal
single-lane tracking.

## Detection Rules

The enhanced left-exit path should only run when all of these are true:

- PPSeg tracking is active and fork-exit repair is enabled.
- Runtime context indicates an exit candidate rather than a first approach to a
  fork.
- The left jump merge row is at or below the trusted merge threshold for a
  right-branch exit.
- The candidate anchor x is not near the image edge.
- The repaired left boundary keeps at least `forkExitMinTrackWidth` pixels of
  road width against the selected right boundary.

When both right-boundary and left-boundary exit probes are present in the same
frame, the right-branch context should prefer the left-boundary repair.

## Patch Hold

Patch hold should be intentionally short and local:

- It is armed only after a successful left-boundary `ForkExit` repair.
- It can apply only while the fork phase is still exit-oriented and the current
  frame remains close to the previous merge area.
- It reuses the previous left repair line to update rows from the top ROI to
  the previous anchor row.
- It must stop immediately if the road has clearly returned to straight tracking
  or if the candidate would collapse road width.

The hold is a continuity aid, not a substitute for detecting the merge.

## Testing

Add a regression test using the 18 provided PNGs as a chronological sequence.
The test should:

- reset fork state and force the right-branch context where needed;
- run `processFrame()` over the sequence with PPSeg mask stabilization disabled
  for deterministic replay;
- count frames whose `ForkExitRepairState` is active with
  `side == ForkExitRepairSide::Left`;
- require left repair on the later merge frames while allowing early approach
  frames to remain unrepaired;
- fail if a left repair anchor is too close to either image edge;
- fail if sampled midline values jump too far between adjacent repaired frames.

Existing regression tests to keep green:

- `./test/build/bin/test_fork_scene_samples`
- `./test/build/bin/test_fork_exit_stable` on representative legacy samples
- the new right-fork exit sequence test

## Risks

- Over-relaxing left-jump thresholds could mistake fork entry or a wide curve
  for a fork exit. The implementation must keep the exit-context and near-merge
  gates.
- Holding repair too long could mask real straight-road recovery. The hold must
  clear as soon as straight tracking is stable.
- The provided images are RGB screenshots from the real pipeline; local RKNN
  availability is still required for full `processFrame()` replay.
