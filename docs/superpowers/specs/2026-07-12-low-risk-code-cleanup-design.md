# Low-Risk Code Cleanup Design

## Goal

Reduce repository noise and remove configuration fields that cannot affect the
current program, without changing perception, control, UART, or state-machine
behavior.

Only the current `configs/config.json` and `configs/config2.json` formats need
to remain supported. Historical configuration compatibility is not required.

## Scope

The cleanup has three parts:

1. Stop tracking generated build output and ignore it in future commits.
2. Remove configuration fields with no runtime consumers.
3. Rename the misleading gold lock-matching parameter without changing its
   value or behavior.

This work does not split large source files or remove dormant subsystems.

## Repository Hygiene

Add explicit `.gitignore` entries for these generated directories:

- `/build/`
- `/build_sign/`
- `/test/build/`
- `/test/build_sign/`
- `/test/shm_test/build/`
- `/Position/slam_workspace/slam_all/build/`

Remove their currently tracked contents from the Git index. Use an index-only
removal so existing local build files remain on disk and currently built
binaries remain available to the developer.

Do not remove or ignore:

- RKNN models under `AI/`
- test source files
- tracked test images and regression samples
- design and implementation documents

## Dead Configuration Removal

Remove the following fields from their structure declarations, JSON loading,
JSON saving, and both current configuration files:

### Image processing

- `blueHueLow`
- `blueHueHigh`
- `blueSatLow`
- `blueSatHigh`
- `blueValLow`
- `blueValHigh`
- `forkExitEntryNearSepPx`

The PPSeg path has no HSV fallback, and these fields have no consumer outside
configuration serialization. Fork entry/exit phase scoring no longer consumes
`forkExitEntryNearSepPx`.

### Track control

- `personComplexBottomDeltaMin`
- `personComplexBottomDeltaMax`
- `personCarWaitDetourBias`
- `personComplexOrangeDodgeOffset`
- `personOutEmergDodgeOffset`

These fields only remain in configuration declarations and serialization. The
current pedestrian state machine does not read them.

Before implementation, repeat a source-wide reference search excluding build
directories. If any listed field has gained a runtime consumer, leave it in
place and document the exception instead of deleting it.

## Gold Parameter Rename

Rename:

```text
goldDistThresh -> goldLockMatchRadiusPx
```

Apply the rename consistently to:

- `TrackControlParams`
- configuration loading and saving
- `configs/config.json`
- `configs/config2.json`
- controller use sites
- tests

Preserve each configuration file's current value. Preserve the controller
expression `max(24, configured value)` and the squared Euclidean comparison.
The rename must not change target selection behavior.

The new name describes the real purpose: the pixel radius used to decide
whether a current-frame gold foot point is the same target as the previously
locked gold. It is not a distance from the track and does not define the gold
reachable band.

## Explicit Non-Goals

- No changes to gold reachable-band geometry, slow modes, error calculation,
  lock fallback, or lost-target behavior.
- No changes to SIGN, pedestrian, vehicle, fork, track, or UART state machines.
- No split or refactor of `drive_control.cpp` or `imgprocess.cpp`.
- No removal of `stop_landmark` or other dormant source modules.
- No model replacement or deletion.
- No history rewrite to purge old build artifacts from previous commits.

## Validation

Validation must use a newly configured build directory after the ignore and
index cleanup, so success does not depend on stale generated files.

Required checks:

1. Source-wide searches show no remaining reference to removed field names or
   `goldDistThresh`, excluding Git history.
2. Both current JSON files parse successfully and contain
   `goldLockMatchRadiusPx` with their original values.
3. Configuration load/save tests pass.
4. `test_no_hsv_fallback` passes.
5. Gold, pedestrian, and fork regression tests pass.
6. The main target builds successfully.
7. `git diff --check` passes.
8. `git status --short` contains no generated build changes after rebuilding.

## Success Criteria

- Generated files no longer appear in normal Git status output.
- Current configuration files contain no fields that were removed in this
  design.
- The gold lock matching parameter has a name that reflects its behavior.
- Existing control and perception regression tests remain green.
- No runtime behavior is intentionally changed.
