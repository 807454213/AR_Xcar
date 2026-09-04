# Unused Configuration Parameters Cleanup Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Remove 17 configuration parameters that have no production consumers while preserving `app.aiNpuCoreStart` compatibility and safe loading of legacy JSON.

**Architecture:** Keep the existing permissive two-level JSON parser: unknown legacy keys remain harmless on load. Remove the obsolete keys from the typed configuration model and serializer so a subsequent save naturally migrates old configuration files by omitting them.

**Tech Stack:** C++17, hand-written JSON configuration loader, CMake, JSON runtime files, Bash verification

## Global Constraints

- Remove exactly the 17 parameters listed in the approved design.
- Preserve `app.aiNpuCoreStart` in JSON, `AppParams`, load, save, tests, and current documentation.
- Preserve `camera.pitch_deg`, which feeds `cameraModel().pitch_rad`.
- Do not rewrite historical files under `docs/superpowers/specs` or `docs/superpowers/plans`.
- Do not overwrite or stage unrelated changes already present in the dirty worktree.

---

### Task 1: Add a legacy-key migration regression test

**Files:**
- Modify: `test/test_config_cleanup.cpp`

**Interfaces:**
- Consumes: `bool configLoad(const std::string&)`, `bool configSave(const std::string&)`, and `config().app.aiNpuCoreStart`.
- Produces: A regression contract proving old keys are ignored, all 17 keys are absent after save, and `aiNpuCoreStart` still round-trips.

- [ ] **Step 1: Extend the removed-key list and legacy input**

Add all approved names to `excludesRemovedKeys()`:

```cpp
"bottomAnchorRows", "forkEntryHuntSwitchSplitFrac", "minBottomDistance",
"noiseAreaThresh", "roadForkLowVarMax", "roadForkLowVarMinRows",
"roadForkTinyVarMax", "roadForkVarMax", "rowSelMaskBandExtraPx",
"rowSelMergeGapPx", "rowSelRefMidXInitRatio", "carAvoidLockFrames",
"carHalfWidth", "errorCalcBand", "personApproachMargin",
"personResumeExitCenterThr", "positionLogEnabled"
```

Write a temporary legacy JSON containing representative removed keys plus `"aiNpuCoreStart": 1`, load it, save it, and assert the saved text excludes every removed key but contains `"aiNpuCoreStart": 1`.

- [ ] **Step 2: Build and run the test to verify RED**

Run:

```bash
cmake --build test/build -j2 --target test_config_cleanup
./test/build/bin/test_config_cleanup
```

Expected: the executable exits non-zero because the current serializer still emits the newly listed obsolete keys.

- [ ] **Step 3: Leave implementation unchanged until the RED failure is recorded**

Do not modify `include/config.h`, `src/io/config.cpp`, or runtime JSON before observing the expected failure.

---

### Task 2: Remove the obsolete typed configuration interface

**Files:**
- Modify: `configs/config.json`
- Modify: `configs/config_stable.json`
- Modify: `include/config.h`
- Modify: `src/io/config.cpp`
- Modify: `test/test_ai_inference_mode_config.cpp`
- Modify: `test/test_gold_slow_band.cpp`
- Modify: `test/test_vehicle_gold_source_driven_control.cpp`
- Modify: `test/test_ped_source_driven_control.cpp`

**Interfaces:**
- Consumes: the exact 17-key removal set from Task 1.
- Produces: `AppConfig` without obsolete members and a serializer that cannot regenerate obsolete keys.

- [ ] **Step 1: Remove runtime JSON entries**

Delete the 17 exact key/value pairs from both JSON files while preserving all neighboring values, ordering, credentials, and valid comma placement. Keep `"aiNpuCoreStart"` unchanged.

- [ ] **Step 2: Remove C++ members**

Delete the corresponding 11 `ImgProcessParams` members, five `TrackControlParams` members, and one `AppParams` member with their obsolete comments from `include/config.h`.

- [ ] **Step 3: Remove loader and serializer statements**

Delete each `jInt`, `jFloat`, `jDouble`, or `jBool` assignment for the removed names from `configLoad()`, and delete each matching `fprintf` from `configSave()`. Do not change the lines for `aiNpuCoreStart` or `pitch_deg`.

- [ ] **Step 4: Remove stale test dependencies**

Remove `errorCalcBand` input, member assertions, and diagnostics from `test_ai_inference_mode_config.cpp`. Remove the unused `symmetricCurveError()` helper and the single `config().tc.errorCalcBand` setup line from `test_gold_slow_band.cpp`. Remove the two remaining harness assignments in the source-driven control tests.

- [ ] **Step 5: Verify GREEN for configuration tests**

Run:

```bash
cmake --build test/build -j2 --target test_config_cleanup test_ai_inference_mode_config
./test/build/bin/test_config_cleanup
./test/build/bin/test_ai_inference_mode_config
```

Expected: both executables exit 0; cleanup output reports the contract passed and the AI configuration output still reports `aiNpuCoreStart=1`.

---

### Task 3: Align current documentation and verify the repository

**Files:**
- Modify: `Xcar2.md`

**Interfaces:**
- Consumes: current configuration interface after Task 2.
- Produces: documentation and verification evidence consistent with the implementation.

- [ ] **Step 1: Remove the obsolete current-interface description**

Delete the `errorCalcBand` row from the `tc` common keys table. Preserve historical design and plan files.

- [ ] **Step 2: Validate both runtime JSON files**

Run:

```bash
jq empty configs/config.json configs/config_stable.json
```

Expected: exit 0 with no output.

- [ ] **Step 3: Scan current interfaces for every removed name**

Run `rg` over `configs/config.json`, `configs/config_stable.json`, `include`, `src`, `AI`, `Position`, `test`, `README.md`, `PROJECT_STATUS.md`, `Xcar2.md`, and `TC264.md`, excluding the cleanup regression test's string list. Expected: no production/config/current-doc matches; only the test's intentional legacy key strings and historical Superpowers documents may contain the names.

- [ ] **Step 4: Verify protected compatibility keys**

Run:

```bash
rg -n 'aiNpuCoreStart' configs/config.json configs/config_stable.json include/config.h src/io/config.cpp test/test_ai_inference_mode_config.cpp
rg -n 'pitch_deg|pitch_rad' configs/config.json configs/config_stable.json src/io/config.cpp include/camera_model.h
```

Expected: `aiNpuCoreStart` remains in the full load/save chain and test; camera pitch conversion remains intact.

- [ ] **Step 5: Build relevant control tests and the main target**

Run:

```bash
cmake --build test/build -j2 --target test_gold_slow_band test_vehicle_gold_source_driven_control test_ped_source_driven_control
./test/build/bin/test_gold_slow_band
./test/build/bin/test_vehicle_gold_source_driven_control
./test/build/bin/test_ped_source_driven_control
cmake --build build -j2 --target main
```

Expected: every command exits 0 and the build output ends with each requested target built.

- [ ] **Step 6: Check patch integrity**

Run:

```bash
git diff --check
git status --short
```

Expected: no whitespace errors; status retains unrelated pre-existing user changes and lists only the intended cleanup edits in the scoped files.
