# Low-Risk Code Cleanup Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Remove tracked build noise and configuration fields with no runtime consumers while preserving every current perception and control behavior.

**Architecture:** Treat the current two JSON files as the complete configuration contract. First add an executable contract test, then remove dead serialization paths and rename the gold lock radius end to end; only after source verification passes, remove generated directories from the Git index and validate from clean `/tmp` build trees.

**Tech Stack:** C++17, CMake, OpenCV, shell/Git, the existing hand-written JSON configuration loader.

## Global Constraints

- Support only `configs/config.json` and `configs/config2.json`; historical configuration compatibility is not required.
- Preserve the current `goldDistThresh` values exactly: `75` in `config.json` and `155` in `config2.json`.
- Preserve `std::max(24, configured value)` and the squared Euclidean gold target comparison.
- Do not change gold reachability, slow modes, error calculation, fallback selection, or lost-target behavior.
- Do not change SIGN, pedestrian, vehicle, fork, track, or UART state machines.
- Do not split `drive_control.cpp` or `imgprocess.cpp`.
- Do not delete models, test sources, test images, dormant source modules, or local build files.
- Remove generated build content from the Git index only; do not rewrite Git history.

## File Map

- Create `test/test_config_cleanup.cpp`: executable contract test for both current JSON files and `configSave()` output.
- Modify `test/CMakeLists.txt`: register `test_config_cleanup` with the same lightweight config-test dependencies used by other config tests.
- Modify `include/config.h`: remove dead members and rename the gold lock radius member.
- Modify `src/io/config.cpp`: remove dead load/save entries and rename the live gold key.
- Modify `src/control/drive_control.cpp`: consume the renamed gold lock radius member with unchanged matching math.
- Modify `test/test_gold_slow_band.cpp`: use the renamed member in existing gold setup.
- Modify `configs/config.json`: remove dead keys and rename the gold key while preserving value `75`.
- Modify `configs/config2.json`: remove dead keys and rename the gold key while preserving value `155`.
- Modify `.gitignore`: explicitly ignore all six tracked build trees.
- Remove tracked generated files under the six build trees from the Git index; local files remain untouched.

---

### Task 1: Clean the Current Configuration Contract

**Files:**
- Create: `test/test_config_cleanup.cpp`
- Modify: `test/CMakeLists.txt`
- Modify: `include/config.h`
- Modify: `src/io/config.cpp`
- Modify: `src/control/drive_control.cpp`
- Modify: `test/test_gold_slow_band.cpp`
- Modify: `configs/config.json`
- Modify: `configs/config2.json`

**Interfaces:**
- Consumes: `bool configLoad(const std::string&)`, `bool configSave(const std::string&)`, and `AppConfig& config()` from `include/config.h`.
- Produces: `TrackControlParams::goldLockMatchRadiusPx` as an `int`, loaded and saved under JSON key `goldLockMatchRadiusPx`, plus executable `test_config_cleanup` returning zero only when the current configuration contract is correct.

- [ ] **Step 1: Add the contract test**

Create `test/test_config_cleanup.cpp` with explicit return checks rather than `assert`, because the test project globally defines `NDEBUG`:

```cpp
#include "config.h"

#include <cstdio>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>

static std::string readFile(const std::string& path)
{
    std::ifstream in(path);
    return std::string(std::istreambuf_iterator<char>(in),
                       std::istreambuf_iterator<char>());
}

static bool excludesRemovedKeys(const std::string& json)
{
    static const char* const removed[] = {
        "blueHueLow", "blueHueHigh", "blueSatLow", "blueSatHigh",
        "blueValLow", "blueValHigh", "forkExitEntryNearSepPx",
        "personComplexBottomDeltaMin", "personComplexBottomDeltaMax",
        "personCarWaitDetourBias", "personComplexOrangeDodgeOffset",
        "personOutEmergDodgeOffset", "goldDistThresh"
    };
    for (const char* key : removed) {
        if (json.find(std::string("\"") + key + "\"") != std::string::npos)
            return false;
    }
    return true;
}

static bool loadAndCheck(const char* path, int expectedRadius)
{
    return configLoad(path) &&
           config().tc.goldLockMatchRadiusPx == expectedRadius &&
           excludesRemovedKeys(readFile(path));
}

int main()
{
    if (!loadAndCheck("configs/config.json", 75)) {
        std::cerr << "config.json cleanup contract failed\n";
        return 1;
    }
    if (!loadAndCheck("configs/config2.json", 155)) {
        std::cerr << "config2.json cleanup contract failed\n";
        return 2;
    }

    const char* savedPath = "/tmp/xcar2_config_cleanup_saved.json";
    if (!configSave(savedPath)) {
        std::cerr << "configSave failed\n";
        return 3;
    }
    const std::string saved = readFile(savedPath);
    std::remove(savedPath);
    if (saved.find("\"goldLockMatchRadiusPx\": 155") == std::string::npos ||
        !excludesRemovedKeys(saved)) {
        std::cerr << "saved config contains stale keys\n";
        return 4;
    }

    std::cout << "config cleanup contract passed\n";
    return 0;
}
```

- [ ] **Step 2: Register the test target**

Add this block beside `test_gold_band_visual_config` in `test/CMakeLists.txt`:

```cmake
add_executable(test_config_cleanup
    ${CMAKE_CURRENT_LIST_DIR}/test_config_cleanup.cpp
    ${ROOT_DIR}/src/io/config.cpp
    ${ROOT_DIR}/src/perception/camera_model.cpp
)
target_link_libraries(test_config_cleanup ${OpenCV_LIBS})
set_target_properties(test_config_cleanup PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
```

- [ ] **Step 3: Build the new target and verify the red state**

Run:

```bash
cmake -S test -B /tmp/xcar2-cleanup-test-red
cmake --build /tmp/xcar2-cleanup-test-red --target test_config_cleanup -j$(nproc)
```

Expected: compilation fails because `TrackControlParams` has no member named
`goldLockMatchRadiusPx`. This proves the test requires the rename rather than
passing against stale behavior.

- [ ] **Step 4: Reconfirm every removal candidate has no runtime consumer**

Run:

```bash
rg -n "blueHueLow|blueHueHigh|blueSatLow|blueSatHigh|blueValLow|blueValHigh|forkExitEntryNearSepPx|personComplexBottomDeltaMin|personComplexBottomDeltaMax|personCarWaitDetourBias|personComplexOrangeDodgeOffset|personOutEmergDodgeOffset" include src configs test -g '!build/**' -g '!build_sign/**' -g '!test/build/**' -g '!test/build_sign/**' -g '!test/shm_test/build/**'
```

Expected: matches occur only in `include/config.h`, `src/io/config.cpp`, the two
JSON files, and `test/test_config_cleanup.cpp`. If a business-logic source has
gained a reference, leave that field unchanged and update the design exception
before continuing.

- [ ] **Step 5: Update the configuration structures**

In `include/config.h`, delete the six `blue*` members, delete
`forkExitEntryNearSepPx`, and delete the five obsolete pedestrian members.
Replace the old gold declaration and misleading comment with:

```cpp
// 已锁定金币的跨帧脚点匹配半径(px)；实际下限为 24px。
int goldLockMatchRadiusPx = 30;
```

Keep all neighboring live fields in their existing order.

- [ ] **Step 6: Update configuration loading and saving**

In `src/io/config.cpp`, delete the load and `fprintf` lines for all twelve dead
fields. Replace the old gold load with:

```cpp
tc.goldLockMatchRadiusPx = jInt(tcSec, "goldLockMatchRadiusPx",
                                tc.goldLockMatchRadiusPx);
```

Replace the old gold save line with:

```cpp
fprintf(fp, "        \"goldLockMatchRadiusPx\": %d,\n",
        tc.goldLockMatchRadiusPx);
```

Preserve valid JSON commas by leaving the first remaining `img` and `tc`
entries comma-terminated exactly as their following entries require.

- [ ] **Step 7: Rename the runtime and test member references**

In `src/control/drive_control.cpp`, change only the member name:

```cpp
const int match_thresh = std::max(24, TC.goldLockMatchRadiusPx);
```

In `test/test_gold_slow_band.cpp`, change only the setup member name:

```cpp
config().tc.goldLockMatchRadiusPx = 80;
```

Do not modify the matching branches, fallback selection, `outside_ring`, or
lost-frame handling.

- [ ] **Step 8: Update both current JSON files**

In `configs/config.json`, replace:

```json
"goldDistThresh": 75
```

with:

```json
"goldLockMatchRadiusPx": 75
```

In `configs/config2.json`, make the same rename while preserving value `155`.
Delete all dead keys listed in Task 1 Step 4 wherever present.

- [ ] **Step 9: Build and run focused tests**

Run:

```bash
cmake --build /tmp/xcar2-cleanup-test-red --target test_config_cleanup test_gold_slow_band test_no_hsv_fallback -j$(nproc)
/tmp/xcar2-cleanup-test-red/bin/test_config_cleanup
/tmp/xcar2-cleanup-test-red/bin/test_gold_slow_band
/tmp/xcar2-cleanup-test-red/bin/test_no_hsv_fallback
```

Expected: all three exit with status `0`; the contract test prints
`config cleanup contract passed`.

- [ ] **Step 10: Verify stale names are gone and JSON is valid**

Run:

```bash
rg -n "blueHueLow|blueHueHigh|blueSatLow|blueSatHigh|blueValLow|blueValHigh|forkExitEntryNearSepPx|personComplexBottomDeltaMin|personComplexBottomDeltaMax|personCarWaitDetourBias|personComplexOrangeDodgeOffset|personOutEmergDodgeOffset|goldDistThresh" include src configs test/*.cpp
cmake --build /tmp/xcar2-cleanup-test-red --target test_config_cleanup -j$(nproc)
/tmp/xcar2-cleanup-test-red/bin/test_config_cleanup
git diff --check -- include/config.h src/io/config.cpp src/control/drive_control.cpp test/test_gold_slow_band.cpp test/test_config_cleanup.cpp test/CMakeLists.txt configs/config.json configs/config2.json
```

Expected: `rg` reports only the deliberate old-key strings inside
`test/test_config_cleanup.cpp`; build and test pass; `git diff --check` prints
nothing.

- [ ] **Step 11: Commit the source and configuration cleanup**

```bash
git add include/config.h src/io/config.cpp src/control/drive_control.cpp test/test_gold_slow_band.cpp test/test_config_cleanup.cpp test/CMakeLists.txt configs/config.json configs/config2.json
git commit -m "refactor: remove obsolete configuration fields"
```

### Task 2: Remove Generated Build Trees from Version Control

**Files:**
- Modify: `.gitignore`
- Git index only: `build/`, `build_sign/`, `test/build/`, `test/build_sign/`, `test/shm_test/build/`, `Position/slam_workspace/slam_all/build/`

**Interfaces:**
- Consumes: existing local build trees and Git index state.
- Produces: ignored local build output that no longer appears in future commits.

- [ ] **Step 1: Add explicit build-tree ignore rules**

Append this block to `.gitignore`:

```gitignore

# CMake build trees
/build/
/build_sign/
/test/build/
/test/build_sign/
/test/shm_test/build/
/Position/slam_workspace/slam_all/build/
```

- [ ] **Step 2: Verify the local build trees exist before index cleanup**

Run:

```bash
test -f build/bin/main
test -d test/build
git ls-files | awk '/(^|\/)(build|build_sign)(\/|$)/ {count++} END {print count+0}'
```

Expected: both `test` commands exit `0`; tracked generated-file count is
currently `712` unless another commit changed it.

- [ ] **Step 3: Remove generated content from the Git index only**

Run:

```bash
git rm -r --cached --ignore-unmatch build build_sign test/build test/build_sign test/shm_test/build Position/slam_workspace/slam_all/build
```

Expected: Git stages generated files as deletions. The files remain on disk
because `--cached` does not remove working-tree content.

- [ ] **Step 4: Verify ignore behavior and local-file preservation**

Run:

```bash
test -f build/bin/main
test -d test/build
git check-ignore -v build/bin/main test/build/bin/test_gold_slow_band test/shm_test/build/CMakeCache.txt Position/slam_workspace/slam_all/build/CMakeCache.txt
git ls-files | awk '/(^|\/)(build|build_sign)(\/|$)/ {print; found=1} END {exit found ? 1 : 0}'
```

Expected: local-file checks pass; `git check-ignore` shows the new rules; the
final command prints nothing and exits `0`.

- [ ] **Step 5: Commit repository hygiene separately**

```bash
git add .gitignore
git commit -m "chore: stop tracking build outputs"
```

Expected: the commit includes `.gitignore` and staged generated-file deletions,
but no source, model, test image, or local binary addition.

### Task 3: Clean Build and Regression Verification

**Files:**
- No source changes expected.

**Interfaces:**
- Consumes: cleaned source/configuration contract and ignored build trees.
- Produces: fresh verification evidence independent of repository build output.

- [ ] **Step 1: Configure fresh build trees outside the repository**

Run:

```bash
cmake -S . -B /tmp/xcar2-cleanup-main-build -DCMAKE_BUILD_TYPE=Release
cmake -S test -B /tmp/xcar2-cleanup-test-build -DCMAKE_BUILD_TYPE=Release
```

Expected: both configurations complete successfully and write no files under
the repository's ignored build trees.

- [ ] **Step 2: Build the main target and required regression targets**

Run:

```bash
cmake --build /tmp/xcar2-cleanup-main-build --target main -j$(nproc)
cmake --build /tmp/xcar2-cleanup-test-build --target test_config_cleanup test_no_hsv_fallback test_gold_slow_band test_gold_track_band test_gold_band_visual_config test_ped_track_widen test_ped_car_conflict_patch test_fork_entry_left test_fork_entry_width test_fork_exit_stable test_fork_run_batch -j$(nproc)
```

Expected: both builds finish with exit status `0`.

- [ ] **Step 3: Run the required regression executables from the repository root**

Run:

```bash
/tmp/xcar2-cleanup-test-build/bin/test_config_cleanup
/tmp/xcar2-cleanup-test-build/bin/test_no_hsv_fallback
/tmp/xcar2-cleanup-test-build/bin/test_gold_slow_band
/tmp/xcar2-cleanup-test-build/bin/test_gold_track_band
/tmp/xcar2-cleanup-test-build/bin/test_gold_band_visual_config
/tmp/xcar2-cleanup-test-build/bin/test_ped_track_widen
/tmp/xcar2-cleanup-test-build/bin/test_ped_car_conflict_patch
/tmp/xcar2-cleanup-test-build/bin/test_fork_entry_left
/tmp/xcar2-cleanup-test-build/bin/test_fork_entry_width
/tmp/xcar2-cleanup-test-build/bin/test_fork_exit_stable
/tmp/xcar2-cleanup-test-build/bin/test_fork_run_batch
```

Expected: every executable exits with status `0`.

- [ ] **Step 4: Run final repository checks**

Run:

```bash
git diff --check HEAD
git status --short
git ls-files | awk '/(^|\/)(build|build_sign)(\/|$)/ {print; found=1} END {exit found ? 1 : 0}'
rg -n "goldDistThresh" include src configs test/*.cpp
```

Expected: `git diff --check` prints nothing; the worktree is clean; no generated
build file is tracked; the only `goldDistThresh` occurrence is the deliberate
legacy-key rejection string in `test/test_config_cleanup.cpp`.

- [ ] **Step 5: Record verification without changing runtime documentation**

Use the final handoff message to report the exact main build result, the list
of regression executables run, the two cleanup commit hashes, and any test that
could not execute. Do not edit `PROJECT_STATUS.md` solely to record this
mechanical cleanup.
