# Position New Map Swap Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make `robot_core` use the new SDF map from `Position/slam_workspace/new_map`.

**Architecture:** Keep the existing runtime contract: `FusionEngine` loads `sdf_map.bin` from the current working directory. Update the hard-coded map dimensions and origin to match the new map, then replace the default SDF files used by source-tree and build-directory execution.

**Tech Stack:** C++17, CMake, Ceres, Eigen, Sophus, binary SDF float map files.

## Global Constraints

- Do not change the main Xcar control pipeline.
- Preserve the existing `robot_core` runtime path used by `/home/orangepi/Desktop/run_all.sh`.
- New map dimensions are `162 x 156`.
- New map resolution is `0.05`.
- New map origin is `[-1.58, -6.05, 0]`.
- Runtime SDF size must be `101088` bytes.

---

### Task 1: Update Position Map Parameters And Runtime SDF

**Files:**
- Modify: `/home/orangepi/Desktop/Xcar/Position/slam_workspace/slam_all/Function.hpp`
- Modify: `/home/orangepi/Desktop/Xcar/Position/slam_workspace/slam_all/ScanMatch.hpp`
- Modify: `/home/orangepi/Desktop/Xcar/Position/slam_workspace/sdf_map.bin`
- Modify: `/home/orangepi/Desktop/Xcar/Position/slam_workspace/slam_all/build/sdf_map.bin`

**Interfaces:**
- Consumes: `Position/slam_workspace/new_map/mapfn.yaml`, `Position/slam_workspace/new_map/sdf_map(2).bin`
- Produces: `robot_core` map constants and default SDF files matching the new map

- [ ] **Step 1: Verify the new map dimensions and SDF size**

Run:

```bash
sed -n '1,8p' /home/orangepi/Desktop/Xcar/Position/slam_workspace/new_map/mapfn.yaml
python3 - <<'PY'
from pathlib import Path
p = Path('/home/orangepi/Desktop/Xcar/Position/slam_workspace/new_map/sdf_map(2).bin')
print(p.stat().st_size)
print(162 * 156 * 4)
PY
```

Expected: YAML shows `resolution: 0.05` and `origin: [-1.58, -6.05, 0]`; both printed sizes are `101088`.

- [ ] **Step 2: Update compile-time constants**

In `Function.hpp`, set:

```cpp
constexpr int MAP_WIDTH = 162;
constexpr int MAP_HEIGHT = 156;
constexpr int SDF_MAP_SIZE = MAP_WIDTH * MAP_HEIGHT;
```

In `ScanMatch.hpp`, set:

```cpp
constexpr float MAP_RESOLUTION = 0.05f;
constexpr float MAP_ORIGIN_X = -1.58f;
constexpr float MAP_ORIGIN_Y = -6.05f;
```

- [ ] **Step 3: Replace default SDF files**

Run:

```bash
cp '/home/orangepi/Desktop/Xcar/Position/slam_workspace/new_map/sdf_map(2).bin' /home/orangepi/Desktop/Xcar/Position/slam_workspace/sdf_map.bin
cp '/home/orangepi/Desktop/Xcar/Position/slam_workspace/new_map/sdf_map(2).bin' /home/orangepi/Desktop/Xcar/Position/slam_workspace/slam_all/build/sdf_map.bin
```

Expected: both destination files are `101088` bytes.

- [ ] **Step 4: Build `robot_core`**

Run:

```bash
cmake --build /home/orangepi/Desktop/Xcar/Position/slam_workspace/slam_all/build
```

Expected: build exits with code 0.

- [ ] **Step 5: Inspect changed files**

Run:

```bash
git -C /home/orangepi/Desktop/Xcar diff -- Position/slam_workspace/slam_all/Function.hpp Position/slam_workspace/slam_all/ScanMatch.hpp
stat -c '%n %s' /home/orangepi/Desktop/Xcar/Position/slam_workspace/sdf_map.bin /home/orangepi/Desktop/Xcar/Position/slam_workspace/slam_all/build/sdf_map.bin
```

Expected: code diff only contains the map constants; both SDF files show `101088`.
