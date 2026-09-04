# Pedestrian Near STOP Release Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 让近场行人在连续三个不同的新 AI 源帧满足坐标拉线条件后，从 `StopInTrack` 可靠转入绕行，并避免脚点赛道区域和停车锁继续强制 STOP。

**Architecture:** 保留现有行人 FSM 和 `tcPersonNearZoneDecide()`，只调整 `tcPedProcessFrame()` 内的判定顺序。近场坐标结果在停车锁和 `PedFootZone` 决策前处理；仅当状态已经是 `StopInTrack` 时累计释放确认，达到配置值后复用 `tcPedEnterDetour()`，近场以外继续走原分支。

**Tech Stack:** C++17、OpenCV、CMake、现有 Xcar2 控制状态机和独立测试程序。

## Global Constraints

- 近场条件必须严格为 `target->center_y > TC.personEmergNearYMax`。
- 近场只忽略 `PedFootZone` 的控制结果；`post-car` 保护继续拥有更高优先级。
- `personStopReleaseConfirm` 只控制 `StopInTrack -> DetourOutside`，最小有效值按 1 处理。
- 释放确认只由可推进状态的不同 `EVID NEW` 源帧累计；`REUSE/UNK` 不增加也不清零。
- 近场停车坐标、离开近场或近场拉线不再成立时，释放计数清零。
- 第三个释放源帧只进入绕行；UART `FAST` 仍需满足现有 `personDetourFastConfirm`。
- `center_y <= personEmergNearYMax` 时保留车辆在赛道内的脚点区域停车优先级。
- 不修改任何配置字段或配置值，不纳入用户现有的 `configs/config.json` 工作区改动。

---

### Task 1: 用源帧测试复现近场 STOP 无法释放

**Files:**
- Modify: `test/test_ped_source_driven_control.cpp`

**Interfaces:**
- Consumes: `PedHarness::run(objects, evidence_kind, source_fid, mid_offset)`；`mid_offset=0` 表示车辆位于配置的赛道内关系范围。
- Produces: 可指定行人中心 Y 的 `makeHuman(int center_x, int center_y = 140)`、固定车辆目标 `makeCar()`，以及近场释放、重复源帧、释放重置和 `post-car` 回归测试。

- [ ] **Step 1: 让测试夹具显式配置近场坐标和释放阈值**

把行人构造函数改为按中心坐标生成检测框：

```cpp
TrackedObject makeHuman(int center_x, int center_y = 140)
{
    TrackedObject human;
    human.class_id = HUMAN;
    human.score = 0.95f;
    human.box = cv::Rect(center_x - 15, center_y - 20, 30, 40);
    human.center_x = center_x;
    human.center_y = center_y;
    human.frame_id = 1;
    return human;
}

TrackedObject makeCar()
{
    TrackedObject car;
    car.class_id = CAR;
    car.score = 0.95f;
    car.box = cv::Rect(190, 130, 45, 40);
    car.center_x = car.box.x + car.box.width / 2;
    car.center_y = car.box.y + car.box.height / 2;
    car.frame_id = 1;
    return car;
}
```

在 `PedHarness` 中显式设置测试所依赖的参数，避免读取进程内其他测试配置：

```cpp
tc.personEmergNearYMax = 180;
tc.personEmergNearXMin = 100;
tc.personEmergNearXMax = 220;
tc.personCloseNearXMin = 150;
tc.personCloseNearXMax = 170;
tc.personStopReleaseConfirm = 3;
tc.personTrackWidthAdd = 45;
tc.personTrackWidthInward = 14;
tc.carFrontY = 150;
tc.carAvoidMinY = 120;
tc.carDetectMaxY = 220;
tc.carAvoidExitY = -1;
tc.carAvoidLostMax = 1;
tc.personPostCarPedDistM = 10.0f;
```

这样 `makeHuman(160, 200)` 是近场停车坐标，`makeHuman(130, 200)` 是近场左接近区拉线坐标，同时脚点稳定处于测试赛道内部区域。

- [ ] **Step 2: 增加近场释放失败回归测试**

增加以下测试，并在 `main()` 中分配新的非零返回码：

```cpp
bool testNearPullNeedsThreeNewSourcesInsideTrack()
{
    PedHarness harness(1);
    harness.run({makeHuman(160, 200)}, AiEvidenceKind::NewSource, 100);
    if (harness.mode() != 1) return false;

    const std::vector<TrackedObject> pull{makeHuman(130, 200)};
    harness.run(pull, AiEvidenceKind::NewSource, 101);
    if (harness.mode() != 1) return false;
    harness.run(pull, AiEvidenceKind::NewSource, 102);
    if (harness.mode() != 1) return false;
    harness.run(pull, AiEvidenceKind::NewSource, 103);
    return harness.mode() == 2;
}

bool testReusedNearPullDoesNotReleaseStop()
{
    PedHarness harness(1);
    harness.run({makeHuman(160, 200)}, AiEvidenceKind::NewSource, 200);
    const std::vector<TrackedObject> pull{makeHuman(130, 200)};
    harness.run(pull, AiEvidenceKind::NewSource, 201);
    for (int i = 0; i < 5; ++i)
        harness.run(pull, AiEvidenceKind::Reused, 201);
    if (harness.mode() != 1) return false;
    harness.run(pull, AiEvidenceKind::NewSource, 202);
    if (harness.mode() != 1) return false;
    harness.run(pull, AiEvidenceKind::NewSource, 203);
    return harness.mode() == 2;
}

bool testNearStopCoordinateResetsReleaseConfirmation()
{
    PedHarness harness(1);
    harness.run({makeHuman(160, 200)}, AiEvidenceKind::NewSource, 300);
    const std::vector<TrackedObject> pull{makeHuman(130, 200)};
    harness.run(pull, AiEvidenceKind::NewSource, 301);
    harness.run({makeHuman(160, 200)}, AiEvidenceKind::NewSource, 302);
    harness.run(pull, AiEvidenceKind::NewSource, 303);
    harness.run(pull, AiEvidenceKind::NewSource, 304);
    if (harness.mode() != 1) return false;
    harness.run(pull, AiEvidenceKind::NewSource, 305);
    return harness.mode() == 2;
}

bool testLeavingNearRegionResetsReleaseConfirmation()
{
    PedHarness harness(1);
    harness.run({makeHuman(160, 200)}, AiEvidenceKind::NewSource, 400);
    const std::vector<TrackedObject> near_pull{makeHuman(130, 200)};
    harness.run(near_pull, AiEvidenceKind::NewSource, 401);
    harness.run({makeHuman(130, 140)}, AiEvidenceKind::NewSource, 402);
    harness.run(near_pull, AiEvidenceKind::NewSource, 403);
    harness.run(near_pull, AiEvidenceKind::NewSource, 404);
    if (harness.mode() != 1) return false;
    harness.run(near_pull, AiEvidenceKind::NewSource, 405);
    return harness.mode() == 2;
}

bool testPostCarProtectionStillBlocksNearRelease()
{
    PedHarness harness(1);
    config().tc.personPostCarEnabled = true;

    harness.run({makeCar()}, AiEvidenceKind::NewSource, 500);
    harness.run({}, AiEvidenceKind::NewSource, 501);
    harness.run({}, AiEvidenceKind::NewSource, 502);

    harness.run({makeHuman(160, 200)}, AiEvidenceKind::NewSource, 503);
    const std::vector<TrackedObject> left_pull{makeHuman(130, 200)};
    harness.run(left_pull, AiEvidenceKind::NewSource, 504);
    harness.run(left_pull, AiEvidenceKind::NewSource, 505);
    harness.run(left_pull, AiEvidenceKind::NewSource, 506);
    if (harness.mode() != 1) return false;

    config().tc.personPostCarEnabled = false;
    harness.run(left_pull, AiEvidenceKind::NewSource, 507);
    harness.run(left_pull, AiEvidenceKind::NewSource, 508);
    if (harness.mode() != 1) return false;
    harness.run(left_pull, AiEvidenceKind::NewSource, 509);
    return harness.mode() == 2;
}
```

`testLeavingNearRegionResetsReleaseConfirmation()` 的远场观测 `center_y=140, center_x=130` 命中现有远场停车带，证明离开近场后恢复原决策并清零释放计数。`testPostCarProtectionStillBlocksNearRelease()` 先通过真实车辆进入/退出流程建立 post-car 窗口，不增加测试专用生产接口。

在 `main()` 的现有七项检查后追加：

```cpp
if (!testNearPullNeedsThreeNewSourcesInsideTrack()) {
    std::cerr << "near pull did not release STOP after three new sources\n";
    return 8;
}
if (!testReusedNearPullDoesNotReleaseStop()) {
    std::cerr << "reused near pull source advanced STOP release\n";
    return 9;
}
if (!testNearStopCoordinateResetsReleaseConfirmation()) {
    std::cerr << "near stop coordinate did not reset release confirmation\n";
    return 10;
}
if (!testLeavingNearRegionResetsReleaseConfirmation()) {
    std::cerr << "leaving near region did not reset release confirmation\n";
    return 11;
}
if (!testPostCarProtectionStillBlocksNearRelease()) {
    std::cerr << "post-car protection no longer blocked near STOP release\n";
    return 12;
}
```

- [ ] **Step 3: 构建并确认测试在旧实现上失败**

Run:

```bash
cmake -S test -B test/build
cmake --build test/build --target test_ped_source_driven_control -j$(nproc)
./test/build/bin/test_ped_source_driven_control
```

Expected: 构建成功，但新增的 `testNearPullNeedsThreeNewSourcesInsideTrack()` 返回失败；旧实现始终被停车锁或 `PERS car-in-track zone stop` 留在 STOP。

---

### Task 2: 接入近场三源帧 STOP 释放

**Files:**
- Modify: `src/control/drive_control.cpp`
- Test: `test/test_ped_source_driven_control.cpp`

**Interfaces:**
- Consumes: `tcPedEvalEmerg()` 的 `emerg_active/emerg_stop/emerg_bias/emerg_offset` 和现有 `g_ped_avoid_phase`。
- Produces: `StopInTrack` 近场释放路径；达到确认值时调用现有 `tcPedEnterDetour(int foot_px, int foot_py, int bias, int offset)`。

- [ ] **Step 1: 在停车锁前计算近场坐标结果**

在脚点区域分类和 streak 更新后，先执行现有 `tcPedEvalEmerg()`，并派生：

```cpp
const bool ped_near = target->center_y > TC.personEmergNearYMax;
const bool emerg_pull = emerg_active && !emerg_stop;
const bool emerg_wants_stop = emerg_active && emerg_stop;
```

把当前位于 `tcPedEvalEmerg()` 之前的 `g_person_stop_lock` 提前返回移动到近场 STOP 释放分支之后。非近场仍经过原停车锁分支，行为不变。

- [ ] **Step 2: 实现 StopInTrack 的近场释放确认**

在通用停车锁分支之前加入：

```cpp
if (g_ped_avoid_phase == PedAvoidPhase::StopInTrack && ped_near) {
    if (tcPedPostCarBlocksFastForFoot(foot_px, TC)) {
        tcPedEnterStop("PERS post-car left stop", true);
        return;
    }
    if (emerg_pull) {
        const int release_confirm = std::max(1, TC.personStopReleaseConfirm);
        if (g_ped_stop_release_streak < release_confirm)
            ++g_ped_stop_release_streak;
        if (g_ped_stop_release_streak >= release_confirm) {
            tcPedEnterDetour(foot_px, foot_py, emerg_bias, emerg_offset);
            tcPedEmitLockedLine(out_dodge_x, out_dodge_y);
        } else {
            tcPedEnterStop("PERS near release wait", false, false);
        }
        return;
    }

    g_ped_stop_release_streak = 0;
    tcPedEnterStop("PERS near coordinate stop", false, false);
    return;
}
```

近场决策函数当前对所有 X 返回停车或拉线，因此最后的 STOP 是防御性保持；它避免未来近场判定增加中性结果后意外解除已有 STOP。

- [ ] **Step 3: 让近场通用分支忽略 PedFootZone**

在近场 STOP 分支返回后、通用停车锁判断之前精确清零其他新源帧路径的释放计数：

```cpp
g_ped_stop_release_streak = 0;

if (g_person_stop_lock > 0 && !tcPedInDetourPhase()) {
    tcPedEnterStop("PERS stop lock", false, false);
    return;
}
```

然后把区域停车和车辆在赛道内的区域强制停车限定在非近场：

```cpp
const bool zone_stop = !ped_near && tcPedZoneConfirmedStop(zone);

if (emerg_pull) {
    if (!ped_near && car_track_relation_inside &&
        tcPedZoneRequiresStopWhenCarInside(zone)) {
        tcPedEnterStop("PERS car-in-track zone stop", true);
        return;
    }
    if (tcPedPostCarBlocksFastForFoot(foot_px, TC)) {
        tcPedEnterStop("PERS post-car left stop", true);
        return;
    }
    tcPedEnterDetour(foot_px, foot_py, emerg_bias, emerg_offset);
    tcPedEmitLockedLine(out_dodge_x, out_dodge_y);
    return;
}
```

不得在 `REUSE/UNK` 的入口保持返回前清零；它们必须保留当前释放进度。

- [ ] **Step 4: 删除冲突的未使用释放 helper**

删除无调用方的 `tcPedStopReleaseOk()`。该函数把橙区固定解释为不可释放，并且没有接入 FSM；保留它会与新的“近场只用坐标”规则形成两套相反语义。

Run:

```bash
rg -n "tcPedStopReleaseOk" src/control/drive_control.cpp
```

Expected: 无输出。

- [ ] **Step 5: 运行目标测试确认通过**

Run:

```bash
cmake --build test/build --target test_ped_source_driven_control -j$(nproc)
./test/build/bin/test_ped_source_driven_control
```

Expected: 输出 `pedestrian source-driven control tests passed`，退出码为 0；新增测试同时证明停车锁被确认释放、赛道内脚点不再覆盖近场坐标，以及复用源帧不计数。

- [ ] **Step 6: 提交最小补丁**

```bash
git add src/control/drive_control.cpp test/test_ped_source_driven_control.cpp
git commit -m "fix: release pedestrian stop from near coordinates"
```

提交前使用 `git diff --cached --name-only` 确认没有加入 `configs/config.json`。

---

### Task 3: 回归验证控制状态和主程序构建

**Files:**
- Verify only; no source changes expected.

**Interfaces:**
- Consumes: Task 2 提交后的行人 FSM。
- Produces: 行人、AI 源帧、车辆金币和主程序构建验证结果。

- [ ] **Step 1: 构建并运行无外部图片依赖的相关测试**

Run:

```bash
cmake --build test/build --target \
  test_ped_source_driven_control test_ai_control_evidence \
  test_ai_control_evidence_bridge test_vehicle_gold_source_driven_control \
  test_ai_detection_hold test_det_sync test_deleted_elements -j$(nproc)
./test/build/bin/test_ped_source_driven_control
./test/build/bin/test_ai_control_evidence
./test/build/bin/test_ai_control_evidence_bridge
./test/build/bin/test_vehicle_gold_source_driven_control
./test/build/bin/test_ai_detection_hold
./test/build/bin/test_det_sync
./test/build/bin/test_deleted_elements
```

Expected: 七个测试全部退出 0。`test_ped_car_conflict_patch` 依赖 `/home/orangepi/Pictures/human*.png`，本计划不把缺失外部图片导致的失败误判为代码回归。

- [ ] **Step 2: 构建 Release 主程序**

Run:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

Expected: 主程序构建成功，无新增编译或链接错误。

- [ ] **Step 3: 检查差异和工作区所有权**

Run:

```bash
git diff --check HEAD^ HEAD
git show --stat --oneline HEAD
git status --short
```

Expected: `git diff --check` 无输出；补丁提交只包含 `src/control/drive_control.cpp` 和 `test/test_ped_source_driven_control.cpp`；用户已有的 `M configs/config.json` 保留且未提交。

## Manual Video Acceptance

使用原录制视频逐帧回放，观察 `EVID`、`PED ph`、`sr` 和 UART 模式：

- 近场中心停车坐标保持 `PED ph=1` 和 `STOP`。
- 行人移动到近场拉线坐标后，不同 `EVID NEW` 依次显示 `sr=1/3`、`2/3`。
- 第三个连续新源帧后进入绕行阶段；脚点即使仍显示 `TRACK/ORG` 也不能重新压回 STOP。
- `EVID REUSE/UNK` 期间 `sr` 和模式保持不变。
- 坐标回到近场停车区时 `sr` 立即归零。
- 进入绕行后仍等待 `personDetourFastConfirm`，达到条件才显示或发送 `FAST`。
