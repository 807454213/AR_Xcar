# Right Fork Perception Lifecycle Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Gate right-branch exit left-boundary repair behind a perception-confirmed right-fork journey while preserving main-road fork behavior.

**Architecture:** Add a private journey state machine to `imgprocess.cpp` that observes existing Right intent, confirms the entrance-to-right-turn handoff, persists through the branch, and alone authorizes right-branch left-exit repair. Refactor the existing left repair into a read-only plan/preflight plus apply path, then update its candidate streak on every PPSeg frame with two-frame normal and one-frame strong evidence.

**Tech Stack:** C++17, OpenCV 4.5.4, RKNN PPSeg, CMake, ffmpeg/ffprobe, existing Xcar perception test harness.

## Global Constraints

- Work from `/home/orangepi/Desktop/Xcar` on branch `Xcar2`.
- The approved spec is `docs/superpowers/specs/2026-07-29-right-fork-perception-lifecycle-design.md`.
- Do not modify `src/control/drive_control.cpp`, control priority, steering, speed policy, SIGN behavior, UART, or TC264 protocol.
- Do not modify `configs/config.json`; preserve the user's existing `debugOverlay: false` worktree change.
- No SIGN still uses the existing Left/straight default; a SIGN decision still has priority.
- `ForkScanBias::Right` starts observation but never directly authorizes left repair.
- Normal entry/exit evidence uses at most two valid frames; strong evidence may pass in one frame.
- Private safety caps are 360 Await frames, 180 Entering frames, 600 InBranch frames, and 1200 total journey frames.
- One or two invalid tracking frames preserve confirmed branch phase but clear exit candidacy; the third resets the journey.
- Positive pre-rolled PNG replay must keep at least 13 of 18 left repairs.
- Validate every frame, step 2 offsets 0/1, and step 3 offsets 0/1/2 for both supplied videos.
- Follow TDD: observe each new regression fail before changing production behavior.
- Stage and commit only files named by the active task.

---

## File Structure

- Modify `include/imgprocess.h`: expose only the read-only `RightForkJourneyPhase` enum/getter.
- Modify `src/perception/imgprocess.cpp`: own the lifecycle, entry/handoff observations, reset behavior, left-repair preflight, per-frame candidate update, and exit arbitration.
- Modify `test/CMakeLists.txt`: register two focused perception tests and one local video replay executable.
- Create `test/test_right_fork_lifecycle_gate.cpp`: deterministic static-fixture state, negative, interruption, and reset assertions.
- Modify `test/test_right_fork_exit_left_repair.cpp`: pre-roll a real right-entry/interior sequence before the 18 positive exit PNGs.
- Create `test/test_right_fork_lifecycle_video.cpp`: record/check the main-road baseline and run all video step/offset variants.
- Create `test/baselines/one_cycle_main_exit.csv`: generated before production edits from the current main-road behavior.
- Create `test/img/right_fork_lifecycle/right/*.png`: sparse right-entry/interior pre-roll fixtures extracted from `RightFork.mp4`.
- Create `test/img/right_fork_lifecycle/main/*.png`: sparse forced-Right main-road entrance/handoff fixtures extracted from `OneCycle.mp4`.

The external MP4 files remain local acceptance inputs; ordinary static regression tests use checked-in PNG fixtures.

---

### Task 1: Freeze Main-Road Behavior and Add the Failing Context Gate

**Files:**
- Modify: `test/CMakeLists.txt`
- Create: `test/test_right_fork_lifecycle_gate.cpp`
- Create: `test/test_right_fork_lifecycle_video.cpp`
- Create: `test/baselines/one_cycle_main_exit.csv` by running the recorder

**Interfaces:**
- Consumes:
  - `processFrame(const cv::Mat&) -> CenterLineResult`
  - `getForkExitRepairState() -> ForkExitRepairState`
  - existing fork reset/bias APIs
- Produces:
  - `test_right_fork_lifecycle_gate`
  - `test_right_fork_lifecycle_video --record-main <video> <csv>`
  - immutable baseline columns `step,offset,right,left,first_right,mid175_hash`

- [ ] **Step 1: Register the new executables**

Add beside the existing fork targets in `test/CMakeLists.txt`:

```cmake
add_xcar_test(test_right_fork_lifecycle_gate
              test_right_fork_lifecycle_gate.cpp)
add_xcar_test(test_right_fork_lifecycle_video
              test_right_fork_lifecycle_video.cpp)
```

- [ ] **Step 2: Write the failing static context-gate test**

Create `test/test_right_fork_lifecycle_gate.cpp`:

```cpp
#include "config.h"
#include "imgprocess.h"
#include "ppseg_infer.hpp"

#include <opencv2/opencv.hpp>

#include <cstdio>
#include <string>
#include <vector>

namespace {

std::string fixture(const char* name)
{
    return std::string(XCAR_PROJECT_ROOT) + "/test/img/" + name;
}

void resetWithRight()
{
    resetTrackRoadMode();
    resetForkPhaseHunt();
    resetForkEntryState();
    resetForkExitSlopeCalib();
    resetForkSideX();
    setForkScanBiasLocked(true);
    setForkScanBias(ForkScanBias::Right);
    setForkPhaseHunt(ForkPhaseHunt::Exit);
    imgprocess_set_sign_blocks_auto_fork(false);
}

int countLeftRepairs(const std::vector<const char*>& names)
{
    int count = 0;
    for (const char* name : names) {
        const cv::Mat frame = cv::imread(fixture(name));
        if (frame.empty()) {
            std::printf("[FAIL] cannot read %s\n", name);
            return -1000;
        }
        (void)processFrame(frame);
        const ForkExitRepairState repair = getForkExitRepairState();
        if (repair.active && repair.side == ForkExitRepairSide::Left)
            ++count;
    }
    return count;
}

} // namespace

int main()
{
    if (!configLoad(std::string(XCAR_PROJECT_ROOT) + "/configs/config.json")) {
        std::printf("[FAIL] config load\n");
        return 1;
    }
    config().img.ppsegMaskStabilize = false;
    if (config().img.usePpSegTrack && !ppsegTrackInit()) {
        std::printf("[FAIL] ppseg init\n");
        return 1;
    }

    const std::vector<const char*> entrance = {
        "shm_20260717_231328_258.png",
        "shm_20260717_231328_743.png",
        "shm_20260717_231329_611.png",
        "shm_20260717_231330_033.png",
        "shm_20260717_231331_809.png",
    };
    const std::vector<const char*> mainExitA = {
        "shm_20260717_212350_036.png",
        "shm_20260717_212355_801.png",
        "shm_20260717_212400_629.png",
        "shm_20260717_212405_692.png",
        "shm_20260717_212409_014.png",
        "shm_20260717_212413_500.png",
        "shm_20260717_212417_651.png",
    };
    const std::vector<const char*> mainExitB = {
        "shm_20260717_225200_814.png",
        "shm_20260717_225207_068.png",
        "shm_20260717_225210_412.png",
        "shm_20260717_225213_057.png",
        "shm_20260717_225235_374.png",
    };
    const std::vector<const char*> directExit = {
        "shm_20260729_142154_247.png",
        "shm_20260729_142201_807.png",
        "shm_20260729_142207_293.png",
        "shm_20260729_142210_820.png",
        "shm_20260729_142214_631.png",
        "shm_20260729_142217_583.png",
        "shm_20260729_142220_538.png",
        "shm_20260729_142224_814.png",
        "shm_20260729_142228_370.png",
        "shm_20260729_142231_503.png",
        "shm_20260729_142233_932.png",
        "shm_20260729_142236_992.png",
        "shm_20260729_142239_283.png",
        "shm_20260729_142247_646.png",
        "shm_20260729_142251_338.png",
        "shm_20260729_142257_286.png",
        "shm_20260729_142301_606.png",
        "shm_20260729_142308_295.png",
    };

    resetWithRight();
    const int entranceLeft = countLeftRepairs(entrance);
    resetWithRight();
    const int mainExitALeft = countLeftRepairs(mainExitA);
    resetWithRight();
    const int mainExitBLeft = countLeftRepairs(mainExitB);
    resetWithRight();
    const int directExitLeft = countLeftRepairs(directExit);

    std::printf(
        "entrance_left=%d main_a_left=%d main_b_left=%d direct_exit_left=%d\n",
        entranceLeft, mainExitALeft, mainExitBLeft, directExitLeft);
    if (entranceLeft != 0 || mainExitALeft != 0 ||
        mainExitBLeft != 0 || directExitLeft != 0)
        return 2;
    return 0;
}
```

- [ ] **Step 3: Write the baseline recorder**

Create `test/test_right_fork_lifecycle_video.cpp` with this initial behavior:

```cpp
#include "config.h"
#include "imgprocess.h"
#include "ppseg_infer.hpp"

#include <opencv2/opencv.hpp>

#include <cstdint>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

namespace {

struct Stats {
    int step = 1;
    int offset = 0;
    int right = 0;
    int left = 0;
    int firstRight = -1;
    int samples = 0;
    std::uint64_t midHash = 1469598103934665603ULL;
};

void resetMainRoad()
{
    resetTrackRoadMode();
    resetForkPhaseHunt();
    resetForkEntryState();
    resetForkExitSlopeCalib();
    resetForkSideX();
    setForkScanBiasLocked(false);
    setForkScanBias(ForkScanBias::None);
    imgprocess_set_sign_blocks_auto_fork(false);
}

Stats collectMainExit(const std::string& path, int step, int offset)
{
    cv::VideoCapture cap(path);
    Stats out;
    out.step = step;
    out.offset = offset;
    if (!cap.isOpened())
        return out;

    const double fps = cap.get(cv::CAP_PROP_FPS);
    resetMainRoad();
    cv::Mat frame;
    int index = 0;
    while (cap.read(frame)) {
        if (index % step != offset) {
            ++index;
            continue;
        }
        const CenterLineResult result = processFrame(frame);
        const double sec = fps > 0.0 ? (double)index / fps : 0.0;
        if (sec >= 6.6 && sec <= 9.8) {
            ++out.samples;
            const ForkExitRepairState repair = getForkExitRepairState();
            if (repair.active && repair.side == ForkExitRepairSide::Right) {
                if (out.firstRight < 0)
                    out.firstRight = index;
                ++out.right;
            }
            if (repair.active && repair.side == ForkExitRepairSide::Left)
                ++out.left;
            const int mid = result.boundary.mid.size() > 175
                ? result.boundary.mid[175] : -1;
            out.midHash ^= (std::uint64_t)(mid + 2);
            out.midHash *= 1099511628211ULL;
        }
        ++index;
    }
    return out;
}

std::vector<Stats> collectMatrix(const std::string& path)
{
    std::vector<Stats> rows;
    for (int step = 1; step <= 3; ++step)
        for (int offset = 0; offset < step; ++offset)
            rows.push_back(collectMainExit(path, step, offset));
    return rows;
}

bool writeCsv(const std::string& path, const std::vector<Stats>& rows)
{
    std::ofstream out(path);
    if (!out)
        return false;
    out << "step,offset,right,left,first_right,mid175_hash\n";
    for (const Stats& s : rows) {
        if (s.samples <= 0)
            return false;
        out << s.step << ',' << s.offset << ',' << s.right << ',' << s.left
            << ',' << s.firstRight << ',' << s.midHash << '\n';
    }
    return true;
}

} // namespace

int main(int argc, char** argv)
{
    if (argc != 4 || std::string(argv[1]) != "--record-main") {
        std::printf("usage: %s --record-main <OneCycle.mp4> <baseline.csv>\n",
                    argv[0]);
        return 1;
    }
    if (!configLoad(std::string(XCAR_PROJECT_ROOT) + "/configs/config.json"))
        return 1;
    config().img.ppsegMaskStabilize = false;
    if (config().img.usePpSegTrack && !ppsegTrackInit())
        return 1;
    const std::vector<Stats> rows = collectMatrix(argv[2]);
    if (rows.size() != 6 || !writeCsv(argv[3], rows))
        return 1;
    return 0;
}
```

- [ ] **Step 4: Build both executables**

Run:

```bash
cmake -S test -B test/build
cmake --build test/build -j$(nproc) \
  --target test_right_fork_lifecycle_gate test_right_fork_lifecycle_video
```

Expected: both targets build.

- [ ] **Step 5: Record the pre-change main-road baseline**

Run:

```bash
mkdir -p test/baselines
./test/build/bin/test_right_fork_lifecycle_video \
  --record-main /home/orangepi/Videos/OneCycle.mp4 \
  test/baselines/one_cycle_main_exit.csv
wc -l test/baselines/one_cycle_main_exit.csv
```

Expected: `7` lines: one header and six step/offset rows. Each row must have
`left=0`; stop if the current normal main-road replay already violates that
assumption.

- [ ] **Step 6: Run the context gate and verify RED**

Run:

```bash
./test/build/bin/test_right_fork_lifecycle_gate
```

Expected: exit `2`; current `9855d7a` behavior reports at least one left repair
in the entrance, forced-Right main-exit, or direct-exit sequence.

- [ ] **Step 7: Commit the baseline and failing regression**

```bash
git add test/CMakeLists.txt \
  test/test_right_fork_lifecycle_gate.cpp \
  test/test_right_fork_lifecycle_video.cpp \
  test/baselines/one_cycle_main_exit.csv
git commit -m "test: add right fork lifecycle gate"
```

---

### Task 2: Confirm the Right Entrance and Branch Handoff

**Files:**
- Modify: `include/imgprocess.h`
- Modify: `src/perception/imgprocess.cpp`
- Modify: `test/CMakeLists.txt`
- Create: `test/test_right_fork_entry_lifecycle.cpp`
- Modify: `test/test_right_fork_exit_left_repair.cpp`
- Create: `test/img/right_fork_lifecycle/right/*.png`
- Create: `test/img/right_fork_lifecycle/main/*.png`

**Interfaces:**
- Consumes:
  - existing `ForkEntryState`, `dualHint`, `stillForkWidthIn`,
    `entrySingleLaneNear`, and `classifyTrackShape()`
- Produces:
  - `enum class RightForkJourneyPhase`
  - `RightForkJourneyPhase getRightForkJourneyPhase()`
  - private `rightForkJourneyBeginFrame(...)`
  - private `rightForkJourneyRecordEntry(...)`
  - armed context only after a right-turn single-lane handoff

- [ ] **Step 1: Extract compact chronological fixtures**

Run:

```bash
mkdir -p test/img/right_fork_lifecycle/right
mkdir -p test/img/right_fork_lifecycle/main
ffmpeg -y -ss 2.4 -i /home/orangepi/Videos/RightFork.mp4 -t 7.0 \
  -vf fps=5 -compression_level 3 \
  test/img/right_fork_lifecycle/right/frame_%03d.png
ffmpeg -y -ss 21.5 -i /home/orangepi/Videos/OneCycle.mp4 -t 4.0 \
  -vf fps=5 -compression_level 3 \
  test/img/right_fork_lifecycle/main/frame_%03d.png
find test/img/right_fork_lifecycle/right -name '*.png' | wc -l
find test/img/right_fork_lifecycle/main -name '*.png' | wc -l
```

Expected: 35 right-journey frames and 20 main-road stress frames.

- [ ] **Step 2: Add the entry-lifecycle test before the API exists**

Register:

```cmake
add_xcar_test(test_right_fork_entry_lifecycle
              test_right_fork_entry_lifecycle.cpp)
```

Create `test/test_right_fork_entry_lifecycle.cpp`:

```cpp
#include "config.h"
#include "imgprocess.h"
#include "ppseg_infer.hpp"

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace {

std::vector<std::string> pngs(const char* relative)
{
    const fs::path dir = fs::path(XCAR_PROJECT_ROOT) / relative;
    std::vector<std::string> out;
    for (const fs::directory_entry& entry : fs::directory_iterator(dir))
        if (entry.path().extension() == ".png")
            out.push_back(entry.path().string());
    std::sort(out.begin(), out.end());
    return out;
}

void resetRight()
{
    resetTrackRoadMode();
    resetForkPhaseHunt();
    resetForkEntryState();
    resetForkExitSlopeCalib();
    resetForkSideX();
    setForkScanBiasLocked(true);
    setForkScanBias(ForkScanBias::Right);
    imgprocess_set_sign_blocks_auto_fork(false);
}

bool replay(const std::vector<std::string>& paths, bool expectBranch)
{
    resetRight();
    bool reachedBranch = false;
    int leftRepairs = 0;
    for (const std::string& path : paths) {
        const cv::Mat frame = cv::imread(path);
        if (frame.empty())
            return false;
        (void)processFrame(frame);
        reachedBranch = reachedBranch ||
            getRightForkJourneyPhase() == RightForkJourneyPhase::InRightBranch;
        const ForkExitRepairState repair = getForkExitRepairState();
        if (repair.active && repair.side == ForkExitRepairSide::Left)
            ++leftRepairs;
    }
    bool persistedAfterBiasClear = true;
    if (expectBranch && reachedBranch) {
        setForkScanBiasLocked(false);
        setForkScanBias(ForkScanBias::None);
        (void)processFrame(cv::imread(paths.back()));
        persistedAfterBiasClear =
            getRightForkJourneyPhase() ==
            RightForkJourneyPhase::InRightBranch;
    }
    std::printf(
        "expect_branch=%d reached=%d persisted=%d left=%d\n",
        expectBranch ? 1 : 0, reachedBranch ? 1 : 0,
        persistedAfterBiasClear ? 1 : 0, leftRepairs);
    return reachedBranch == expectBranch &&
           persistedAfterBiasClear && leftRepairs == 0;
}

} // namespace

int main()
{
    if (!configLoad(std::string(XCAR_PROJECT_ROOT) + "/configs/config.json"))
        return 1;
    config().img.ppsegMaskStabilize = false;
    if (config().img.usePpSegTrack && !ppsegTrackInit())
        return 1;

    const bool rightOk = replay(
        pngs("test/img/right_fork_lifecycle/right"), true);
    const bool mainOk = replay(
        pngs("test/img/right_fork_lifecycle/main"), false);
    return rightOk && mainOk ? 0 : 2;
}
```

Build:

```bash
cmake --build test/build -j$(nproc) \
  --target test_right_fork_entry_lifecycle
```

Expected: compile fails because `RightForkJourneyPhase` and its getter do not
exist.

- [ ] **Step 3: Add the read-only perception phase API**

Add after `ForkScanBias` in `include/imgprocess.h`:

```cpp
enum class RightForkJourneyPhase : int8_t {
    Idle = 0,
    AwaitRightEntry = 1,
    EnteringRight = 2,
    InRightBranch = 3,
    RightExitRepair = 4,
    Cooldown = 5,
};

RightForkJourneyPhase getRightForkJourneyPhase();
```

- [ ] **Step 4: Add the private lifecycle state and reset helpers**

Replace `g_right_fork_left_exit_candidate_cnt` with this state near the existing
fork globals in `src/perception/imgprocess.cpp`:

```cpp
struct RightForkJourneyState {
    RightForkJourneyPhase phase = RightForkJourneyPhase::Idle;
    int phaseFrames = 0;
    int journeyFrames = 0;
    int invalidFrames = 0;
    int entryEvidence = 0;
    int handoffEvidence = 0;
    int straightReject = 0;
    int exitCandidate = 0;
    int recoveryFrames = 0;
    int lastSplitY = -1;
    int maxSplitY = -1;
    bool sawDual = false;
    bool sawRightEntry = false;
    bool sawNearSplit = false;
    bool sawRightSelection = false;
};

static RightForkJourneyState g_right_fork_journey;

static void resetRightForkJourney()
{
    g_right_fork_journey = RightForkJourneyState();
}

RightForkJourneyPhase getRightForkJourneyPhase()
{
    return g_right_fork_journey.phase;
}

static bool rightForkJourneyArmed()
{
    return g_right_fork_journey.phase ==
               RightForkJourneyPhase::InRightBranch ||
           g_right_fork_journey.phase ==
               RightForkJourneyPhase::RightExitRepair;
}

static void setRightForkJourneyPhase(RightForkJourneyPhase phase)
{
    g_right_fork_journey.phase = phase;
    g_right_fork_journey.phaseFrames = 0;
    g_right_fork_journey.entryEvidence = 0;
    g_right_fork_journey.handoffEvidence = 0;
    g_right_fork_journey.straightReject = 0;
    g_right_fork_journey.exitCandidate = 0;
    g_right_fork_journey.recoveryFrames = 0;
}
```

In `resetForkPhaseHunt()`, replace the old candidate reset with:

```cpp
resetRightForkJourney();
```

- [ ] **Step 5: Add entry and handoff observations**

Add these private helpers after the entry probe helpers:

```cpp
static bool rightForkSelectedRightAtSplit(const TrackBoundary& bd, int splitY)
{
    for (int dy = 0; dy <= 4; ++dy) {
        const int y = splitY - dy;
        if (y < 0 || y >= (int)bd.rowSegments.size() ||
            y >= (int)bd.selectedLeft.size())
            continue;
        std::vector<std::pair<int, int>> valid;
        for (const auto& seg : bd.rowSegments[y])
            if (seg.second - seg.first + 1 >=
                std::max(3, config().img.forkScanMinSegW))
                valid.push_back(seg);
        if (valid.size() < 2 || bd.selectedLeft[y] < 0 ||
            bd.selectedRight[y] <= bd.selectedLeft[y])
            continue;
        const int leftMid = (valid.front().first + valid.front().second) / 2;
        const int rightMid = (valid.back().first + valid.back().second) / 2;
        const int selectedMid =
            (bd.selectedLeft[y] + bd.selectedRight[y]) / 2;
        return selectedMid >= (leftMid + rightMid) / 2;
    }
    return false;
}

static void rightForkJourneyCheckTimeout()
{
    const auto phase = g_right_fork_journey.phase;
    const int phaseLimit =
        phase == RightForkJourneyPhase::AwaitRightEntry ? 360 :
        phase == RightForkJourneyPhase::EnteringRight ? 180 :
        phase == RightForkJourneyPhase::InRightBranch ? 600 : 1200;
    if (g_right_fork_journey.phaseFrames > phaseLimit ||
        g_right_fork_journey.journeyFrames > 1200)
        resetRightForkJourney();
}

static void rightForkJourneyBeginFrame(bool valid, bool dualHint,
                                       bool stillForkWidth,
                                       bool singleLaneNear,
                                       bool rightTurnSingleLane)
{
    auto& s = g_right_fork_journey;
    if (!valid) {
        s.exitCandidate = 0;
        if (++s.invalidFrames >= 3)
            resetRightForkJourney();
        return;
    }
    s.invalidFrames = 0;

    if (s.phase == RightForkJourneyPhase::Idle) {
        if (getForkScanBias() == ForkScanBias::Right)
            setRightForkJourneyPhase(
                RightForkJourneyPhase::AwaitRightEntry);
        else
            return;
    }

    ++s.phaseFrames;
    ++s.journeyFrames;
    rightForkJourneyCheckTimeout();
    if (s.phase == RightForkJourneyPhase::Idle)
        return;

    const ForkScanBias bias = getForkScanBias();
    if ((s.phase == RightForkJourneyPhase::AwaitRightEntry ||
         s.phase == RightForkJourneyPhase::EnteringRight) &&
        bias == ForkScanBias::Left) {
        resetRightForkJourney();
        return;
    }
    if (s.phase == RightForkJourneyPhase::AwaitRightEntry &&
        bias == ForkScanBias::None && !s.sawRightEntry) {
        resetRightForkJourney();
        return;
    }

    if (dualHint)
        s.sawDual = true;

    if (s.phase == RightForkJourneyPhase::AwaitRightEntry &&
        s.sawDual && !dualHint && !stillForkWidth &&
        singleLaneNear && !s.sawRightEntry) {
        resetRightForkJourney();
        return;
    }

    if (s.phase == RightForkJourneyPhase::EnteringRight &&
        !dualHint && !stillForkWidth && singleLaneNear) {
        const bool strong =
            s.sawNearSplit && s.sawRightSelection &&
            rightTurnSingleLane;
        const bool normal = s.sawRightEntry && rightTurnSingleLane;
        if (strong || normal) {
            ++s.handoffEvidence;
            s.straightReject = 0;
            if (strong || s.handoffEvidence >= 2)
                setRightForkJourneyPhase(
                    RightForkJourneyPhase::InRightBranch);
        } else if (++s.straightReject >= 2) {
            resetRightForkJourney();
        }
    }
}

static void rightForkJourneyRecordEntry(const ForkEntryState& entry,
                                        const TrackBoundary& bd,
                                        int yBottom)
{
    auto& s = g_right_fork_journey;
    if ((s.phase != RightForkJourneyPhase::AwaitRightEntry &&
         s.phase != RightForkJourneyPhase::EnteringRight) ||
        !entry.active || entry.appliedBias != ForkScanBias::Right)
        return;

    s.sawDual = true;
    s.sawRightEntry = true;
    const int tolerance = std::max(1, config().img.forkEntryScanStepY);
    if (s.lastSplitY < 0 || entry.splitY + tolerance >= s.lastSplitY)
        ++s.entryEvidence;
    s.lastSplitY = entry.splitY;
    s.maxSplitY = std::max(s.maxSplitY, entry.splitY);

    const int nearY =
        yBottom - std::max(8, config().img.forkEntryHuntSwitchBottomPx);
    if (entry.splitY >= nearY) {
        s.sawNearSplit = true;
        s.sawRightSelection =
            rightForkSelectedRightAtSplit(bd, entry.splitY);
    }
    if ((s.sawNearSplit && s.sawRightSelection) ||
        s.entryEvidence >= 2)
        setRightForkJourneyPhase(RightForkJourneyPhase::EnteringRight);
}

static void rightForkJourneyRejectNewEntry(
    const ForkPhaseMetrics& fm, bool dualHint)
{
    if (g_right_fork_journey.phase !=
            RightForkJourneyPhase::InRightBranch ||
        !dualHint || !fm.hasEntryMask || fm.exitTrusted)
        return;
    if (fm.gapGrowPx >=
        std::max(0, config().img.forkEntryMinGapGrowPx))
        resetRightForkJourney();
}
```

- [ ] **Step 6: Wire observations into the PPSeg frame path**

In `trackFromBoundary`, compute one split hint and raw curve direction before
`trustedExit`:

```cpp
int splitYHint = -1;
int spanHint = 0;
dualHint = forkEntryQuickDualProbe(
    entryMask, bd, yTop2, yBottomEff, &splitYHint, &spanHint) || dualHint;

float rawVar = -1.0f;
float rawDelta = 0.0f;
classifyTrackShape(bd, yTop2, yBottomEff, &rawVar, &rawDelta);
int validBoundaryRows = 0;
for (int y = yTop2; y <= yBottomEff; ++y)
    if (y < (int)bd.left.size() && bd.left[y] >= 0 &&
        bd.right[y] > bd.left[y])
        ++validBoundaryRows;
const bool journeyValid = validBoundaryRows >= P.minValidRows;
const bool rightTurnSingleLane =
    entrySingleLaneNear && !dualHint && !stillForkWidthIn &&
    rawDelta >= std::max(0.0f, P.trackMidDirDeltaThresh);
rightForkJourneyBeginFrame(journeyValid, dualHint, stillForkWidthIn,
                           entrySingleLaneNear, rightTurnSingleLane);
rightForkJourneyRejectNewEntry(fm, dualHint);
```

Reuse `splitYHint` later instead of redeclaring it. After all entry-repair and
patch-hold branches, before exit repair, record the result:

```cpp
if (entryRepaired)
    rightForkJourneyRecordEntry(g_fork_entry, bd, yBottomEff);
```

Replace the broad context helpers:

```cpp
static bool forkExitRightBranchContext()
{
    return rightForkJourneyArmed();
}
```

In `forkExitHasRuntimeContext()`, replace raw Right-bias context with:

```cpp
const ForkScanBias bias = getForkScanBias();
return g_fork_phase_hunt == ForkPhaseHunt::Exit ||
       g_fork_exit_repair.active ||
       g_fork_entry.active ||
       g_fork_entry_patch_hold.has ||
       bias == ForkScanBias::Left ||
       rightForkJourneyArmed();
```

Replace the locked-Right exception:

```cpp
const bool lockedRightExitPull =
    lockedBiasPull &&
    getForkScanBias() == ForkScanBias::Right &&
    rightForkJourneyArmed();
```

- [ ] **Step 7: Pre-roll the existing positive exit test**

In `test/test_right_fork_exit_left_repair.cpp`, make reset start in entry hunt:

```cpp
setForkPhaseHunt(ForkPhaseHunt::Entry);
```

Add these path helpers in the test namespace:

```cpp
std::string fixture(const char* name)
{
    return std::string(XCAR_PROJECT_ROOT) + "/test/img/" + name;
}

std::vector<std::string> sortedPngs(const char* relative)
{
    const std::filesystem::path dir =
        std::filesystem::path(XCAR_PROJECT_ROOT) / relative;
    std::vector<std::string> paths;
    for (const auto& entry : std::filesystem::directory_iterator(dir))
        if (entry.path().extension() == ".png")
            paths.push_back(entry.path().string());
    std::sort(paths.begin(), paths.end());
    return paths;
}
```

Before the 18 exit frames, replay the sorted right lifecycle fixture directory:

```cpp
const std::vector<std::string> preRoll =
    sortedPngs("test/img/right_fork_lifecycle/right");
for (const std::string& path : preRoll) {
    const cv::Mat frame = cv::imread(path);
    if (frame.empty())
        return 1;
    (void)processFrame(frame);
    const ForkExitRepairState repair = getForkExitRepairState();
    if (repair.active && repair.side == ForkExitRepairSide::Left)
        return 2;
}
if (getRightForkJourneyPhase() !=
    RightForkJourneyPhase::InRightBranch)
    return 2;
```

Add `<filesystem>` and replace absolute exit paths with
`std::string(XCAR_PROJECT_ROOT) + "/test/img/..."`.

- [ ] **Step 8: Build and verify the entry lifecycle and gate**

Run:

```bash
cmake -S test -B test/build
cmake --build test/build -j$(nproc) --target \
  test_right_fork_entry_lifecycle \
  test_right_fork_lifecycle_gate \
  test_right_fork_exit_left_repair
./test/build/bin/test_right_fork_entry_lifecycle
./test/build/bin/test_right_fork_lifecycle_gate
./test/build/bin/test_right_fork_exit_left_repair
```

Expected:

- right fixtures reach `InRightBranch`;
- forced-Right main fixtures never reach `InRightBranch`;
- all three no-history negative sequences report zero left repairs;
- pre-rolled positive replay still reports at least 13 left repairs.

- [ ] **Step 9: Confirm the main-road baseline is unchanged**

Extend the video tool with CSV loading in Task 4; until then, regenerate to
`/tmp/one_cycle_after_entry.csv` and compare exactly:

```bash
./test/build/bin/test_right_fork_lifecycle_video \
  --record-main /home/orangepi/Videos/OneCycle.mp4 \
  /tmp/one_cycle_after_entry.csv
cmp test/baselines/one_cycle_main_exit.csv /tmp/one_cycle_after_entry.csv
```

Expected: `cmp` exits `0`.

- [ ] **Step 10: Commit the journey lifecycle**

```bash
git add include/imgprocess.h src/perception/imgprocess.cpp \
  test/CMakeLists.txt \
  test/test_right_fork_entry_lifecycle.cpp \
  test/test_right_fork_exit_left_repair.cpp \
  test/img/right_fork_lifecycle
git commit -m "fix: confirm right fork journey in perception"
```

---

### Task 3: Make Exit Candidacy Continuous, Fast, and Non-Mutating

**Files:**
- Modify: `src/perception/imgprocess.cpp`
- Modify: `test/test_right_fork_exit_left_repair.cpp`
- Modify: `test/test_right_fork_lifecycle_gate.cpp`

**Interfaces:**
- Consumes:
  - armed `RightForkJourneyPhase::InRightBranch`
  - `ForkPhaseMetrics`
  - existing left merge probe and line-fitting helpers
- Produces:
  - private `ForkExitLeftRepairPlan`
  - private `buildForkExitLeftRepairPlan(...) -> bool`
  - private `RightForkExitEvidence`
  - private `rightForkUpdateExitCandidate(...) -> bool`
  - two-frame normal and one-frame strong trigger

- [ ] **Step 1: Add failing readiness and interruption assertions**

In `test/test_right_fork_exit_left_repair.cpp`, factor the fixture pre-roll into:

```cpp
bool preRollRightBranch()
{
    resetForkStateForRightExit();
    const auto paths = sortedPngs(
        "test/img/right_fork_lifecycle/right");
    for (const std::string& path : paths) {
        const cv::Mat frame = cv::imread(path);
        if (frame.empty())
            return false;
        (void)processFrame(frame);
    }
    return getRightForkJourneyPhase() ==
           RightForkJourneyPhase::InRightBranch;
}
```

Add these tests before the full 18-frame replay:

```cpp
if (!preRollRightBranch())
    return 2;
(void)processFrame(cv::imread(fixture("shm_20260729_142201_807.png")));
const bool firstNormalRepair =
    getForkExitRepairState().active;
(void)processFrame(cv::imread(fixture("shm_20260729_142207_293.png")));
const bool secondNormalRepair =
    getForkExitRepairState().active &&
    getForkExitRepairState().side == ForkExitRepairSide::Left;
if (firstNormalRepair || !secondNormalRepair) {
    std::printf("[FAIL] normal exit evidence did not trigger on frame two\n");
    return 2;
}

if (!preRollRightBranch())
    return 2;
(void)processFrame(cv::imread(fixture("shm_20260729_142231_503.png")));
if (!getForkExitRepairState().active ||
    getForkExitRepairState().side != ForkExitRepairSide::Left) {
    std::printf("[FAIL] deep exit evidence did not trigger in one frame\n");
    return 2;
}

if (!preRollRightBranch())
    return 2;
(void)processFrame(cv::imread(fixture("shm_20260729_142201_807.png")));
const auto branchFrames = sortedPngs(
    "test/img/right_fork_lifecycle/right");
(void)processFrame(cv::imread(
    branchFrames[branchFrames.size() / 2]));
(void)processFrame(cv::imread(fixture("shm_20260729_142207_293.png")));
if (getForkExitRepairState().active) {
    std::printf("[FAIL] interrupted candidates accumulated\n");
    return 2;
}
```

Run:

```bash
cmake --build test/build -j$(nproc) \
  --target test_right_fork_exit_left_repair
./test/build/bin/test_right_fork_exit_left_repair
```

Expected: FAIL because the old readiness gate requires
`max(2, roadSmEnterFrames) == 5` candidates.

- [ ] **Step 2: Extract a read-only left-repair plan**

Add before `repairForkExitLeftMergeBoundary()`:

```cpp
struct ForkExitLeftRepairPlan {
    int mergeY = -1;
    int nearY = -1;
    int tipY = -1;
    int lineStartY = -1;
    int minTrackWidth = 0;
    float slope = 0.0f;
    float intercept = 0.0f;
};

static bool buildForkExitLeftRepairPlan(const Mat& trackMask,
                                        const TrackBoundary& bd,
                                        int yTop, int yBottom, int imgW,
                                        ForkExitLeftRepairPlan& plan)
{
    plan = ForkExitLeftRepairPlan();
    const auto& P = config().img;
    if (!P.forkExitRepairEnabled || imgW <= 0)
        return false;

    if (!forkExitLeftMergeProbe(bd, yTop, yBottom,
                                &plan.mergeY, &plan.nearY))
        return false;

    int tipLeftX = -1;
    plan.tipY = plan.mergeY;
    if (!trackMask.empty())
        forkMaskVTipProbe(trackMask, yTop, yBottom,
                          &plan.tipY, &tipLeftX);

    const int step = std::max(1, P.forkExitScanStepY);
    const int slopeN = std::max(2, P.forkExitSlopeRows);
    plan.minTrackWidth = std::max(4, P.forkExitMinTrackWidth);
    const int downRows =
        std::max(0, P.forkExitLineStartDownRows) +
        std::max(0, P.forkExitLeftLineStartExtraDownRows);
    plan.lineStartY =
        clampInt(plan.nearY + downRows, yTop, yBottom);

    vector<int> fitY;
    vector<int> fitX;
    if (!collectExitSlopeFitBelow(bd, plan.lineStartY, yBottom,
                                  step, slopeN, true, fitY, fitX) ||
        !fitBoundaryLine(fitY, fitX, plan.slope, plan.intercept))
        return false;

    int anchorX = (int)std::lround(
        plan.slope * (float)plan.lineStartY + plan.intercept);
    if (plan.lineStartY >= 0 &&
        plan.lineStartY < (int)bd.left.size() &&
        bd.left[plan.lineStartY] >= 0)
        anchorX = bd.left[plan.lineStartY];
    anchorX = clampInt(anchorX, 0, imgW - 1);

    const int edgeGuard = std::max(14, imgW / 20);
    if (anchorX <= edgeGuard || anchorX >= imgW - 1 - edgeGuard)
        return false;
    if (plan.lineStartY >= (int)bd.right.size() ||
        bd.right[plan.lineStartY] < anchorX + plan.minTrackWidth)
        return false;

    const int minSegW = std::max(3, P.forkScanMinSegW);
    const int maxSingleSegW = std::max(90, minSegW * 30);
    const int yScanTop = 2;
    const int topAnchorX = forkExitTopAnchorX(
        bd, yScanTop, plan.mergeY, plan.nearY, anchorX,
        minSegW, maxSingleSegW, true);
    vector<int> topY;
    vector<int> topX;
    if (collectExitTopStableEdge(
            bd, yScanTop, plan.lineStartY, plan.mergeY, topAnchorX,
            std::max(2, P.forkExitTopStableRows),
            std::max(1, P.forkExitTopStableMaxDx),
            std::max(4, P.forkExitTopAnchorBandPx),
            true, topY, topX))
        (void)fitExitLineTopPointToAnchor(
            topY, topX, plan.lineStartY, anchorX,
            plan.slope, plan.intercept);

    return true;
}
```

Replace `repairForkExitLeftMergeBoundary()` with:

```cpp
bool repairForkExitLeftMergeBoundary(const Mat& trackMask,
                                     TrackBoundary& bd,
                                     int yTop, int yBottom, int imgW)
{
    g_fork_exit_repair = ForkExitRepairState();
    ForkExitLeftRepairPlan plan;
    if (!buildForkExitLeftRepairPlan(
            trackMask, bd, yTop, yBottom, imgW, plan))
        return false;

    const int repairEndY = plan.lineStartY;
    const int minW = plan.minTrackWidth;
    const int blendBelow = std::min(
        3, std::max(0, config().img.forkExitLineStartDownRows));
    int repaired = 0;
    auto applyLeftAtY = [&](int y, int lNew) {
        if (y < 0 || y >= (int)bd.left.size())
            return;
        const int r = bd.right[y];
        if (r < 0)
            return;
        lNew = clampInt(lNew, 0, r - minW);
        if (bd.left[y] < 0 || bd.left[y] != lNew) {
            bd.left[y] = lNew;
            ++repaired;
        }
        bd.mid[y] = (lNew + r) >> 1;
        if (bd.selectedLeft[y] >= 0)
            bd.selectedLeft[y] = lNew;
    };

    for (int y = yTop; y <= repairEndY; ++y) {
        const int lLine = (int)std::lround(
            plan.slope * (float)y + plan.intercept);
        applyLeftAtY(y, lLine);
    }
    for (int blendRow = 1; blendRow <= blendBelow; ++blendRow) {
        const int y = repairEndY + blendRow;
        if (y > yBottom)
            break;
        const int lRaw = bd.left[y];
        if (lRaw < 0)
            continue;
        const int lLine = (int)std::lround(
            plan.slope * (float)y + plan.intercept);
        const float t =
            (float)blendRow / (float)(blendBelow + 1);
        const int lMix = (int)std::lround(
            (1.0f - t) * (float)lLine + t * (float)lRaw);
        applyLeftAtY(y, lMix);
    }

    g_fork_exit_repair.active = plan.mergeY >= 0;
    g_fork_exit_repair.side = ForkExitRepairSide::Left;
    g_fork_exit_repair.mergeY = plan.mergeY;
    g_fork_exit_repair.anchorY = plan.lineStartY;
    g_fork_exit_repair.tipY = plan.tipY;
    g_fork_exit_repair.slope = plan.slope;
    g_fork_exit_repair.intercept = plan.intercept;
    g_fork_exit_repair.repairedRows = repaired;
    return g_fork_exit_repair.active;
}
```

- [ ] **Step 3: Classify normal and strong exit evidence**

Add near the journey helpers:

```cpp
enum class RightForkExitEvidence {
    None = 0,
    Normal = 1,
    Strong = 2,
};

static RightForkExitEvidence classifyRightForkExitEvidence(
    const ForkPhaseMetrics& fm, bool repairFeasible)
{
    if (!rightForkJourneyArmed() || !repairFeasible ||
        !fm.hasExitBoundary || !fm.exitTrusted ||
        !fm.exitIsLeftJump)
        return RightForkExitEvidence::None;

    const auto& P = config().img;
    const int leftJump = std::max(8, P.forkExitLeftJumpDx);
    const int rightStable = std::max(4, P.forkExitRightMaxDx);
    const int leftDx = std::abs(fm.exitJumpDL);
    const int rightDx = std::abs(fm.exitJumpDR);
    const int trustedY =
        std::max(0, P.forkExitMinMergeY) +
        std::max(0, P.forkExitLeftTrustedPadPx);
    if (fm.exitMergeY < trustedY ||
        leftDx < leftJump ||
        rightDx > rightStable ||
        leftDx <= rightDx + 4)
        return RightForkExitEvidence::None;

    const int deepPad =
        std::max(8, P.forkExitTrustedPadPx + 4);
    if (fm.exitMergeY >= trustedY + deepPad &&
        leftDx > rightDx + 8)
        return RightForkExitEvidence::Strong;
    return RightForkExitEvidence::Normal;
}

static bool rightForkUpdateExitCandidate(RightForkExitEvidence evidence)
{
    auto& s = g_right_fork_journey;
    if (!rightForkJourneyArmed() ||
        evidence == RightForkExitEvidence::None) {
        s.exitCandidate = 0;
        return false;
    }
    if (s.phase == RightForkJourneyPhase::RightExitRepair) {
        s.exitCandidate = 2;
        return true;
    }
    if (evidence == RightForkExitEvidence::Strong) {
        s.exitCandidate = 2;
        return true;
    }
    s.exitCandidate = std::min(2, s.exitCandidate + 1);
    return s.exitCandidate >= 2;
}

static void rightForkRecordExitResult(bool attempted, bool repaired)
{
    auto& s = g_right_fork_journey;
    if (!attempted)
        return;
    if (repaired) {
        setRightForkJourneyPhase(
            RightForkJourneyPhase::RightExitRepair);
    } else {
        s.exitCandidate = 0;
        if (s.phase == RightForkJourneyPhase::RightExitRepair)
            setRightForkJourneyPhase(
                RightForkJourneyPhase::InRightBranch);
    }
}
```

- [ ] **Step 4: Update the candidate exactly once per PPSeg frame**

Immediately after `ForkPhaseMetrics fm` is computed and after the journey
begin-frame observation, add:

```cpp
ForkExitLeftRepairPlan rightExitPlan;
const bool rightExitFeasible =
    rightForkJourneyArmed() &&
    buildForkExitLeftRepairPlan(
        entryMask, bd, yTop2, yBottomEff, width, rightExitPlan);
const RightForkExitEvidence rightExitEvidence =
    classifyRightForkExitEvidence(fm, rightExitFeasible);
const bool rightBranchLeftReady =
    rightForkUpdateExitCandidate(rightExitEvidence);
```

Remove `forkExitRightBranchLeftRepairReady()` and its old counter. Permit a ready
armed candidate to force exit arbitration:

```cpp
if (rightBranchLeftReady) {
    g_fork_phase_hunt = ForkPhaseHunt::Exit;
    g_fork_exit_hunt_clear_cnt = 0;
    phase = TrackRoadMode::ForkExit;
}

doExit =
    (((phase == TrackRoadMode::ForkExit) && prevOrRawForkExit(raw)) ||
     rightBranchLeftReady) &&
    (!lockedBiasPull || lockedRightExitPull);
```

In the repair block, use:

```cpp
const bool preferRightBranchLeftExit =
    forkExitRightBranchContext();
if (preferRightBranchLeftExit && !rightBranchLeftReady) {
    g_fork_exit_repair = ForkExitRepairState();
} else if (fm.exitIsLeftJump || preferRightBranchLeftExit) {
    exitRepaired = repairForkExitLeftMergeBoundary(
        entryMask, bd, yTop2, yBottomEff, width);
} else {
    exitRepaired = repairForkExitMergeBoundary(
        bd, yTop2, yBottomEff, width);
}
rightForkRecordExitResult(
    preferRightBranchLeftExit && rightBranchLeftReady,
    exitRepaired);
```

Do not call `rightForkUpdateExitCandidate()` anywhere else.

- [ ] **Step 5: Build and verify GREEN**

Run:

```bash
cmake --build test/build -j$(nproc) --target \
  test_right_fork_exit_left_repair \
  test_right_fork_lifecycle_gate \
  test_fork_exit_stable
./test/build/bin/test_right_fork_exit_left_repair
./test/build/bin/test_right_fork_lifecycle_gate
./test/build/bin/test_fork_exit_stable \
  test/img/shm_20260729_142231_503.png /tmp/right-exit-plan.png
```

Expected:

- first normal repair occurs on the second candidate;
- `shm_20260729_142231_503.png` repairs from one strong candidate;
- interrupted candidates do not combine;
- full positive replay has at least 13 left repairs;
- all no-history negatives have zero left repairs;
- anchor remains outside the 14-pixel/image-width guard.

- [ ] **Step 6: Recheck the main-road golden baseline**

```bash
./test/build/bin/test_right_fork_lifecycle_video \
  --record-main /home/orangepi/Videos/OneCycle.mp4 \
  /tmp/one_cycle_after_exit.csv
cmp test/baselines/one_cycle_main_exit.csv /tmp/one_cycle_after_exit.csv
```

Expected: `cmp` exits `0`.

- [ ] **Step 7: Commit the exit gate**

```bash
git add src/perception/imgprocess.cpp \
  test/test_right_fork_exit_left_repair.cpp \
  test/test_right_fork_lifecycle_gate.cpp
git commit -m "fix: gate right fork exit by journey state"
```

---

### Task 4: Validate Two-to-Three-Times Speed With Every Sampling Offset

**Files:**
- Modify: `test/test_right_fork_lifecycle_video.cpp`

**Interfaces:**
- Consumes:
  - `RightForkJourneyPhase` getter
  - baseline CSV from Task 1
  - `/home/orangepi/Videos/RightFork.mp4`
  - `/home/orangepi/Videos/OneCycle.mp4`
- Produces:
  - `--check <RightFork.mp4> <OneCycle.mp4> <baseline.csv>`
  - six right-fork variants, six normal-main variants, and six forced-Right
    entrance/handoff variants

- [ ] **Step 1: Add CSV loading and exact baseline comparison**

Add:

```cpp
bool sameStats(const Stats& a, const Stats& b)
{
    return a.step == b.step &&
           a.offset == b.offset &&
           a.right == b.right &&
           a.left == b.left &&
           a.firstRight == b.firstRight &&
           a.midHash == b.midHash;
}

std::vector<Stats> readCsv(const std::string& path)
{
    std::ifstream in(path);
    std::vector<Stats> rows;
    std::string line;
    std::getline(in, line);
    while (std::getline(in, line)) {
        Stats s;
        unsigned long long hash = 0;
        if (std::sscanf(line.c_str(), "%d,%d,%d,%d,%d,%llu",
                        &s.step, &s.offset, &s.right, &s.left,
                        &s.firstRight, &hash) == 6) {
            s.midHash = (std::uint64_t)hash;
            rows.push_back(s);
        }
    }
    return rows;
}
```

- [ ] **Step 2: Add chronological lifecycle acceptance collection**

Add:

```cpp
struct JourneyStats {
    bool reachedBranch = false;
    bool branchAtMainExit = false;
    int entranceLeft = 0;
    int exitLeft = 0;
};

JourneyStats collectJourney(const std::string& path,
                            int step, int offset,
                            bool forceRight,
                            double entranceBegin,
                            double entranceEnd,
                            double exitBegin,
                            double exitEnd)
{
    cv::VideoCapture cap(path);
    JourneyStats out;
    if (!cap.isOpened())
        return out;
    const double fps = cap.get(cv::CAP_PROP_FPS);
    resetMainRoad();
    if (forceRight) {
        setForkScanBiasLocked(true);
        setForkScanBias(ForkScanBias::Right);
    }

    cv::Mat frame;
    int index = 0;
    while (cap.read(frame)) {
        if (index % step != offset) {
            ++index;
            continue;
        }
        (void)processFrame(frame);
        const double sec = fps > 0.0 ? (double)index / fps : 0.0;
        const RightForkJourneyPhase journey =
            getRightForkJourneyPhase();
        if (journey == RightForkJourneyPhase::InRightBranch ||
            journey == RightForkJourneyPhase::RightExitRepair)
            out.reachedBranch = true;
        const ForkExitRepairState repair = getForkExitRepairState();
        const bool left = repair.active &&
            repair.side == ForkExitRepairSide::Left;
        if (sec >= entranceBegin && sec <= entranceEnd && left)
            ++out.entranceLeft;
        if (sec >= exitBegin && sec <= exitEnd) {
            if (left)
                ++out.exitLeft;
            if (journey == RightForkJourneyPhase::InRightBranch ||
                journey == RightForkJourneyPhase::RightExitRepair)
                out.branchAtMainExit = true;
        }
        ++index;
    }
    return out;
}
```

- [ ] **Step 3: Add the complete check mode**

Replace `main()` with:

```cpp
int main(int argc, char** argv)
{
    if (!configLoad(std::string(XCAR_PROJECT_ROOT) + "/configs/config.json"))
        return 1;
    config().img.ppsegMaskStabilize = false;
    if (config().img.usePpSegTrack && !ppsegTrackInit())
        return 1;

    if (argc == 4 && std::string(argv[1]) == "--record-main") {
        const auto rows = collectMatrix(argv[2]);
        return rows.size() == 6 && writeCsv(argv[3], rows) ? 0 : 1;
    }
    if (argc != 5 || std::string(argv[1]) != "--check") {
        std::printf(
            "usage: %s --check <RightFork.mp4> <OneCycle.mp4> <baseline.csv>\n",
            argv[0]);
        return 1;
    }

    const std::vector<Stats> expected = readCsv(argv[4]);
    const std::vector<Stats> actual = collectMatrix(argv[3]);
    if (expected.size() != 6 || actual.size() != expected.size())
        return 2;
    for (size_t i = 0; i < expected.size(); ++i)
        if (!sameStats(expected[i], actual[i])) {
            std::printf("[FAIL] main baseline step=%d offset=%d\n",
                        actual[i].step, actual[i].offset);
            return 2;
        }

    for (int step = 1; step <= 3; ++step) {
        for (int offset = 0; offset < step; ++offset) {
            const JourneyStats right = collectJourney(
                argv[2], step, offset, true,
                2.4, 5.0, 9.6, 12.8);
            if (!right.reachedBranch ||
                right.entranceLeft != 0 ||
                right.exitLeft == 0) {
                std::printf(
                    "[FAIL] right step=%d offset=%d reached=%d entryLeft=%d exitLeft=%d\n",
                    step, offset, right.reachedBranch ? 1 : 0,
                    right.entranceLeft, right.exitLeft);
                return 2;
            }

            const JourneyStats main = collectJourney(
                argv[3], step, offset, false,
                21.5, 23.8, 6.6, 9.8);
            if (main.reachedBranch ||
                main.entranceLeft != 0 ||
                main.exitLeft != 0 ||
                main.branchAtMainExit) {
                std::printf(
                    "[FAIL] main step=%d offset=%d reached=%d entryLeft=%d exitLeft=%d\n",
                    step, offset, main.reachedBranch ? 1 : 0,
                    main.entranceLeft, main.exitLeft);
                return 2;
            }

            const JourneyStats stress = collectJourney(
                argv[3], step, offset, true,
                21.5, 25.5, 21.5, 25.5);
            if (stress.reachedBranch ||
                stress.entranceLeft != 0 ||
                stress.exitLeft != 0) {
                std::printf(
                    "[FAIL] stress step=%d offset=%d reached=%d left=%d\n",
                    step, offset, stress.reachedBranch ? 1 : 0,
                    stress.entranceLeft + stress.exitLeft);
                return 2;
            }
        }
    }
    std::printf("right fork lifecycle video matrix passed\n");
    return 0;
}
```

- [ ] **Step 4: Build and run all 18 video variants**

Run:

```bash
cmake --build test/build -j$(nproc) \
  --target test_right_fork_lifecycle_video
./test/build/bin/test_right_fork_lifecycle_video \
  --check \
  /home/orangepi/Videos/RightFork.mp4 \
  /home/orangepi/Videos/OneCycle.mp4 \
  test/baselines/one_cycle_main_exit.csv
```

Expected: `right fork lifecycle video matrix passed`.

If one variant fails, stop implementation and use its printed
`step/offset/window` to inspect the exact missed transition. Do not increase
debounce counts or relax exit geometry globally.

- [ ] **Step 5: Commit the speed replay**

```bash
git add test/test_right_fork_lifecycle_video.cpp
git commit -m "test: cover right fork lifecycle video speeds"
```

---

### Task 5: Complete Invalid-Frame Reset, Recovery, and Isolation Verification

**Files:**
- Modify: `src/perception/imgprocess.cpp`
- Modify: `test/test_right_fork_lifecycle_gate.cpp`

**Interfaces:**
- Consumes:
  - confirmed branch state from fixture pre-roll
  - `processFrameWithPpSegMask(frame, mask)`
  - `resetForkPhaseHunt()`
- Produces:
  - exactly two invalid-frame grace observations
  - third-invalid reset
  - reset-entry-point isolation
  - no candidate accumulation through invalid/non-candidate frames

- [ ] **Step 1: Add failing invalid/reset tests**

Add sorted fixture loading and this helper to
`test/test_right_fork_lifecycle_gate.cpp`:

```cpp
bool preRollRightBranch()
{
    resetWithRight();
    const std::filesystem::path dir =
        std::filesystem::path(XCAR_PROJECT_ROOT) /
        "test/img/right_fork_lifecycle/right";
    std::vector<std::string> paths;
    for (const auto& entry : std::filesystem::directory_iterator(dir))
        if (entry.path().extension() == ".png")
            paths.push_back(entry.path().string());
    std::sort(paths.begin(), paths.end());
    for (const std::string& path : paths) {
        const cv::Mat frame = cv::imread(path);
        if (frame.empty())
            return false;
        (void)processFrame(frame);
    }
    return getRightForkJourneyPhase() ==
           RightForkJourneyPhase::InRightBranch;
}
```

Add to `main()`:

```cpp
if (!preRollRightBranch())
    return 2;
const cv::Mat blankFrame(240, 320, CV_8UC3, cv::Scalar(0, 0, 0));
(void)processFrameWithPpSegMask(blankFrame, cv::Mat());
if (getRightForkJourneyPhase() !=
    RightForkJourneyPhase::InRightBranch)
    return 2;
(void)processFrameWithPpSegMask(blankFrame, cv::Mat());
if (getRightForkJourneyPhase() !=
    RightForkJourneyPhase::InRightBranch)
    return 2;
(void)processFrameWithPpSegMask(blankFrame, cv::Mat());
if (getRightForkJourneyPhase() != RightForkJourneyPhase::Idle)
    return 2;

if (!preRollRightBranch())
    return 2;
resetForkPhaseHunt();
if (getRightForkJourneyPhase() != RightForkJourneyPhase::Idle)
    return 2;

if (!preRollRightBranch())
    return 2;
(void)processFrame(cv::imread(
    fixture("shm_20260729_142231_503.png")));
if (getRightForkJourneyPhase() !=
    RightForkJourneyPhase::RightExitRepair)
    return 2;
const std::string ordinaryPath =
    std::string(XCAR_PROJECT_ROOT) +
    "/test/img/right_fork_lifecycle/right/frame_020.png";
const cv::Mat ordinaryFrame = cv::imread(ordinaryPath);
if (ordinaryFrame.empty())
    return 2;
(void)processFrame(ordinaryFrame);
(void)processFrame(ordinaryFrame);
if (getRightForkJourneyPhase() != RightForkJourneyPhase::Cooldown)
    return 2;
(void)processFrame(ordinaryFrame);
(void)processFrame(ordinaryFrame);
if (getRightForkJourneyPhase() != RightForkJourneyPhase::Idle)
    return 2;
```

Add `<algorithm>` and `<filesystem>`, then run:

```bash
cmake --build test/build -j$(nproc) \
  --target test_right_fork_lifecycle_gate
./test/build/bin/test_right_fork_lifecycle_gate
```

Expected: FAIL if failed-track frames do not yet reach the lifecycle observer.

- [ ] **Step 2: Observe each failed frame exactly once**

At the start of `processFrameInternal()` add:

```cpp
bool rightForkJourneyObserved = false;
```

Where the valid/invalid journey observation is made inside
`trackFromBoundary`, set:

```cpp
rightForkJourneyBeginFrame(
    journeyValid, dualHint, stillForkWidthIn,
    entrySingleLaneNear, rightTurnSingleLane);
rightForkJourneyObserved = true;
```

At the top of `failedTrack` add:

```cpp
if (!rightForkJourneyObserved) {
    rightForkJourneyBeginFrame(false, false, false, false, false);
    rightForkJourneyObserved = true;
}
```

This prevents the low-valid-row path from counting the same frame twice.

- [ ] **Step 3: Add recovery-to-cooldown handling**

In `rightForkJourneyBeginFrame()`, after the EnteringRight block, add:

```cpp
if (s.phase == RightForkJourneyPhase::RightExitRepair) {
    const bool ordinaryLane =
        singleLaneNear && !dualHint && !stillForkWidth;
    if (ordinaryLane) {
        if (++s.recoveryFrames >= 2)
            setRightForkJourneyPhase(
                RightForkJourneyPhase::Cooldown);
    } else {
        s.recoveryFrames = 0;
    }
} else if (s.phase == RightForkJourneyPhase::Cooldown) {
    const bool ordinaryLane =
        singleLaneNear && !dualHint && !stillForkWidth;
    if (ordinaryLane) {
        if (++s.recoveryFrames >= 2)
            resetRightForkJourney();
    } else {
        s.recoveryFrames = 0;
    }
}
```

The existing per-frame exit classifier returns `None` in both recovery states,
so their candidate count remains zero.

- [ ] **Step 4: Run focused perception tests**

Run:

```bash
cmake --build test/build -j$(nproc) --target \
  test_right_fork_lifecycle_gate \
  test_right_fork_entry_lifecycle \
  test_right_fork_exit_left_repair \
  test_fork_scene_samples \
  test_fork_exit_stable \
  test_fork_entry_left \
  test_fork_entry_width \
  test_fork_r_frames \
  test_road_straight_curve_samples

./test/build/bin/test_right_fork_lifecycle_gate
./test/build/bin/test_right_fork_entry_lifecycle
./test/build/bin/test_right_fork_exit_left_repair
./test/build/bin/test_fork_scene_samples
./test/build/bin/test_fork_entry_left
./test/build/bin/test_fork_entry_width
./test/build/bin/test_fork_r_frames
./test/build/bin/test_road_straight_curve_samples
```

Run `test_fork_exit_stable` with the known image argument:

```bash
./test/build/bin/test_fork_exit_stable \
  test/img/shm_20260729_142231_503.png \
  /tmp/right-fork-exit-final.png
```

Expected: every command exits `0`.

- [ ] **Step 5: Run the full speed matrix again**

```bash
./test/build/bin/test_right_fork_lifecycle_video \
  --check \
  /home/orangepi/Videos/RightFork.mp4 \
  /home/orangepi/Videos/OneCycle.mp4 \
  test/baselines/one_cycle_main_exit.csv
```

Expected: all step/offset variants pass and the main-road baseline matches
exactly.

- [ ] **Step 6: Run SIGN/control isolation tests**

Build and run:

```bash
cmake --build test/build -j$(nproc) --target \
  test_sign_strategy \
  test_sign_strategy_control \
  test_sign_session_isolation \
  test_ai_control_evidence

./test/build/bin/test_sign_strategy
./test/build/bin/test_sign_strategy_control
./test/build/bin/test_sign_session_isolation
./test/build/bin/test_ai_control_evidence
```

Expected: all commands exit `0`.

- [ ] **Step 7: Verify scope and worktree preservation**

Run:

```bash
git diff --check
git diff --name-only 60ada3a..HEAD
git status --short
```

Expected:

- implementation commits contain only `include/imgprocess.h`,
  `src/perception/imgprocess.cpp`, `test/CMakeLists.txt`, perception test
  sources, baseline CSV, and lifecycle PNG fixtures;
- no control, SIGN, UART, protocol, or config file appears in committed diffs;
- `configs/config.json` remains the user's sole unrelated worktree modification.

- [ ] **Step 8: Commit reset/recovery completion**

```bash
git add src/perception/imgprocess.cpp \
  test/test_right_fork_lifecycle_gate.cpp
git commit -m "test: cover right fork lifecycle resets"
```

- [ ] **Step 9: Request final code review**

Invoke the `requesting-code-review` skill against the complete implementation
range and address only findings that preserve the approved perception-only
scope. Re-run Steps 4-7 after any review fix.
