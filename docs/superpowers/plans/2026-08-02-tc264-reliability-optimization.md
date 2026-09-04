# TC264 Reliability Optimization Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Harden TC264 UART safety, telemetry consistency, IMU start readiness, Flash persistence, and diagnostics while preserving the current Xcar UART protocol.

**Architecture:** Keep production firmware changes localized to `/home/orangepi/Desktop/TC264/code` and `/home/orangepi/Desktop/TC264/user`. Add host-buildable tests under `/home/orangepi/Desktop/TC264/host_tests` for pure protocol and guard logic because the TASKING `cctc` target compiler is not installed on this Orange Pi.

**Tech Stack:** C99 firmware, TASKING/AURIX target project, GCC host tests, fixed 9-byte UART protocol, CRC16-CCITT.

## Global Constraints

- Do not change command IDs, payload sizes, byte order, baud rate, or CRC algorithm.
- Do not add automatic low-voltage hard stop behavior.
- Do not revert unrelated Xcar workspace changes.
- Use host tests for pure logic and report that target build requires TASKING `cctc`.

---

### Task 1: Host Test Harness

**Files:**
- Create: `/home/orangepi/Desktop/TC264/host_tests/test_tc264_reliability.c`
- Create: `/home/orangepi/Desktop/TC264/host_tests/run_tests.sh`

**Interfaces:**
- Produces: a GCC test executable that verifies pure firmware helper behavior.

- [ ] **Step 1: Write failing tests** for CRC, parser resync, valid-frame watchdog gating, upload snapshots, IMU timeout minimum samples, and Flash clamp helpers.
- [ ] **Step 2: Run tests** with `bash /home/orangepi/Desktop/TC264/host_tests/run_tests.sh`; expected failure because helpers are not implemented yet.

### Task 2: UART Safety

**Files:**
- Modify: `/home/orangepi/Desktop/TC264/code/Global.c`
- Modify: `/home/orangepi/Desktop/TC264/code/Global.h`
- Modify: `/home/orangepi/Desktop/TC264/code/send_data.c`
- Modify: `/home/orangepi/Desktop/TC264/user/cpu0_main.c`

**Interfaces:**
- Produces: `uart_note_valid_control_frame(uint8 cmd)`, RX diagnostic counters, and fixed-frame resync behavior.

- [ ] **Step 1: Move watchdog reset** from UART RX ISR to valid parsed control frames.
- [ ] **Step 2: Add parser resync** after CRC/length failure.
- [ ] **Step 3: Run host UART tests** and verify they pass.

### Task 3: Telemetry Snapshots

**Files:**
- Modify: `/home/orangepi/Desktop/TC264/code/Global.c`
- Modify: `/home/orangepi/Desktop/TC264/code/Global.h`
- Modify: `/home/orangepi/Desktop/TC264/code/encoder.c`
- Modify: `/home/orangepi/Desktop/TC264/code/send_data.c`

**Interfaces:**
- Produces: pending encoder and yaw snapshot variables used by `send()`.

- [ ] **Step 1: Snapshot left/right encoder deltas** in the 2 ms sampling path.
- [ ] **Step 2: Send snapshot values** from upload service instead of live motor fields.
- [ ] **Step 3: Run host snapshot tests** and verify they pass.

### Task 4: IMU Calibration Guard

**Files:**
- Modify: `/home/orangepi/Desktop/TC264/code/Global.c`
- Modify: `/home/orangepi/Desktop/TC264/code/Global.h`
- Modify: `/home/orangepi/Desktop/TC264/code/imu_fusion.c`
- Modify: `/home/orangepi/Desktop/TC264/code/imu_fusion.h`

**Interfaces:**
- Produces: `imu_calibration_warning` and timeout behavior that requires at least `IMU_MIN_CAL_SAMPLES`.

- [ ] **Step 1: Require minimum samples** before timeout can complete calibration.
- [ ] **Step 2: Record warning state** if timeout fires with too few samples.
- [ ] **Step 3: Run host IMU tests** and verify they pass.

### Task 5: Flash Guards

**Files:**
- Modify: `/home/orangepi/Desktop/TC264/code/Flash.c`
- Modify: `/home/orangepi/Desktop/TC264/code/Flash.h`

**Interfaces:**
- Produces: Flash metadata slots, CRC validation, and parameter clamp helpers.

- [ ] **Step 1: Write metadata** with magic, version, and CRC.
- [ ] **Step 2: Read only matching metadata** and keep defaults when invalid.
- [ ] **Step 3: Clamp accepted parameter ranges** after reading.
- [ ] **Step 4: Run host Flash tests** and verify they pass.

### Task 6: Low Voltage and Docs

**Files:**
- Modify: `/home/orangepi/Desktop/TC264/user/cpu0_main.c`
- Modify: `/home/orangepi/Desktop/Xcar/TC264.md`

**Interfaces:**
- Produces: restored low-voltage sampling and accurate documentation.

- [ ] **Step 1: Restore low-voltage sampling** in the main loop.
- [ ] **Step 2: Update TC264 docs** for real source paths, valid-frame watchdog, coherent telemetry, PRERUN/IMU behavior, and Flash guard behavior.
- [ ] **Step 3: Run final host tests and target compiler check**.
