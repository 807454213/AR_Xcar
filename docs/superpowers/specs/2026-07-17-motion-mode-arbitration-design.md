# Motion Mode Arbitration Design

## Goal

Ensure every `tc_process()` control frame sends at most one final `0x02`
motion-mode command, and ensure a request from a higher-priority control owner
cannot be overwritten by a later lower-priority request. In particular, the
first frame that enters `AVOID_PED` must not send `NORMAL` when leaving
`STABLE_SPEED`, `FAST_BACK`, or `RETURN_TRACK`.

## Root Cause

`MotionModeBatchGuard` currently defers every `setMotionMode()` call until the
end of `tc_process()`. The batch stores only the last request. During the batch,
`lastMotionMode()` still exposes the previous successfully sent value rather
than the pending value.

When a pedestrian appears while the previous state owns mode `5`, `7`, or `8`,
the pedestrian FSM first queues `STOP`. The old-state exit block then sees the
stale previous mode and queues `NORMAL`, replacing the pending `STOP`. The state
flag can consequently report `AVOID_PED` one frame before `STOP` is sent.

The same mechanism is structurally unsafe for every pair of `0x02` writers:
call order is being used as an implicit priority even though the project
already defines an explicit control-owner priority through `DriveState`.

## Considered Approaches

### 1. Remove batching

Send every request immediately. This makes `lastMotionMode()` current again,
but restores transient same-frame sequences such as `NORMAL` followed by
`STOP`. The TC264 can observe both commands, so this is rejected.

### 2. Add a pending-aware getter only

Expose the pending mode and use it in the three old-state exit checks. This is
the smallest repair for the observed pedestrian failure, but other later
writers can still overwrite earlier safety requests. This is suitable only as
an emergency patch and is not the selected design.

### 3. Explicit single-frame arbitration

Keep deferred single-frame sending, but associate every request with its
control owner and resolve by explicit priority. Equal-priority requests from
the same owner use the latest request so a legitimate phase transition, such
as pedestrian `STOP` to pedestrian `FAST`, can complete. This is the selected
design.

## Selected Design

### Request model

`UartCommander` will expose a batch-aware motion request API containing:

- requested `0x02` mode;
- `MotionModeOwner` identifying the controlling subsystem;
- diagnostic reason;
- optional force flag for existing explicit resend behavior.

The owner order follows the existing top-level control policy:

```text
Pedestrian > Vehicle > Sign > Gold > ReturnTrack
           > LeavingCar > FastBack > StableSpeed > Normal
```

Modes are not numerically prioritized. For example, vehicle avoidance requests
`NORMAL` mode but still outranks a gold slowdown request because the vehicle
owner has higher priority.

During a batch:

1. A request replaces the current winner when its owner has higher priority.
2. A lower-priority request is ignored.
3. An equal-owner request replaces the winner, preserving intentional
   same-subsystem phase transitions.
4. Identical requests remain harmless and are deduplicated at flush time.

Outside a batch, existing direct operations such as `startCar()` keep their
immediate behavior.

### State visibility and send ordering

The commander will expose the effective motion mode: the winning pending mode
inside a batch, otherwise the last successfully sent mode. Decisions made
inside `tc_process()` must use this effective value rather than stale sent
state.

The winning `0x02` request will be flushed before a changed `DriveState` is
reported through `0x09`. This ensures the serial stream does not announce
`AVOID_PED` before its motion-mode decision has been issued. The HUD continues
to show the last successfully sent `0x02` value.

If the UART send fails, the last successfully sent mode and HUD state are not
advanced. Existing UART failure counters and logging remain authoritative.

### Call-site ownership

All `0x02` requests made during `tc_process()` will declare an owner:

- pedestrian stop, pending-fast stop, and detour fast: `Pedestrian`;
- vehicle avoidance normal recovery: `Vehicle`;
- SIGN approach/stop: `Sign`; SIGN completion/loss recovery to `NORMAL`:
  `Normal`;
- gold slow/band entry or maintenance: `Gold`; gold exit/absence recovery to
  `NORMAL`: `Normal`;
- active return-track, leaving-car, fast-back, and stable-speed requests: their
  matching owners;
- previous-state exit cleanup to `NORMAL`: `Normal`.

No control thresholds, FSM transition counts, UART payloads, or TC264 behavior
change in this work.

## Safety Invariants

- A frame whose winning state is `AVOID_PED` and whose pedestrian phase is
  stopped or waiting for fast sends `0x02=1` in that same frame.
- A pedestrian request cannot be replaced by SIGN, gold, recovery, stable-speed,
  or generic NORMAL cleanup.
- A generic NORMAL cleanup never replaces any active element request.
- At most one actual `0x02` frame is emitted by one `tc_process()` call.
- `0x09` for a new drive state is sent only after the frame's winning `0x02`
  request has been flushed.

## Tests

Add regressions that first establish each previous state and then introduce a
side-entry pedestrian on a new AI source:

- `STABLE_SPEED -> AVOID_PED` sends `STOP` on the transition frame;
- `FAST_BACK -> AVOID_PED` sends `STOP` on the transition frame;
- `RETURN_TRACK -> AVOID_PED` sends `STOP` on the transition frame;
- the transition frame emits no intermediate or final `NORMAL` command;
- source-driven reused/unknown evidence continues holding the selected STOP;
- higher-priority requests win regardless of request order;
- equal-owner pedestrian phase updates retain their intended latest value;
- existing SIGN normal-resume, gold, vehicle, and pedestrian tests remain green.

## Scope

- Modify only `UartCommander`, `drive_control.cpp`, and focused control tests.
- Keep `UartCommander` as the sole discrete UART command exit.
- Do not change `configs/config.json`, detection thresholds, AI fusion,
  `DriveState` ordering, UART framing, or TC264 firmware.
- Do not add sleeps, retries, acknowledgements, or new runtime configuration.
