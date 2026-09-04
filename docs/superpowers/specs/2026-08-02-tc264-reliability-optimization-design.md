# TC264 Reliability Optimization Design

## Goal

Improve the TC264 firmware reliability around UART safety, telemetry consistency, start readiness, parameter persistence, and operator diagnostics without changing the existing upper/lower command contract.

## Scope

The active firmware source tree is `/home/orangepi/Desktop/TC264`, with source files in `code/` and `user/`. The upper-level Xcar protocol remains fixed 9-byte UART frames at 460800 8N1, `len=4`, CRC16-CCITT over the first 7 bytes.

## Design

UART RX ISR only moves bytes into the ring buffer. Watchdog recovery happens only after a CRC-valid, supported command is parsed. The parser keeps the fixed-frame contract, but CRC failure now attempts a sliding resync to the next `0x55` already present in the parse buffer and records CRC/resync counters.

Encoder upload uses a coherent pending snapshot. The 2 ms interrupt copies left and right tick deltas into dedicated pending fields and marks them pending. The main loop sends both values from that same snapshot before clearing the pending flag, so one host-side sample cannot combine wheels from different ISR periods.

PRERUN continues to wait for IMU calibration before entering RUNNING. Calibration must gather at least `IMU_MIN_CAL_SAMPLES` static samples before timeout can complete it. If timeout happens with fewer samples, the firmware records a calibration warning and keeps waiting instead of silently declaring a normal calibration.

Flash persistence gains `magic`, `version`, and CRC fields in reserved slots. Reads only accept versioned data whose CRC matches. Read values are clamped to safe ranges before use. Old or corrupt Flash pages leave the compiled defaults in place.

Low-voltage sampling is restored in the main loop. It updates warning counters and buzzer behavior only; it does not introduce a new automatic hard stop.

## Testing

Because `cctc` is not available on the Orange Pi, verification uses host-compiled tests for pure logic and a target-toolchain availability check. Host tests cover CRC, parser resync, valid-frame watchdog gating, upload snapshots, IMU calibration timeout behavior, and Flash guard helpers.

## Documentation

`TC264.md` will be updated to use the actual `/home/orangepi/Desktop/TC264/code` and `/home/orangepi/Desktop/TC264/user` paths, and to describe the current PRERUN/IMU behavior.
