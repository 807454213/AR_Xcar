# SIGN Session Isolation Design

## Goal

Make every SIGN OCR/LLM decision flow an independent session so delayed OCR or
LLM results from an earlier road sign cannot modify a later road sign's state,
direction, speed, or text aggregation.

This design fixes audit findings 3, 4, 5, 6, 13, and 14:

- late LLM results crossing into the next SIGN session;
- late OCR results crossing into the next SIGN session;
- `tc_on_llm_result()` mutating `Done` before validating the callback;
- `Requesting` having no terminal timeout;
- approach-only SIGN loss leaving motion mode 3 latched;
- session state following a different detection target.

The track guarantees that two physical road signs are not visible at the same
time. No multi-target IoU tracker is required.

## Session Identity

Use a `uint64_t` SIGN session ID:

- `0` means no active session.
- A process-lifetime monotonic counter generates a nonzero ID when an Idle SIGN
  flow genuinely triggers OCR.
- Resetting `SignOcrState`, completing a session, or calling `tc_reset()` does
  not move the counter backward.
- Counter wrap skips `0`.

`SignOcrState` stores the current ID. `ControlResult` exposes it as
`ocr_session_id` whenever it requests SIGN OCR.

## Callback Contract

OCR and LLM callbacks must carry the originating session ID:

```cpp
bool tc_on_ocr_result(uint64_t session_id, int class_id,
                      const std::vector<TcOcrTextResult>& results);
bool tc_notify_ocr_started(uint64_t session_id, int class_id);
bool tc_notify_ocr_stopped(uint64_t session_id, int class_id);
bool tc_on_llm_result(uint64_t session_id,
                      const std::string& action, int flag);
bool tc_on_sign_timeout(uint64_t session_id);
uint64_t tc_current_sign_session_id();
```

The return value reports whether the callback was accepted.

An OCR callback is accepted only when:

- `class_id == SIGN`;
- `session_id` is nonzero and equals the current SIGN session;
- phase is `Requesting` or `WaitingOcr`.

OCR started/stopped notifications carry the processor's locked session ID and
are rejected before changing phase, center-error ownership, or motion mode when
that ID is stale.

An LLM or local-rule callback is accepted only when:

- `session_id` is nonzero and equals the current SIGN session;
- phase is `WaitingLlm`.

`tc_on_sign_timeout()` is the only completion path accepted from `Requesting`
or `WaitingOcr`. It requires the current nonzero session ID and any active SIGN
flow phase, then completes that session as `go_straight`. Pipeline uses
`tc_current_sign_session_id()` only to compare and retire pending request
records; it cannot mutate the session through that accessor.

Validation occurs before changing phase, OCR payload, LLM fields, fork
direction, center-error ownership, or motion mode. Rejected callbacks log one
stale-result diagnostic and have no side effects.

Tests and local callers must pass an explicit session ID to OCR result,
started/stopped notification, LLM result, and timeout APIs. Do not retain a
sessionless overload that can bypass the contract.

## Pipeline Ownership

The Pipeline keeps session metadata with every asynchronous operation.

The active OCR processor records the SIGN session ID it was created for.
Every result drained from that processor is delivered with that ID. If control
has moved to another session, the callback rejects it.

Replace the single global LLM future with pending request records:

```cpp
struct PendingSignLlm {
    uint64_t session_id = 0;
    std::vector<std::string> submitted_texts;
    std::future<ControlCommand> future;
};
```

Pipeline polling rules:

- A current session may have at most one pending LLM request.
- A new session can submit its own request even while an older session's future
  remains in flight.
- Ready results use the record's immutable `submitted_texts` for fallback, not
  the current control state's OCR text.
- Ready stale results are discarded and erased.
- An in-flight stale future remains only for nonblocking polling/reaping. It is
  never waited on or destroyed from the frame loop.

LLM object lifetime and shutdown cancellation are audit finding 7 and belong
to the next repair batch. This batch must not make frame-loop future cleanup
blocking.

## Target Updates

Only one physical SIGN can be visible at a time. During an active session, the
highest-confidence current SIGN detection may continue updating `last_box` and
source fid so the OCR ROI follows vehicle motion.

Target data is scoped by session ID. A detection after the previous session has
rearmed belongs to the newly generated session and cannot reuse old OCR or LLM
payload.

## Timeout And Loss Behavior

`phase_frames` advances for `Requesting`, `WaitingOcr`, and `WaitingLlm`.

Use `signLlmWaitMaxFrames` as the bounded decision timeout for all three phases:

- `Requesting` and `WaitingOcr` timeout call `tc_on_sign_timeout()` for the
  current session;
- `WaitingLlm` may use the same timeout completion path;
- accepted timeout completion restores normal speed and enters `Done`.

An approach-only SIGN can set mode 3 while phase remains `Idle`. If it is lost
for more than two frames, release mode 3 only when `UartCommander` still reports
mode 3. Do not overwrite a pedestrian, vehicle, RETURN_TRACK, or other owner's
motion mode.

## Decision Completion And Reset

When a current-session decision is accepted:

1. Transfer fork direction into the independent `ForkBiasState`.
2. Keep only `Done` and the session ID in `SignOcrState` so the same visible sign
   cannot retrigger.
3. Clear OCR texts, aggregation candidates, valid counts, temporary LLM action,
   and temporary LLM flag immediately.
4. Stop accepting OCR for that session and clear Pipeline OCR input/session
   state.
5. Erase a ready LLM record after consuming it. Mark any still-running record
   stale and reap it later without applying its result.

When the sign leaves and the existing lost/cooldown condition rearms the flow,
return to `Idle` with session ID `0`. The monotonic generator remains unchanged.

## Error Handling

- A stale callback is not an error requiring fallback; it is discarded.
- A current-session invalid LLM command uses the immutable texts stored in its
  pending request for local/straight fallback.
- OCR initialization or source-frame failure is bounded by the `Requesting`
  timeout and cannot leave the vehicle in an infinite approach state.
- Session cleanup is idempotent. Repeated stop notifications or stale ready
  futures cannot clear a newer session.

## Testing

Use explicit-return tests or compile affected targets with `-UNDEBUG`.

Required RED/GREEN cases:

1. Start session A, rearm and start B, then deliver A OCR; B phase and aggregator
   remain unchanged and callback returns false.
2. Deliver A LLM while B is active; B does not enter `Done`, speed and fork
   direction do not change, and callback returns false.
3. Deliver a matching-session LLM outside `WaitingLlm`; it is rejected before
   any state mutation.
4. Deliver A's OCR stopped notification while B is active; B phase and motion
   mode remain unchanged.
5. Keep A future pending, start B, and verify B can enqueue its own pending LLM
   record.
6. Complete A after B starts; Pipeline discards A and retains B.
7. Hold a session in `Requesting` past `signLlmWaitMaxFrames`; it accepts current
   session `go_straight`, enters `Done`, and restores mode 0.
8. Lose an approach-only SIGN for three frames; mode 3 returns to 0.
9. Lose an approach-only SIGN while another owner has changed the mode; SIGN
   cleanup leaves that mode untouched.
10. Run a current-session OCR-to-LLM-to-`Done` flow; fork direction remains
   correct after session payload is cleared.
11. Verify `Done` prevents retrigger while the same sign remains visible and
    rearming creates a greater nonzero session ID.

Run the focused SIGN tests, Pipeline/session tests, main build, and adjacent
gold/pedestrian/control regressions because the motion-mode release path is
shared.

## Non-Goals

- Do not implement multi-sign geometric association; the track guarantees only
  one visible physical SIGN.
- Do not change OCR scoring, aggregation thresholds, prompts, or local road-sign
  rules.
- Do not fix LLM raw-pointer lifetime, CURL synchronization, OCR queue teardown,
  OCR partial initialization, or OCR ROI data races in this batch; those are
  audit findings 7, 8, 9, and 25 in the next batch.
- Do not redesign global motion-mode ownership; only release approach mode 3
  conditionally. Finding 11 is handled in its own control-safety batch.
