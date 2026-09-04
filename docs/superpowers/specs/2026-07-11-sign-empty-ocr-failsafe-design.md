# SIGN Empty OCR Failsafe Design

## Problem

When SIGN OCR reaches `signOcrMaxAttempts` without producing any candidate,
`tc_on_ocr_result()` resets the aggregator and leaves the state in
`WaitingOcr`. The car remains stopped and repeats OCR windows indefinitely.

## Required Behavior

- A SIGN stop must always reach a terminal decision.
- High-confidence, stable, and context-candidate OCR behavior remains unchanged.
- If the configured attempt limit is reached with no candidate, choose the
  conservative `go_straight` action with `flag=0`.
- Complete through `tc_on_llm_result()` so fork direction, cooldown, SIGN state,
  and normal-speed restoration use the existing completion path.
- Do not retry another OCR window after the no-candidate limit is reached.

## Design

Expose a small control-layer completion helper for the no-candidate timeout.
`tc_on_ocr_result()` calls it when the aggregator reports `timedOut`. The helper
logs the fallback reason and invokes `tc_on_llm_result("go_straight", 0)`.

The fallback stays in the control layer because this timeout occurs before an
LLM request exists. It does not create a fake OCR payload or start an API call.
The existing `tc_on_llm_result()` path remains the single owner of SIGN
completion and UART speed restoration.

## Testing

Add a regression test that initializes SIGN control, triggers SIGN OCR, submits
empty OCR results up to `signOcrMaxAttempts`, and verifies that the flow no
longer requests OCR and reaches the existing completed decision behavior.

Keep the existing aggregator tests for candidate promotion and timeout. Run the
focused SIGN tests and rebuild the main target after the fix.

## Non-Goals

- Changing OCR thresholds, ROI geometry, or model files.
- Retrying while moving.
- Changing UART packet formats or bypassing `UartCommander`.
- Changing valid OCR or LLM fallback decisions.
