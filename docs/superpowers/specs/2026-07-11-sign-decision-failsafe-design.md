# SIGN decision failsafe

## Problem

The car can remain stopped in front of a SIGN even though OCR text is printed.
Two loops have no terminal decision: context-only OCR attempts are discarded at
the attempt limit, and an invalid LLM response restarts OCR indefinitely. The
initial OCR aggregator also uses constructor defaults until a later reset.

## Design

Initialize the SIGN OCR aggregator from `TrackControlParams` in `tc_init`.
When OCR reaches its attempt limit with usable candidates, preserve and expose
the best payload as decision-ready instead of resetting the window. If an LLM
response is invalid, apply the existing local SIGN rule to the preserved OCR
payload; if no local rule matches, issue the conservative supported action
`go_straight`. Both fallback paths complete the normal `tc_on_llm_result` flow,
which sets fork bias and restores normal speed.

Do not change normal high-confidence OCR or valid LLM behavior. Keep retries
only for an OCR window that produced no usable candidate text.

## Tests

Add regression coverage for context-only candidates becoming available at the
attempt limit, configured aggregator initialization, and invalid-decision
fallback selection. Run the focused SIGN/OCR tests and the main build.
