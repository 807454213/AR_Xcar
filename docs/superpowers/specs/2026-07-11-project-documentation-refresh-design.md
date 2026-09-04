# Project documentation refresh

## Purpose

Keep `Xcar2.md` as the durable architecture and operating guide, and add
`PROJECT_STATUS.md` as a concise handoff snapshot for a new AI window.

## Xcar2.md

Update only sections affected by the current AR_Xcar work: asynchronous AI
fusion and one-frame unmatched hold, same-source SIGN OCR, V4 OCR thresholds
and aggregation, bounded SIGN decision fallback, configuration keys, focused
tests, and critical modification constraints. Preserve unrelated gold and
control documentation already present in the working tree.

## PROJECT_STATUS.md

Describe the current objective, active runtime configuration without secrets,
completed work, current source changes grouped by subsystem, verification
status, known risks, recommended next steps, and a short reading order. Clearly
separate source changes from generated build artifacts and avoid claiming that
unverified hardware behavior is proven.

## Safety

Do not include LLM credentials or copy generated binaries into documentation.
Do not modify `README.md`, `TC264.md`, source code, configuration, models, or
tests as part of this documentation task.
