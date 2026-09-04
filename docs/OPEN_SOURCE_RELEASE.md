# Open Source Release Checklist

Use this checklist before making the public repository or release package visible.

## Secrets

- Rotate or revoke any LLM/API credentials that were ever committed.
- Search the full working tree for keys:

```bash
rg -n "(api[_-]?key|secret|token|password|Authorization|Bearer|PRIVATE KEY)" -S .
```

- If a secret appears in Git history, clean history with `git filter-repo` or BFG, then force-push only after coordinating with collaborators.

## Repository Hygiene

- Remove generated build directories from Git tracking:

```bash
git rm -r --cached build build-test test/build
git rm -r --cached Position/slam_workspace/slam_all/build
```

- Review whether model files should be removed from Git tracking:

```bash
git ls-files "*.rknn" "*.onnx" "*.pdmodel" "*.pdiparams" "*.bin"
```

- Keep local-only files ignored: logs, recordings, private configs and generated binaries.

## Legal and Attribution

- Confirm the project license in `LICENSE`.
- Verify third-party licenses listed in `THIRD_PARTY_LICENSES.md`.
- Do not redistribute competition documents unless the organizer allows it.
- For each model artifact, document source, conversion steps and redistribution permission.

## Documentation

- Ensure `README.md` has build, run, test, hardware and model setup instructions.
- Keep `Xcar2.md` as the architecture and modification-boundary guide.
- Note which tests require RK3588, RKNN runtime, local image fixtures or real hardware.

## Safety

- Test protection stop and start sequence before real driving.
- Validate UART command policy with `python3 test/check_uart_protocol_policy.py`.
- Run real-car tests with wheels lifted or drivetrain disabled for first verification.
