#!/usr/bin/env python3
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

FORBIDDEN = {
    "src/control/drive_control.cpp": [
        "obstacle_control_enabled",
        "g_current_lap",
        "g_current_lap == 1",
    ],
    "README.md": [
        "只在第一圈启用",
        "g_current_lap != 1",
    ],
    "PROJECT_STATUS.md": [
        "只在第一圈启用",
        "导致第一圈障碍逻辑被关闭",
        "g_current_lap != 1",
    ],
    "Xcar2.md": [
        "只在第一圈启用",
        "g_current_lap != 1",
    ],
}


def main() -> int:
    failures = []
    for rel, tokens in FORBIDDEN.items():
        text = (ROOT / rel).read_text(encoding="utf-8")
        for token in tokens:
            if token in text:
                failures.append(f"{rel}: forbidden obstacle lap-gate token {token!r}")

    if failures:
        for failure in failures:
            print(failure)
        return 1

    print("obstacle lap policy check passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
