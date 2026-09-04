#!/usr/bin/env python3
import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CHECKED_FILES = [
    "include/trackcontrol.h",
    "src/app/Pipeline.cpp",
    "src/control/drive_control.cpp",
]
FORBIDDEN = re.compile(r"\btc_set_turn_deg\b|\bg_current_turn_deg\b")


violations = []
for relative in CHECKED_FILES:
    path = ROOT / relative
    for line_no, line in enumerate(path.read_text().splitlines(), 1):
        if FORBIDDEN.search(line):
            violations.append(f"{relative}:{line_no}: {line.strip()}")

if violations:
    print("Unused turn-degree control API must not remain:")
    print("\n".join(violations))
    sys.exit(1)

print("turn-degree policy passed")
