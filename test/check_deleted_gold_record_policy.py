#!/usr/bin/env python3
import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]

CHECKED_FILES = [
    "include/config.h",
    "include/trackcontrol.h",
    "src/control/drive_control.cpp",
    "src/io/config.cpp",
    "configs/config.json",
    "configs/config_stable.json",
    "configs/config_fast.json",
    "configs/config_medium.json",
    "test/CMakeLists.txt",
    "Xcar2.md",
]

FORBIDDEN = re.compile(
    r"goldOutsideTrackRecordMode|goldOutsideTrackRecordedCount|"
    r"tc_gold_record|gold_outside_record|"
    r"赛道外金币记录模式|赛道外金币数量记录"
)


violations = []
for relative in CHECKED_FILES:
    path = ROOT / relative
    if not path.exists():
        continue
    for line_no, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if FORBIDDEN.search(line):
            violations.append(f"{relative}:{line_no}: {line.strip()}")

if violations:
    print("Deleted gold outside record mode must not remain:")
    print("\n".join(violations))
    sys.exit(1)

print("deleted gold record policy passed")
