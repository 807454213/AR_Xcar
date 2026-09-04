#!/usr/bin/env python3
import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CHECKED_FILES = [
    "CMakeLists.txt",
    "test/CMakeLists.txt",
    "include/config.h",
    "src/io/config.cpp",
    "src/app/Pipeline.cpp",
    "src/control/drive_control.cpp",
    "README.md",
    "Xcar2.md",
    "PROJECT_STATUS.md",
    "configs/config.json",
    "configs/config_stable.json",
    "configs/config_fast.json",
]
FORBIDDEN = re.compile(
    r"\baiFusionEnabled\b|\baiDetSync\w*\b|\baiDetComp\w*\b|"
    r"\bAiDetectionHold\b|\bAiDetectionFrame\b|\bdet_sync\b|"
    r"\bai_detection_hold\b|\btest_det_sync\b|\btest_ai_detection_hold\b"
)


violations = []
for relative in CHECKED_FILES:
    path = ROOT / relative
    for line_no, line in enumerate(path.read_text().splitlines(), 1):
        if FORBIDDEN.search(line):
            violations.append(f"{relative}:{line_no}: {line.strip()}")

if violations:
    print("Removed AI fallback/det-sync path must not remain:")
    print("\n".join(violations))
    sys.exit(1)

print("AI fallback policy passed")
